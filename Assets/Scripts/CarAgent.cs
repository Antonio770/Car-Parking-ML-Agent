using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEngine;

public class CarParkingAgent : Agent {

    private CarController carController;
    private Collider carCollider;
    private Rigidbody carRb;

    [SerializeField] private float maxLinearVelocity = 10f;
    [SerializeField] private float maxAngularVelocity = 2f;
    [SerializeField] private Transform parkingSpotTransform;

    [SerializeField] private float angleThreshold = 10f;
    [SerializeField] private int fullParkingSpotCount = 2;
    [SerializeField] private float randomPosition = 25f;
    [SerializeField] private float randomRotation = 45f;

    [SerializeField] private GameObject fullParkingSpace;
    private List<GameObject> spawnedFullParkingSpots = new List<GameObject>();

    private List<Vector2Int> availablePositions = new List<Vector2Int>();

    private float previousDistance;
    private float initialDistanceToEmptySpot;

    private float previousAngle;
    private float initialAngleWithEmptySpot;

    private bool firstTimeTouchingSpot;

    public override void Initialize() {
        carController = GetComponent<CarController>();
        carCollider = GetComponentInChildren<Collider>();
        carRb = GetComponent<Rigidbody>();

        for (int i = 0; i <= 6; i++) {
            availablePositions.Add(new Vector2Int(i, -1));
            availablePositions.Add(new Vector2Int(i, 1));
        }

        previousDistance = 0f;
        previousAngle = 0f;
    }

    public override void OnEpisodeBegin() {
        float randPos= Random.Range(-randomPosition, -10f);
        float randRot = Random.Range(-randomRotation, randomRotation);

        float initialPosition = Academy.Instance.EnvironmentParameters.GetWithDefault("initial_position", randPos);
        float initialRotation = Academy.Instance.EnvironmentParameters.GetWithDefault("initial_rotation", randRot);

        transform.localPosition = new Vector3(0f, 0f, initialPosition);
        transform.localRotation = Quaternion.Euler(0f, initialRotation, 0f);

        carRb.linearVelocity = Vector3.zero;
        carRb.angularVelocity = Vector3.zero;

        Vector2Int pos = availablePositions[Random.Range(0, availablePositions.Count)];

        parkingSpotTransform.localPosition = new Vector3(pos.y * 8f, 0f, pos.x * 4.5f);
        parkingSpotTransform.localRotation = Quaternion.Euler(new Vector3(0f, pos.y * 90f, 0f));

        initialDistanceToEmptySpot = Vector3.Distance(transform.localPosition, parkingSpotTransform.localPosition);
        previousDistance = initialDistanceToEmptySpot;

        initialAngleWithEmptySpot = Quaternion.Angle(transform.localRotation, parkingSpotTransform.localRotation);
        previousAngle = initialAngleWithEmptySpot;

        fullParkingSpotCount = (int) Academy.Instance.EnvironmentParameters.GetWithDefault("number_of_full_spots", fullParkingSpotCount);
        SpawnOrReuseFullParkingSpots();

        firstTimeTouchingSpot = true;
    }

    private void FinishEpisode() {
        foreach (GameObject spot in spawnedFullParkingSpots) {
            if (spot != null) {
                spot.SetActive(false);
            }
        }

        EndEpisode();
    }

    public override void CollectObservations(VectorSensor sensor) {
        sensor.AddObservation(transform.InverseTransformDirection(carRb.linearVelocity) / maxLinearVelocity);
        sensor.AddObservation(transform.InverseTransformDirection(carRb.angularVelocity) / maxAngularVelocity);
        sensor.AddObservation(Vector3.SignedAngle(transform.forward, parkingSpotTransform.forward, Vector3.up) / 180f);
    }

    public override void OnActionReceived(ActionBuffers actions) {
        ActionSegment<float> continuousActions = actions.ContinuousActions;
        ActionSegment<int> discreteActions = actions.DiscreteActions;

        float verticalInput = continuousActions[0];
        float horizontalInput = continuousActions[1];
        bool brakeInput = discreteActions[0] == 1 ? true : false;

        carController.SetInputs(verticalInput, horizontalInput, brakeInput);

        float currentDistance = Vector3.Distance(transform.localPosition, parkingSpotTransform.localPosition);
        float distanceDiff = previousDistance - currentDistance;
        previousDistance = currentDistance;

        float currentAngle = Quaternion.Angle(transform.localRotation, parkingSpotTransform.localRotation);
        float angleDiff = previousAngle - currentAngle;
        previousAngle = currentAngle;

        // Sigmoid function to calculate distance and angular weights
        // The closer the car is to the parking spot, the more important the angle
        float dist = currentDistance;
        float midpoint = 12f;
        float steepness = 5f;

        float angleWeight = 1f / (1f + Mathf.Exp(steepness * (dist - midpoint)));
        float distanceWeight = Mathf.Clamp(1 - angleWeight, 0.2f, 1f);

        // A maximum of 1 point for moving the entire episode
        if (carRb.linearVelocity.magnitude > 0.01f) {
            AddReward(1f / MaxStep);
        }

        // A maximum of 1 point for reaching the empty spot
        AddReward(distanceDiff / initialDistanceToEmptySpot * distanceWeight);

        // A maximum of 1 point for getting the right angle
        AddReward(angleDiff / initialAngleWithEmptySpot * angleWeight);

        // A maximum of 0.2 penalty for being too slow
        AddReward(-0.2f / MaxStep);
    }

    public void OnTriggerEnter(Collider other) {
        if (other.CompareTag("Wall") || other.CompareTag("FullSpot")) {
            AddReward(-1f);
            FinishEpisode();
        }

        // Only award the agent the first time it hits the empty spot
        if (other.CompareTag("EmptySpot") && firstTimeTouchingSpot) {
            AddReward(0.5f);
            firstTimeTouchingSpot = false;
        }
    }

    public void OnTriggerStay(Collider other) {
        if (other.CompareTag("EmptySpot") && IsCarFullyParked(other)) {
            AddReward(1.5f);
            FinishEpisode();
        }

        if (other.CompareTag("WhiteLine")) {
            AddReward(-0.2f / MaxStep);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut) {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        ActionSegment<int> discreteActions = actionsOut.DiscreteActions;

        continuousActions[0] = Input.GetAxis("Vertical");
        continuousActions[1] = Input.GetAxis("Horizontal");
        discreteActions[0] = Input.GetKey(KeyCode.Space) == true ? 1 : 0;
    }

    private bool IsCarFullyParked(Collider parkingSpotCollider) {
        Bounds carBounds = carCollider.bounds;
        Bounds spotBounds = parkingSpotCollider.bounds;
        float angle = Quaternion.Angle(transform.localRotation, parkingSpotTransform.localRotation);

        bool inside = spotBounds.Contains(carBounds.min) && spotBounds.Contains(carBounds.max);
        bool aligned = angle < angleThreshold || (180f - angle) < angleThreshold;
        bool stopped = carRb.linearVelocity.magnitude < 0.01f;

        return inside && aligned && stopped;
    }

    private void SpawnOrReuseFullParkingSpots() {
        List<Vector2Int> freePositions = new List<Vector2Int>(availablePositions);

        freePositions.Remove(new Vector2Int(
            Mathf.RoundToInt(parkingSpotTransform.transform.localPosition.z / 4.5f),
            Mathf.RoundToInt(parkingSpotTransform.transform.localPosition.x / 8f)
        ));

        for (int i = 0; i < fullParkingSpotCount; i++) {
            if (i >= spawnedFullParkingSpots.Count) {
                GameObject newSpot = Instantiate(fullParkingSpace, transform.parent);
                spawnedFullParkingSpots.Add(newSpot);
            }

            int randIndex = Random.Range(0, freePositions.Count);
            Vector2Int pos = freePositions[randIndex];
            freePositions.RemoveAt(randIndex);

            Vector3 newPosition = new Vector3(pos.y * 8f, 0f, pos.x * 4.5f);
            Quaternion newRotation = Quaternion.Euler(0f, pos.y * 90f, 0f);

            spawnedFullParkingSpots[i].transform.localPosition = newPosition;
            spawnedFullParkingSpots[i].transform.localRotation = newRotation;

            spawnedFullParkingSpots[i].SetActive(true);
        }

        for (int i = fullParkingSpotCount; i < spawnedFullParkingSpots.Count; i++) {
            spawnedFullParkingSpots[i].SetActive(false);
        }
    }

}

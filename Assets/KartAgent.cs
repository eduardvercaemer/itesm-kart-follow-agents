using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using KartGame.KartSystems;

public class KartAgent : Agent, IInput
{
    bool accelerate = false;
    bool brake = false;
    float turnInput = 0.0f;

    public override void Initialize()
    {
    }

    public override void OnEpisodeBegin()
    {
    }

    public override void CollectObservations(VectorSensor sensor)
    {
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        turnInput = Mathf.Clamp(actions.ContinuousActions[0], -1.0f, 1.0f);
        accelerate = Mathf.Clamp(actions.ContinuousActions[1], -1.0f, 1.0f) > 0.0f;
        brake = Mathf.Clamp(actions.ContinuousActions[2], -1.0f, 1.0f) > 0.0f;

        CalculateReward();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetButton("Accelerate") ? 1.0f : -1.0f;
        continuousActionsOut[2] = Input.GetButton("Brake") ? 1.0f : -1.0f;
    }

    public InputData GenerateInput()
    {
        return new InputData
        {
            Accelerate = accelerate,
            Brake = brake,
            TurnInput = turnInput
        };
    }

    private void OnCollisionEnter(Collision collision) { }

    private void OnTriggerEnter(Collider other) { }

    private void CalculateReward()
    {
    }
}
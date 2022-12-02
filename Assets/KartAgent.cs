using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using KartGame.KartSystems;

public class KartAgent : Agent, IInput
{
    public Transform nextKart;

    // ---- RUNTIME
    int amountOfRays = 15;
    float startAngle = -90.0f;
    float endAngle = 90.0f;
    bool accelerate = false;
    bool brake = false;
    float turnInput = 0.0f;
    Rigidbody m_RigidBody;
    Vector3 startPosition;
    Quaternion startRotation;
    float behaviour = 0.0f;

    // ---- DEBUG
    string message;
    UnityEngine.Color messageColor;
    Vector3 messagePosition;

    // ---- CONFIG
    float targetLeaderSpeed;
    float targetFollowerDistance;
    float targetFollowerSpeed;
    float turnPunishmentFactor;
    float leaderSpeedPunihsmentFactor;
    float leaderCenteringPunishmentFactor;
    float leaderAlignmentPunishmentFactor;
    float followerAnglePunishmentFactor;
    float followerDistancePunishmentFactor;
    float followerSpeedPunishmentFactor;

    public override void Initialize()
    {
        m_RigidBody = GetComponent<Rigidbody>();
        startPosition = transform.position;
        startRotation = transform.rotation;
    }

    public override void OnEpisodeBegin()
    {
        transform.position = startPosition;
        transform.rotation = startRotation;
        m_RigidBody.velocity = Vector3.zero;
        m_RigidBody.angularVelocity = Vector3.zero;

        targetLeaderSpeed = Academy.Instance.EnvironmentParameters.GetWithDefault("target_leader_speed", 10.0f);
        targetFollowerDistance = Academy.Instance.EnvironmentParameters.GetWithDefault("target_follower_distance", 10.0f);
        targetFollowerSpeed = Academy.Instance.EnvironmentParameters.GetWithDefault("target_follower_speed", 12.0f);

        turnPunishmentFactor = Academy.Instance.EnvironmentParameters.GetWithDefault("turn_punishment_factor", 0.1f);
        leaderSpeedPunihsmentFactor = Academy.Instance.EnvironmentParameters.GetWithDefault("leader_speed_punishment_factor", 0.9f);
        leaderCenteringPunishmentFactor = Academy.Instance.EnvironmentParameters.GetWithDefault("leader_centering_punishment_factor", 4.5f);
        leaderAlignmentPunishmentFactor = Academy.Instance.EnvironmentParameters.GetWithDefault("leader_alignment_punishment_factor", 4.5f);

        followerAnglePunishmentFactor = Academy.Instance.EnvironmentParameters.GetWithDefault("follower_angle_punishment_factor", 4.0f);
        followerDistancePunishmentFactor = Academy.Instance.EnvironmentParameters.GetWithDefault("follower_distance_punishment_factor", 5.0f);
        followerSpeedPunishmentFactor = Academy.Instance.EnvironmentParameters.GetWithDefault("follower_speed_punishment_factor", 1.0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // rays = 15 * 4 = 60
        float angle = startAngle;
        float angleStep = (endAngle - startAngle) / amountOfRays;
        for (int i = 0; i < amountOfRays; i++)
        {
            Vector3 direction = Quaternion.AngleAxis(angle, transform.up) * transform.forward;
            direction.y = 0.0f;
            direction.Normalize();
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, 100.0f))
            {
                sensor.AddObservation(hit.distance / 100.0f);
                sensor.AddObservation(hit.normal);
                Debug.DrawRay(transform.position, direction * hit.distance, Color.red / 2.0f);
                Debug.DrawRay(hit.point, hit.normal, Color.green / 2.0f);
            }
            else
            {
                sensor.AddObservation(0.0f);
                sensor.AddObservation(Vector3.zero);
            }
            angle += angleStep;
        }

        // rigidbody = 4
        sensor.AddObservation(m_RigidBody.velocity);
        sensor.AddObservation(m_RigidBody.velocity.magnitude);

        // next kart = 6
        if (nextKart != null)
        {
            Vector3 relativeDirection = transform.InverseTransformDirection(nextKart.position - transform.position);
            sensor.AddObservation(relativeDirection);
            sensor.AddObservation(relativeDirection.magnitude);
            sensor.AddObservation(nextKart.GetComponent<Rigidbody>().velocity.magnitude);
            sensor.AddObservation(Vector3.SignedAngle(transform.forward, nextKart.position - transform.position, transform.up) / 180.0f);
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0.0f);
            sensor.AddObservation(0.0f);
            sensor.AddObservation(0.0f);
        }

        // config = 4
        sensor.AddObservation(targetLeaderSpeed);
        sensor.AddObservation(targetFollowerDistance);
        sensor.AddObservation(targetFollowerSpeed);
        sensor.AddObservation(behaviour);

        // total = 74
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

    private void CalculateReward()
    {
        if (nextKart != null) /* follower */
        {
            RaycastHit hit;
            float distance = Vector3.Distance(transform.position, nextKart.position);
            if (Physics.Raycast(transform.position, nextKart.position - transform.position, out hit, distance))
            {
                LeaderBehaviour();
            }
            else
            {
                FollowerBehaviour();
            }
        }
        else /* leader */
        {
            LeaderBehaviour();
        }

        QuadraticPunishment(
            turnInput,
            0.0f,
            1.0f,
            turnPunishmentFactor
        );
    }

    private void LeaderBehaviour()
    {
        behaviour = 1.0f;
        Debug.DrawRay(transform.position, transform.forward * 10.0f, Color.blue);

        // ==== SPEED
        float speed_r;
        float speed = m_RigidBody.velocity.magnitude;
        if (speed < targetLeaderSpeed)
        {
            speed_r = -QuadraticPunishment(
                speed,
                targetLeaderSpeed,
                targetLeaderSpeed,
                leaderSpeedPunihsmentFactor
            );
        }
        else
        {
            speed_r = LinearReward(
                speed,
                targetLeaderSpeed,
                targetLeaderSpeed,
                leaderSpeedPunihsmentFactor
            );
        }

        // ==== ALIGNMENT & CENTERING
        float leftAngle = -90.0f;
        float rightAngle = 90.0f;
        RaycastHit leftHit, rightHit;
        Vector3 leftDirection = Quaternion.AngleAxis(leftAngle, transform.up) * transform.forward;
        Vector3 rightDirection = Quaternion.AngleAxis(rightAngle, transform.up) * transform.forward;
        leftDirection.y = 0.0f;
        rightDirection.y = 0.0f;
        leftDirection.Normalize();
        rightDirection.Normalize();
        float totalAngle, distanceDiff;
        if (
            Physics.Raycast(transform.position, leftDirection, out leftHit, 100.0f) &&
            Physics.Raycast(transform.position, rightDirection, out rightHit, 100.0f)
        )
        {
            Debug.DrawRay(transform.position, leftHit.point - transform.position, Color.green);
            Debug.DrawRay(transform.position, rightHit.point - transform.position, Color.green);

            // get angle with normals
            float leftAngleWithNormal = Vector3.SignedAngle(leftHit.normal, leftDirection, transform.up);
            float rightAngleWithNormal = Vector3.SignedAngle(rightHit.normal, rightDirection, transform.up);
            totalAngle = Mathf.Abs(leftAngleWithNormal) + Mathf.Abs(rightAngleWithNormal);

            // get distances
            float leftDistance = leftHit.distance;
            float rightDistance = rightHit.distance;
            distanceDiff = Mathf.Abs(leftDistance - rightDistance);
        }
        else
        {
            totalAngle = 240.0f;
            distanceDiff = 0.0f;
        }

        float center_r = -QuadraticPunishment(
            totalAngle,
            360.0f,
            120.0f,
            leaderAlignmentPunishmentFactor
        );
        float distance_r = -QuadraticPunishment(
            distanceDiff,
            0.0f,
            8.0f,
            leaderCenteringPunishmentFactor
        );

        float total_r = speed_r + center_r + distance_r;
        DrawDebugText(
            $"s:{speed} r:{speed_r} a:{totalAngle} r:{center_r} d:{distanceDiff} r:{distance_r} t:{total_r}",
            transform.position,
            Color.red
        );
    }

    private void FollowerBehaviour()
    {
        behaviour = 0.0f;
        Debug.DrawRay(transform.position, nextKart.position - transform.position, Color.green);

        // ==== ANGLE
        float angle = Vector3.SignedAngle(transform.forward, nextKart.position - transform.position, transform.up);
        float angle_r = -QuadraticPunishment(
            angle,
            0.0f,
            180.0f,
            followerAnglePunishmentFactor
        );

        // ==== DISTANCE
        float distance = Vector3.Distance(transform.position, nextKart.position);
        float distance_r;
        if (distance < targetFollowerDistance)
        {
            distance_r = -QuadraticPunishment(
                distance,
                targetFollowerDistance,
                targetFollowerDistance,
                followerDistancePunishmentFactor
            );
        }
        else
        {
            distance_r = LinearReward(
                distance,
                targetFollowerDistance,
                targetFollowerDistance * 2,
                followerDistancePunishmentFactor
            );
        }

        // ==== SPEED
        float speed_r;
        float speed = m_RigidBody.velocity.magnitude;
        if (speed < targetFollowerSpeed)
        {
            speed_r = -QuadraticPunishment(
                speed,
                targetFollowerSpeed,
                targetFollowerSpeed,
                followerSpeedPunishmentFactor
            );
        }
        else
        {
            speed_r = LinearReward(
                speed,
                targetFollowerSpeed,
                targetFollowerSpeed,
                followerSpeedPunishmentFactor
            );
        }

        float total_r = angle_r + distance_r;
        DrawDebugText(
            $"a:{angle} r:{angle_r} d:{distance} r:{distance_r} s:{speed} r:{speed_r} t:{total_r}",
            (nextKart.position + transform.position) / 2.0f,
            Color.blue
        );
    }

    private void DrawDebugText(string text, Vector3 position, Color color)
    {
        message = text;
        messagePosition = position;
        messageColor = color;
    }

    public void OnDrawGizmos()
    {
        drawString(message, messagePosition, messageColor);
    }

    static public void drawString(string text, Vector3 worldPos, Color? colour = null)
    {
#if UNITY_EDITOR
        UnityEditor.Handles.BeginGUI();

        var restoreColor = GUI.color;

        if (colour.HasValue) GUI.color = colour.Value;
        var view = UnityEditor.SceneView.currentDrawingSceneView;
        Vector3 screenPos = view.camera.WorldToScreenPoint(worldPos);

        if (screenPos.y < 0 || screenPos.y > Screen.height || screenPos.x < 0 || screenPos.x > Screen.width || screenPos.z < 0)
        {
            GUI.color = restoreColor;
            UnityEditor.Handles.EndGUI();
            return;
        }

        Vector2 size = GUI.skin.label.CalcSize(new GUIContent(text));
        GUI.Label(new Rect(screenPos.x - (size.x / 2), -screenPos.y + view.position.height + 4, size.x, size.y), text);
        GUI.color = restoreColor;
        UnityEditor.Handles.EndGUI();
#endif
    }

    private float LinearReward(
        float value,
        float target,
        float maxDiff,
        float factor
    )
    {
        // a diff of zero means full reward,
        // a diff of maxDiff means no reward

        float diff = Mathf.Abs(value - target);
        float reward;
        if (diff > maxDiff)
        {
            reward = -factor * (diff - maxDiff);
        }
        else
        {
            reward = 0.0f;
        }

        AddReward(reward);
        return reward;
    }

    private float QuadraticPunishment(
        float value,
        float target,
        float maxDiff,
        float factor
    )
    {
        // a diff of zero means no punishment,
        // a diff of maxDiff means full punishment

        float diff = Mathf.Abs(value - target);
        float punishment;
        if (diff > maxDiff)
        {
            diff = maxDiff;
        }

        // 0 -> 0
        // maxDiff -> factor

        punishment = factor * diff * diff / (maxDiff * maxDiff);
        AddReward(-punishment);
        return punishment;
    }
}
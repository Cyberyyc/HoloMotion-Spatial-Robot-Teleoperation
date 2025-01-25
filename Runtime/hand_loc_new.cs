using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

public class hand_loc_new : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/arm/arm_follow";
    private ROSConnection ros;

    [Header("Control Settings")]
    public float pinchThreshold = 0.02f;
    public float toggleCooldown = 1.0f;

    [Header("Stability Settings")]
    [Tooltip("阻尼系数 (0-1)，越小阻尼感越强。建议 0.15")]
    public float posSmoothing = 0.15f;
    [Tooltip("跳变拦截阈值 (米)。防止握拳瞬间的坐标闪变")]
    public float jumpThreshold = 0.03f;

    [Header("Debug Settings")]
    [Tooltip("Remoting 时在 Unity 控制台打印位姿")]
    public bool enablePoseDebug = true;
    [Tooltip("控制台输出间隔 (秒)")]
    public float debugInterval = 0.2f;

    private bool isControlling = false;
    private bool isFirstFrame = false;
    private float lastToggleTime = 0f;

    private Vector3 rightHandStartPos;
    private Quaternion rightHandStartRot;
    private Quaternion systemStartHeadRot = Quaternion.identity;

    private Vector3 smoothedLocalDelta = Vector3.zero;
    private Vector3 lastRawLocalPos = Vector3.zero;

    private float debugTimer = 0f;
    private Arm_moveMsg armMsg;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Arm_moveMsg>(topicName);
        armMsg = new Arm_moveMsg();
        ResetMessage();
    }

    void Update()
    {
        HandleLeftHandSwitch();

        if (isControlling)
        {
            HandleRightHandTracking();
        }
        else
        {
            // 未控制时持续发 reset 保活，避免 ROS 端收不到首包
            SendRosMessage(Vector3.zero, Quaternion.identity, true);
        }
    }

    bool GetStablePose(out MixedRealityPose pose)
    {
        return HandJointUtils.TryGetJointPose(
            TrackedHandJoint.MiddleMetacarpal, Handedness.Right, out pose);
    }

    void HandleLeftHandSwitch()
    {
        if (Time.time - lastToggleTime < toggleCooldown) return;

        MixedRealityPose leftThumb, leftMiddle;
        if (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Left, out leftThumb) &&
            HandJointUtils.TryGetJointPose(TrackedHandJoint.MiddleTip, Handedness.Left, out leftMiddle))
        {
            if (Vector3.Distance(leftThumb.Position, leftMiddle.Position) < pinchThreshold)
            {
                ToggleControlState();
                lastToggleTime = Time.time;
            }
        }
    }

    void ToggleControlState()
    {
        isControlling = !isControlling;

        if (isControlling)
        {
            InitializeRightHandAnchor();
            isFirstFrame = true;
            SendRosMessage(Vector3.zero, Quaternion.identity, true);
            Debug.Log("<color=green><b>【系统启动】</b>已对齐头部参考系</color>");
        }
        else
        {
            smoothedLocalDelta = Vector3.zero;
            lastRawLocalPos = Vector3.zero;
            SendRosMessage(Vector3.zero, Quaternion.identity, true);
            Debug.Log("<color=red><b>【系统暂停】</b></color>");
        }
    }

    void InitializeRightHandAnchor()
    {
        MixedRealityPose rightPose;
        if (GetStablePose(out rightPose))
        {
            rightHandStartPos = rightPose.Position;
            rightHandStartRot = rightPose.Rotation;

            Vector3 headEuler = Camera.main.transform.eulerAngles;
            systemStartHeadRot = Quaternion.Euler(0, headEuler.y, 0);
        }
    }

    void HandleRightHandTracking()
    {
        MixedRealityPose currentPose;
        if (GetStablePose(out currentPose))
        {
            Vector3 worldDelta = currentPose.Position - rightHandStartPos;
            Vector3 rawLocalPos = Quaternion.Inverse(systemStartHeadRot) * worldDelta;

            if (isFirstFrame)
            {
                smoothedLocalDelta = rawLocalPos;
                lastRawLocalPos = rawLocalPos;
                isFirstFrame = false;
            }
            else
            {
                float dist = Vector3.Distance(rawLocalPos, lastRawLocalPos);
                if (dist < jumpThreshold)
                {
                    smoothedLocalDelta = Vector3.Lerp(smoothedLocalDelta, rawLocalPos, posSmoothing);
                }
                lastRawLocalPos = rawLocalPos;
            }

            Vector3 deltaPos = smoothedLocalDelta * 0.5f;

            Quaternion currentInHead = Quaternion.Inverse(systemStartHeadRot) * currentPose.Rotation;
            Quaternion startInHead = Quaternion.Inverse(systemStartHeadRot) * rightHandStartRot;
            Quaternion deltaRot = currentInHead * Quaternion.Inverse(startInHead);

            SendRosMessage(deltaPos, deltaRot, false);
            LogPoseToConsole(deltaPos, deltaRot);
        }
    }

    void LogPoseToConsole(Vector3 pos, Quaternion rot)
    {
        if (!enablePoseDebug || Time.time - debugTimer < debugInterval) return;

        debugTimer = Time.time;
        Vector3 euler = rot.eulerAngles;
        Debug.Log(
            $"[HandPose] Pos(x,y,z): ({pos.x:F3}, {pos.y:F3}, {pos.z:F3}) | " +
            $"Quat(x,y,z,w): ({rot.x:F3}, {rot.y:F3}, {rot.z:F3}, {rot.w:F3}) | " +
            $"Euler(x,y,z): ({NormalizeAngle(euler.x):F1}, {NormalizeAngle(euler.y):F1}, {NormalizeAngle(euler.z):F1})");
    }

    void SendRosMessage(Vector3 pos, Quaternion rot, bool isInit)
    {
        armMsg.init_pose = isInit;
        armMsg.position.x = pos.x;
        armMsg.position.y = pos.y;
        armMsg.position.z = pos.z;

        // 旋转增量：直接发四元数，不要转 eulerAngles
        armMsg.orientation.x = rot.x;
        armMsg.orientation.y = rot.y;
        armMsg.orientation.z = rot.z;
        armMsg.orientation.w = rot.w;

        ros.Publish(topicName, armMsg);
    }

    void ResetMessage()
    {
        armMsg.init_pose = false;
        armMsg.position.x = 0;
        armMsg.position.y = 0;
        armMsg.position.z = 0;
        armMsg.orientation.x = 0;
        armMsg.orientation.y = 0;
        armMsg.orientation.z = 0;
        armMsg.orientation.w = 1;
    }

    float NormalizeAngle(float angle)
    {
        angle %= 360f;
        if (angle > 180f) angle -= 360f;
        if (angle < -180f) angle += 360f;
        return angle;
    }
}

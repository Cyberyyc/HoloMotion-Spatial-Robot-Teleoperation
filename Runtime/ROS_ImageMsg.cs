using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UI;

public class ROS_ImageMsg : MonoBehaviour
{
    [Header("UI Components")]
    public RawImage ARMImagePanel;
    private Texture2D ARMTexture;

    [Header("显示设置")]
    [Tooltip("你希望在屏幕上看到的画面宽度 (像素)，高度会自动计算")]
    public float displayWidth = 480f; // 建议设为 480 或 360，根据你的UI布局需求

    [Header("强力旋转设置")]
    [Tooltip("勾选此项以解决ROS图像默认上下颠倒的问题")]
    public bool flipVertical = true;

    [Tooltip("旋转角度 (90, -90, 180 等)")]
    [Range(-360, 360)]
    public float rotationAngle = -90f; // 逆时针安装通常填 -90 或 90

    [Tooltip("是否自动锁定长宽比（防止旋转后画面被压扁）")]
    public bool autoLockAspectRatio = true;

    // 状态记录
    private float lastRotationAngle;
    private float lastDisplayWidth;
    private bool isInitialized = false;

    void Start()
    {
        ViewConnect();
    }

    // 【核心机制】使用 LateUpdate
    // 它的执行顺序在 LayoutGroup 计算之后。无论父物体怎么强制归零，
    // 这里都会在渲染前最后一刻把角度再次改回来。
    void LateUpdate()
    {
        if (ARMImagePanel == null) return;

        // 1. 强力应用旋转 (解决Layout Group重置问题)
        Vector3 currentRot = ARMImagePanel.rectTransform.localEulerAngles;
        // 只有当角度偏差大时才赋值，避免浮点抖动
        if (Mathf.Abs(currentRot.z - rotationAngle) > 0.01f)
        {
            ARMImagePanel.rectTransform.localEulerAngles = new Vector3(0, 0, rotationAngle);
        }

        // 2. 强力应用翻转 (UV Rect)
        if (flipVertical)
            ARMImagePanel.uvRect = new Rect(0, 1, 1, -1);
        else
            ARMImagePanel.uvRect = new Rect(0, 0, 1, 1);

        // 3. 实时检测并修正长宽比
        if (ARMTexture != null && autoLockAspectRatio)
        {
            // 如果旋转角度或目标宽度发生变化，或者刚初始化，则更新尺寸
            if (lastRotationAngle != rotationAngle || lastDisplayWidth != displayWidth || !isInitialized)
            {
                UpdateAspectRatio();
                lastRotationAngle = rotationAngle;
                lastDisplayWidth = displayWidth;
                isInitialized = true;
            }
        }
    }

    public void ViewConnect()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<CompressedImageMsg>(
            "/camera/color/image_raw/compressed",
            GetARMView
        );
    }

    private void GetARMView(CompressedImageMsg msg)
    {
        if (ARMTexture == null)
        {
            ARMTexture = new Texture2D(2, 2);
        }

        ARMTexture.LoadImage(msg.data);
        ARMTexture.Apply();

        if (ARMImagePanel != null)
        {
            ARMImagePanel.texture = ARMTexture;

            // 第一次收到图像数据时，确保尺寸被计算一次
            if (ARMTexture.width > 0 && !isInitialized)
            {
                UpdateAspectRatio();
            }
        }
    }

    // 【核心修复】基于目标宽度锁定比例，杜绝变形
    private void UpdateAspectRatio()
    {
        if (ARMImagePanel == null || ARMTexture == null || ARMTexture.width == 0) return;

        // 1. 获取纹理的原始物理比例 (例如 640/480 = 1.333)
        float textureAspect = (float)ARMTexture.width / ARMTexture.height;

        float absAngle = Mathf.Abs(rotationAngle % 180);

        // 2. 根据旋转角度计算 RectTransform 的 sizeDelta
        // 原理：RectTransform 的 (X, Y) 必须始终符合 textureAspect 的比例 (X/Y = 1.333)
        // 这样图像才不会被拉伸。我们只改变"哪个轴对应屏幕宽度"。

        // --- 场景 A: 竖屏状态 (旋转 90 或 270 或 -90 度) ---
        // 此时 RectTransform 本地坐标系的 Y 轴，变成了屏幕上的"水平方向"。
        if (Mathf.Approximately(absAngle, 90f))
        {
            // 我们希望视觉宽度 (Local Y) = displayWidth
            float h = displayWidth;

            // Local X (视觉高度) 必须按比例缩放: X / Y = textureAspect
            // 所以 X = Y * aspect
            float w = displayWidth * textureAspect;

            ARMImagePanel.rectTransform.sizeDelta = new Vector2(w, h);
        }
        // --- 场景 B: 横屏状态 (旋转 0 或 180 度) ---
        // 此时 RectTransform 本地坐标系的 X 轴，就是屏幕上的"水平方向"。
        else
        {
            // 我们希望视觉宽度 (Local X) = displayWidth
            float w = displayWidth;

            // Local Y (视觉高度) 按比例: X / Y = aspect => Y = X / aspect
            float h = displayWidth / textureAspect;

            ARMImagePanel.rectTransform.sizeDelta = new Vector2(w, h);
        }

        // 强制重置 Scale，防止被父物体干扰
        ARMImagePanel.rectTransform.localScale = Vector3.one;

        Debug.Log($"[比例锁定] 角度:{rotationAngle}, 计算尺寸:{ARMImagePanel.rectTransform.sizeDelta} (目标视宽: {displayWidth})");
    }
}
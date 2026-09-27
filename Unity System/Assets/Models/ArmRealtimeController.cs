using UnityEngine;

/// <summary>
/// Controls the forearm bone and visual EMG feedback.
/// Input angle is positive elbow flexion from MATLAB, normally 0..140 degrees.
/// If the model bends in the wrong direction, change foreArmRotationAxis in Inspector:
///   (1, 0, 0) or (-1, 0, 0)
/// </summary>
public class ArmRealtimeController : MonoBehaviour
{
    [Header("Bones")]
    public Transform foreArm;
    public Transform muscleCenterUp;
    public Transform muscleCenterDown;
    public Transform muscleLeft;
    public Transform muscleRight;

    [Header("Elbow Settings")]
    [Tooltip("0 degrees = arm straight.")]
    public float elbowRestAngle = 0f;

    [Tooltip("Maximum elbow flexion shown on model.")]
    public float elbowFlexedAngle = 140f;

    [Tooltip("Use (1,0,0) or (-1,0,0) depending on your imported model direction.")]
    public Vector3 foreArmRotationAxis = new Vector3(-1f, 0f, 0f);

    [Tooltip("Higher value = model follows MATLAB angle faster.")]
    public float elbowSmoothSpeed = 18f;

    [Header("Muscle Scale")]
    public Vector3 centerUpMinScale = Vector3.one;
    public Vector3 centerUpMaxScale = new Vector3(0.8f, 0f, 0f);

    public Vector3 centerDownMinScale = Vector3.one;
    public Vector3 centerDownMaxScale = new Vector3(1f, 1f, 0.45f);

    public Vector3 leftMinScale = Vector3.one;
    public Vector3 leftMaxScale = new Vector3(1f, 0.9f, 1.1f);

    public Vector3 rightMinScale = Vector3.one;
    public Vector3 rightMaxScale = new Vector3(1f, 0.9f, 1.1f);

    [Header("Biceps Color Only")]
    public Renderer muscleRenderer;
    public int bicepsMatIndex = 1;
    public Color lowColor = Color.white;
    public Color highColor = Color.red;
    public float colorSmoothSpeed = 8f;

    [Header("Live Input / Display")]
    [Tooltip("Angle currently applied to the model.")]
    public float elbowAngle = 0f;

    [Tooltip("Latest target angle received from MATLAB/UDP.")]
    public float targetElbowAngle = 0f;

    [Range(0f, 1f)]
    public float emgBiceps = 0f;

    [Header("Waveform")]
    public float[] emgWave = new float[80];

    private Quaternion foreArmInitRot;
    private MaterialPropertyBlock block;
    private float smoothBiceps = 0f;

    void Start()
    {
        if (foreArm != null)
            foreArmInitRot = foreArm.localRotation;

        targetElbowAngle = Mathf.Clamp(elbowAngle, elbowRestAngle, elbowFlexedAngle);
        block = new MaterialPropertyBlock();
    }

    void Update()
    {
        SmoothElbowInput();
        ApplyRotation();
        ApplyMuscleScale();
        ApplyBicepsColor();
    }

    public void SetElbowAngle(float angleDeg)
    {
        targetElbowAngle = Mathf.Clamp(angleDeg, elbowRestAngle, elbowFlexedAngle);
    }

    void SmoothElbowInput()
    {
        float follow = 1f - Mathf.Exp(-elbowSmoothSpeed * Time.deltaTime);
        elbowAngle = Mathf.Lerp(elbowAngle, targetElbowAngle, follow);
    }

    void ApplyRotation()
    {
        if (foreArm == null) return;

        Vector3 axis = foreArmRotationAxis.sqrMagnitude > 0.0001f
            ? foreArmRotationAxis.normalized
            : Vector3.right;

        Quaternion delta = Quaternion.AngleAxis(elbowAngle, axis);
        foreArm.localRotation = foreArmInitRot * delta;
    }

    void ApplyMuscleScale()
    {
        float t = Mathf.Clamp01(Mathf.InverseLerp(elbowRestAngle, elbowFlexedAngle, elbowAngle));

        if (muscleCenterUp != null)
            muscleCenterUp.localScale = Vector3.Lerp(centerUpMinScale, centerUpMaxScale, t);

        if (muscleCenterDown != null)
            muscleCenterDown.localScale = Vector3.Lerp(centerDownMinScale, centerDownMaxScale, t);

        if (muscleLeft != null)
            muscleLeft.localScale = Vector3.Lerp(leftMinScale, leftMaxScale, t);

        if (muscleRight != null)
            muscleRight.localScale = Vector3.Lerp(rightMinScale, rightMaxScale, t);
    }

    void ApplyBicepsColor()
    {
        if (muscleRenderer == null) return;

        smoothBiceps = Mathf.Lerp(smoothBiceps, emgBiceps, Time.deltaTime * colorSmoothSpeed);
        Color c = Color.Lerp(lowColor, highColor, smoothBiceps);

        muscleRenderer.GetPropertyBlock(block, bicepsMatIndex);
        block.SetColor("_Color", c);
        block.SetColor("_BaseColor", c);
        muscleRenderer.SetPropertyBlock(block, bicepsMatIndex);
    }
}

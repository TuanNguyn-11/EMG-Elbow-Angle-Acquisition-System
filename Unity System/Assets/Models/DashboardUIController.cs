using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class DashboardUIController : MonoBehaviour
{
    [Header("Data Source")]
    public ArmRealtimeController armController;

    [Header("Texts")]
    public TextMeshProUGUI angleValueText;
    public TextMeshProUGUI emgValueText;
    public TextMeshProUGUI bicepsStateText;

    [Header("Slider")]
    public Slider emgSlider;
    public Image emgFillImage;

    [Header("Colors")]
    public Color weakColor = new Color(0.7f, 0.9f, 1f);       // nhạt
    public Color moderateColor = new Color(1f, 0.85f, 0.2f);  // vàng
    public Color strongColor = new Color(1f, 0.25f, 0.25f);   // đỏ

    void Update()
    {
        if (armController == null) return;

        float angle = armController.elbowAngle;
        float emg = Mathf.Clamp01(armController.emgBiceps);

        if (angleValueText != null)
            angleValueText.text = $"Elbow Angle: {angle:F1}°";

        if (emgValueText != null)
            emgValueText.text = $"EMG Amplitude: {emg:F3}";

        if (bicepsStateText != null)
        {
            string state;
            Color stateColor;

            if (emg < 0.33f)
            {
                state = "Weak";
                stateColor = weakColor;
            }
            else if (emg < 0.66f)
            {
                state = "Moderate";
                stateColor = moderateColor;
            }
            else
            {
                state = "Strong";
                stateColor = strongColor;
            }

            bicepsStateText.text = $"Muscle State: {state}";
            bicepsStateText.color = stateColor;
        }

        if (emgSlider != null)
            emgSlider.value = emg;

        if (emgFillImage != null)
            emgFillImage.color = EvaluateColor(emg);
    }

    Color EvaluateColor(float value)
    {
        value = Mathf.Clamp01(value);

        if (value < 0.5f)
        {
            return Color.Lerp(weakColor, moderateColor, value / 0.5f);
        }
        else
        {
            return Color.Lerp(moderateColor, strongColor, (value - 0.5f) / 0.5f);
        }
    }
}
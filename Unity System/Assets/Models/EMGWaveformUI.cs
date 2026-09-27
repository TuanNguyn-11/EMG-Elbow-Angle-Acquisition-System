using UnityEngine;
public class EMGWaveformUI : MonoBehaviour
{
    public ArmRealtimeController armController;
    public EMGWaveformGraphic waveformGraphic;

    void Update()
    {
        if (armController == null || waveformGraphic == null) return;
        waveformGraphic.SetWave(armController.emgWave);
    }
}
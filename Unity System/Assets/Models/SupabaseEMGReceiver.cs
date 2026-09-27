using System;
using System.Collections;
using UnityEngine;
using UnityEngine.Networking;

public class SupabaseEMGReceiver : MonoBehaviour
{
    public enum WaveSource
    {
        Raw,
        Filtered,
        Delta,
        CloudWave
    }

    [Header("Supabase")]
    [Tooltip("Chỉ nhập tới .co, không thêm /rest/v1")]
    public string supabaseUrl = "https://YOUR_PROJECT.supabase.co";

    [TextArea(2, 4)]
    public string anonKey = "YOUR_ANON_KEY";

    [Header("Target")]
    public ArmRealtimeController armController;

    [Header("Polling")]
    public float pollInterval = 0.05f;

    [Header("Unity Smoothing")]
    public float followSpeed = 28f;

    [Header("Waveform")]
    public WaveSource waveSource = WaveSource.Filtered;
    public int waveLength = 80;

    [Tooltip("Arduino Nano ADC midpoint thường là 512")]
    public float rawMid = 512f;

    [Tooltip("Scale raw waveform. Nếu sóng nhỏ quá thì giảm, nếu lớn quá thì tăng.")]
    public float rawScale = 250f;

    [Tooltip("Scale filtered/delta waveform. Nếu sóng nhỏ quá thì giảm, nếu lớn quá thì tăng.")]
    public float filteredScale = 500f;

    [Header("Debug")]
    public bool showDebugLog = false;
    public bool showOnlyWhenNewData = false;

    private float targetEmg = 0f;
    private float targetAngle = 0f;

    private float[] localWave;
    private long lastTimeMs = -1;
    private int receivedCount = 0;
    private bool hasData = false;
    private Coroutine pollCoroutine;

    [Serializable]
    public class EMGLiveRow
    {
        public string id;
        public long time_ms;

        public float raw;
        public float filtered;
        public float baseline;
        public float delta;
        public bool active;

        public float emg;
        public float angle;

        public float[] wave;
        public string updated_at;
    }

    void Awake()
    {
        localWave = new float[waveLength];
    }

    void OnEnable()
    {
        if (supabaseUrl.EndsWith("/"))
            supabaseUrl = supabaseUrl.TrimEnd('/');

        if (supabaseUrl.Contains("/rest/v1"))
        {
            Debug.LogWarning("supabaseUrl không nên chứa /rest/v1. Hãy nhập dạng https://xxx.supabase.co");
        }

        if (armController == null)
        {
            Debug.LogWarning("Chưa gán ArmRealtimeController vào SupabaseEMGReceiver.");
        }

        pollCoroutine = StartCoroutine(PollLoop());
    }

    void OnDisable()
    {
        if (pollCoroutine != null)
            StopCoroutine(pollCoroutine);
    }

    void Update()
    {
        if (armController == null || !hasData)
            return;

        float k = 1f - Mathf.Exp(-followSpeed * Time.deltaTime);

        armController.emgBiceps = Mathf.Lerp(
            armController.emgBiceps,
            Mathf.Clamp01(targetEmg),
            k
        );

        armController.elbowAngle = Mathf.Lerp(
            armController.elbowAngle,
            targetAngle,
            k
        );

        if (localWave != null && localWave.Length > 1)
        {
            armController.emgWave = localWave;
        }
    }

    IEnumerator PollLoop()
    {
        while (true)
        {
            yield return StartCoroutine(GetCurrentData());
            yield return new WaitForSeconds(pollInterval);
        }
    }

    IEnumerator GetCurrentData()
    {
        string url =
            supabaseUrl +
            "/rest/v1/emg_live" +
            "?id=eq.current" +
            "&select=id,time_ms,raw,filtered,baseline,delta,active,emg,angle,wave,updated_at" +
            "&limit=1";

        using (UnityWebRequest req = UnityWebRequest.Get(url))
        {
            req.timeout = 2;

            req.SetRequestHeader("apikey", anonKey);
            req.SetRequestHeader("Authorization", "Bearer " + anonKey);
            req.SetRequestHeader("Accept", "application/json");
            req.SetRequestHeader("Cache-Control", "no-cache");

            yield return req.SendWebRequest();

            if (req.result != UnityWebRequest.Result.Success)
            {
                if (showDebugLog)
                {
                    Debug.LogWarning(
                        "Supabase GET failed\n" +
                        "Error: " + req.error + "\n" +
                        "Response: " + req.downloadHandler.text
                    );
                }

                yield break;
            }

            string json = req.downloadHandler.text;

            if (string.IsNullOrWhiteSpace(json) || json == "[]")
            {
                if (showDebugLog)
                    Debug.LogWarning("Không tìm thấy dòng id=current trong bảng emg_live.");

                yield break;
            }

            EMGLiveRow[] rows;

            try
            {
                rows = JsonHelper.FromJsonArray<EMGLiveRow>(json);
            }
            catch (Exception e)
            {
                Debug.LogError("Parse JSON failed: " + e.Message + "\nJSON: " + json);
                yield break;
            }

            if (rows == null || rows.Length == 0)
                yield break;

            EMGLiveRow row = rows[0];

            bool isNewData = row.time_ms != lastTimeMs;

            targetEmg = Mathf.Clamp01(row.emg);
            targetAngle = row.angle;

            if (isNewData)
            {
                lastTimeMs = row.time_ms;
                receivedCount++;

                UpdateLocalWave(row);

                if (showDebugLog || showOnlyWhenNewData)
                {
                    Debug.Log(
                        $"NEW EMG #{receivedCount} | " +
                        $"time_ms={row.time_ms} | " +
                        $"raw={row.raw:F0} | " +
                        $"filtered={row.filtered:F2} | " +
                        $"baseline={row.baseline:F2} | " +
                        $"delta={row.delta:F2} | " +
                        $"emg={row.emg:F3} | " +
                        $"angle={row.angle:F1} | " +
                        $"active={row.active} | " +
                        $"waveSource={waveSource}"
                    );
                }
            }

            hasData = true;
        }
    }

    void UpdateLocalWave(EMGLiveRow row)
    {
        if (localWave == null || localWave.Length != waveLength)
            localWave = new float[waveLength];

        float value = 0f;

        switch (waveSource)
        {
            case WaveSource.Raw:
                value = (row.raw - rawMid) / rawScale;
                break;

            case WaveSource.Filtered:
                value = (row.filtered - row.baseline) / filteredScale;
                break;

            case WaveSource.Delta:
                value = row.delta / filteredScale;
                break;

            case WaveSource.CloudWave:
                if (row.wave != null && row.wave.Length > 1)
                {
                    localWave = row.wave;
                    return;
                }
                else
                {
                    value = (row.filtered - row.baseline) / filteredScale;
                }
                break;
        }

        value = Mathf.Clamp(value, -1f, 1f);

        for (int i = 0; i < localWave.Length - 1; i++)
        {
            localWave[i] = localWave[i + 1];
        }

        localWave[localWave.Length - 1] = value;
    }
}

public static class JsonHelper
{
    public static T[] FromJsonArray<T>(string json)
    {
        string wrappedJson = "{\"items\":" + json + "}";
        Wrapper<T> wrapper = JsonUtility.FromJson<Wrapper<T>>(wrappedJson);
        return wrapper.items;
    }

    [Serializable]
    private class Wrapper<T>
    {
        public T[] items;
    }
}
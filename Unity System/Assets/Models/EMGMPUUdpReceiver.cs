using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class EMGMPUUdpReceiver : MonoBehaviour
{
    [Header("UDP Local Receiver")]
    public int listenPort = 55001;

    [Header("Data Target")]
    public ArmRealtimeController armController;

    [Header("EMG display scaling only")]
    public float deltaForFullActivation = 120f;

    [Range(16, 512)]
    public int waveformPointCount = 80;

    [Header("Debug")]
    public float latestElbowDeg;
    public float latestRaw;
    public float latestFiltered;
    public float latestBaseline;
    public float latestDelta;
    public bool latestActive;
    public int receivedPacketCount;
    public string latestPacket;

    private UdpClient udpClient;
    private Thread receiveThread;
    private volatile bool running;

    private readonly object sampleLock = new object();
    private LocalSample newestSample;
    private bool hasSample;

    private readonly Queue<float> waveform = new Queue<float>();

    private struct LocalSample
    {
        public float elbowDeg;
        public float raw;
        public float filtered;
        public float baseline;
        public float delta;
        public bool active;
        public string packet;
    }

    void Start()
    {
        try
        {
            udpClient = new UdpClient(listenPort);
            udpClient.Client.ReceiveTimeout = 100;
            running = true;

            receiveThread = new Thread(ReceiveLoop);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            Debug.Log("EMG + MPU UDP listening on port " + listenPort);
        }
        catch (Exception ex)
        {
            Debug.LogError("Cannot start UDP receiver on port " + listenPort + ": " + ex.Message);
        }
    }

    private void ReceiveLoop()
    {
        IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);

        while (running)
        {
            try
            {
                byte[] bytes = udpClient.Receive(ref sender);
                string message = Encoding.UTF8.GetString(bytes).Trim();

                if (!TryParsePacket(message, out LocalSample sample))
                    continue;

                sample.packet = message;

                lock (sampleLock)
                {
                    newestSample = sample;
                    hasSample = true;     // keep only newest packet; old packets are intentionally dropped
                    receivedPacketCount++;
                }
            }
            catch (SocketException)
            {
                // timeout is normal; it lets the thread exit quickly when Play stops
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch
            {
                // Ignore malformed UDP packets to avoid Console spam / frame drops.
            }
        }
    }

    void Update()
    {
        if (armController == null)
            return;

        LocalSample sample;
        lock (sampleLock)
        {
            if (!hasSample)
                return;

            sample = newestSample;
            hasSample = false;
        }

        latestElbowDeg = sample.elbowDeg;
        latestRaw = sample.raw;
        latestFiltered = sample.filtered;
        latestBaseline = sample.baseline;
        latestDelta = sample.delta;
        latestActive = sample.active;
        latestPacket = sample.packet;

        float displayEMG = Mathf.Clamp01(sample.delta / Mathf.Max(deltaForFullActivation, 0.001f));

        armController.SetElbowAngle(sample.elbowDeg);
        armController.emgBiceps = displayEMG;

        waveform.Enqueue(displayEMG);
        while (waveform.Count > waveformPointCount)
            waveform.Dequeue();

        armController.emgWave = waveform.ToArray();
    }

    private static bool TryParsePacket(string message, out LocalSample sample)
    {
        sample = default;
        string[] p = message.Split(',');

        // MATLAB format:
        // time_ms,elbow_deg,raw,filtered,baseline,delta,active
        if (p.Length != 7)
            return false;

        if (!TryParseFloat(p[1], out sample.elbowDeg)) return false;
        if (!TryParseFloat(p[2], out sample.raw)) return false;
        if (!TryParseFloat(p[3], out sample.filtered)) return false;
        if (!TryParseFloat(p[4], out sample.baseline)) return false;
        if (!TryParseFloat(p[5], out sample.delta)) return false;

        string activeText = p[6].Trim().ToLowerInvariant();
        sample.active = activeText == "1" || activeText == "true";
        return true;
    }

    private static bool TryParseFloat(string value, out float result)
    {
        return float.TryParse(value.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out result);
    }

    void OnDestroy()
    {
        StopReceiver();
    }

    void OnApplicationQuit()
    {
        StopReceiver();
    }

    private void StopReceiver()
    {
        running = false;

        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }

        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Join(200);
            receiveThread = null;
        }
    }
}

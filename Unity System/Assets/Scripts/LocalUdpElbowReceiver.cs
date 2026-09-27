using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class LocalUdpElbowReceiver : MonoBehaviour
{
    [Header("UDP")]
    public int listenPort = 5005;

    [Header("Target Controller")]
    public ArmRealtimeController armController;

    [Header("Input Angle")]
    public float inputMinAngle = 0f;
    public float inputMaxAngle = 140f;

    [Header("Unity Output Angle")]
    public float unityRestAngle = 0f;
    public float unityFlexAngle = 140f;

    [Header("Smoothing")]
    public float followSpeed = 18f;

    [Header("Debug")]
    public bool showDebugOnScreen = true;
    public bool showDebugLog = true;

    private UdpClient udpClient;
    private Thread receiveThread;
    private volatile bool running = false;

    private readonly object dataLock = new object();

    private float inputAngle = 0f;
    private float targetUnityAngle = 0f;
    private int receivedCount = 0;
    private string lastMessage = "No UDP data yet";
    private bool hasData = false;

    void Start()
    {
        StartUdp();
    }

    void Update()
    {
        if (armController == null || !hasData)
            return;

        float angleCopy;

        lock (dataLock)
        {
            angleCopy = targetUnityAngle;
        }

        float k = 1f - Mathf.Exp(-followSpeed * Time.deltaTime);

        armController.elbowAngle = Mathf.Lerp(
            armController.elbowAngle,
            angleCopy,
            k
        );
    }

    void OnGUI()
    {
        if (!showDebugOnScreen) return;

        GUIStyle style = new GUIStyle();
        style.fontSize = 24;
        style.normal.textColor = Color.green;

        GUI.Label(
            new Rect(20, 20, 900, 180),
            "LOCAL UDP ELBOW RECEIVER\n" +
            "Port: " + listenPort + "\n" +
            "Received: " + receivedCount + "\n" +
            "Input angle: " + inputAngle.ToString("F2") + "\n" +
            "Unity angle: " + targetUnityAngle.ToString("F2") + "\n" +
            "Last message: " + lastMessage,
            style
        );
    }

    void StartUdp()
    {
        try
        {
            udpClient = new UdpClient(listenPort);
            udpClient.Client.ReceiveTimeout = 1000;

            running = true;

            receiveThread = new Thread(ReceiveLoop);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            Debug.Log("LocalUdpElbowReceiver started on UDP port " + listenPort);
        }
        catch (Exception e)
        {
            Debug.LogError("UDP start error: " + e.Message);
            lastMessage = "UDP start error: " + e.Message;
        }
    }

    void ReceiveLoop()
    {
        IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);

        while (running)
        {
            try
            {
                byte[] data = udpClient.Receive(ref remoteEP);
                string msg = Encoding.UTF8.GetString(data).Trim();

                float angle;

                if (TryParseAngle(msg, out angle))
                {
                    angle = Mathf.Clamp(angle, inputMinAngle, inputMaxAngle);

                    float t = Mathf.InverseLerp(inputMinAngle, inputMaxAngle, angle);
                    float unityAngle = Mathf.Lerp(unityRestAngle, unityFlexAngle, t);

                    lock (dataLock)
                    {
                        inputAngle = angle;
                        targetUnityAngle = unityAngle;
                        receivedCount++;
                        lastMessage = msg + " from " + remoteEP.Address;
                        hasData = true;
                    }

                    if (showDebugLog)
                    {
                        Debug.Log(
                            "UDP received: " + msg +
                            " | inputAngle=" + angle.ToString("F2") +
                            " | unityAngle=" + unityAngle.ToString("F2")
                        );
                    }
                }
                else
                {
                    lock (dataLock)
                    {
                        lastMessage = "Invalid UDP: " + msg;
                    }
                }
            }
            catch (SocketException)
            {
                // Timeout bình thường, bỏ qua
            }
            catch (Exception e)
            {
                if (running)
                {
                    lock (dataLock)
                    {
                        lastMessage = "UDP receive error: " + e.Message;
                    }
                }
            }
        }
    }

    bool TryParseAngle(string msg, out float angle)
    {
        angle = 0f;

        if (string.IsNullOrWhiteSpace(msg))
            return false;

        string[] parts = msg.Split(',');

        // MATLAB gửi dạng: ANGLE,90
        if (parts.Length >= 2 && parts[0].Trim().Equals("ANGLE", StringComparison.OrdinalIgnoreCase))
        {
            return float.TryParse(
                parts[1].Trim(),
                NumberStyles.Float,
                CultureInfo.InvariantCulture,
                out angle
            );
        }

        // Dự phòng nếu chỉ gửi số: 90
        return float.TryParse(
            msg.Trim(),
            NumberStyles.Float,
            CultureInfo.InvariantCulture,
            out angle
        );
    }

    void OnDestroy()
    {
        StopUdp();
    }

    void OnApplicationQuit()
    {
        StopUdp();
    }

    void StopUdp()
    {
        running = false;

        try
        {
            if (udpClient != null)
            {
                udpClient.Close();
                udpClient = null;
            }
        }
        catch { }

        try
        {
            if (receiveThread != null && receiveThread.IsAlive)
            {
                receiveThread.Join(300);
            }
        }
        catch { }
    }
}
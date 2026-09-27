using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// Nhan goi tin UDP tu MATLAB dang: ELBOW,90.25
/// va dua goc duong vao ArmRealtimeController.
/// </summary>
public class MatlabElbowUdpReceiver : MonoBehaviour
{
    [Header("UDP From MATLAB")]
    public int listenPort = 55001;

    [Header("Target")]
    public ArmRealtimeController armController;

    [Header("Debug")]
    public bool showDebugLog = false;
    public float lastReceivedAngle = 0f;
    public bool isReceiving = false;

    private UdpClient udpClient;
    private Thread receiveThread;
    private volatile bool running;

    private readonly object angleLock = new object();
    private float pendingAngle;
    private bool hasPendingAngle;

    void Start()
    {
        Application.runInBackground = true;

        try
        {
            udpClient = new UdpClient(listenPort);
            udpClient.Client.ReceiveTimeout = 500;
            running = true;

            receiveThread = new Thread(ReceiveLoop);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            Debug.Log($"MATLAB elbow UDP receiver listening on port {listenPort}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Cannot open UDP port {listenPort}: {e.Message}");
        }
    }

    void Update()
    {
        float newAngle = 0f;
        bool apply = false;

        lock (angleLock)
        {
            if (hasPendingAngle)
            {
                newAngle = pendingAngle;
                hasPendingAngle = false;
                apply = true;
            }
        }

        if (!apply) return;

        lastReceivedAngle = newAngle;
        isReceiving = true;

        if (armController != null)
        {
            armController.SetElbowAngle(newAngle);
        }

        if (showDebugLog)
        {
            Debug.Log($"MATLAB elbow received: {newAngle:F2} deg");
        }
    }

    void ReceiveLoop()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);

        while (running)
        {
            try
            {
                byte[] bytes = udpClient.Receive(ref remoteEndPoint);
                string message = Encoding.UTF8.GetString(bytes).Trim();

                if (!message.StartsWith("ELBOW,", StringComparison.OrdinalIgnoreCase))
                    continue;

                string valueText = message.Substring("ELBOW,".Length);

                if (!float.TryParse(
                    valueText,
                    NumberStyles.Float,
                    CultureInfo.InvariantCulture,
                    out float angle))
                {
                    continue;
                }

                lock (angleLock)
                {
                    pendingAngle = Mathf.Clamp(angle, 0f, 140f);
                    hasPendingAngle = true;
                }
            }
            catch (SocketException)
            {
                // Timeout de thread co the kiem tra running va thoat sach.
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (Exception)
            {
                // Khong goi Unity API tu background thread.
            }
        }
    }

    void OnDestroy()
    {
        StopReceiver();
    }

    void OnApplicationQuit()
    {
        StopReceiver();
    }

    void StopReceiver()
    {
        running = false;

        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }

        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Join(700);
        }

        receiveThread = null;
    }
}

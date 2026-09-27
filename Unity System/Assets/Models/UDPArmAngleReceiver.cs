using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Globalization;

public class UDPArmAngleReceiver : MonoBehaviour
{
    [Header("UDP")]
    public int listenPort = 5005;

    [Header("Target Controller")]
    public ArmRealtimeController armController;

    [Header("Debug Data")]
    public float timeMs;
    public float elbowAngle;
    public float unityAngle;

    private UdpClient udpClient;
    private Thread receiveThread;
    private bool running = false;

    private readonly object dataLock = new object();
    private string latestMessage = "";

    void Start()
    {
        udpClient = new UdpClient(listenPort);
        running = true;

        receiveThread = new Thread(ReceiveLoop);
        receiveThread.IsBackground = true;
        receiveThread.Start();

        Debug.Log("UDP Arm Angle Receiver started on port " + listenPort);
    }

    void ReceiveLoop()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, listenPort);

        while (running)
        {
            try
            {
                byte[] data = udpClient.Receive(ref remoteEndPoint);
                string text = Encoding.UTF8.GetString(data);

                lock (dataLock)
                {
                    latestMessage = text;
                }
            }
            catch
            {
                // Ignore closing errors
            }
        }
    }

    void Update()
    {
        string msg;

        lock (dataLock)
        {
            msg = latestMessage;
        }

        if (string.IsNullOrEmpty(msg))
            return;

        if (ParseMessage(msg))
        {
            if (armController != null)
            {
                armController.elbowAngle = unityAngle;
            }
        }
    }

    bool ParseMessage(string msg)
    {
        string[] p = msg.Trim().Split(',');

        if (p.Length != 3)
        {
            Debug.LogWarning("Invalid UDP message: " + msg);
            return false;
        }

        bool ok0 = float.TryParse(p[0], NumberStyles.Float, CultureInfo.InvariantCulture, out timeMs);
        bool ok1 = float.TryParse(p[1], NumberStyles.Float, CultureInfo.InvariantCulture, out elbowAngle);
        bool ok2 = float.TryParse(p[2], NumberStyles.Float, CultureInfo.InvariantCulture, out unityAngle);

        return ok0 && ok1 && ok2;
    }

    void OnApplicationQuit()
    {
        running = false;

        if (udpClient != null)
        {
            udpClient.Close();
            udpClient = null;
        }

        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Abort();
        }
    }
}
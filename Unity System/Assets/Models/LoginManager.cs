using UnityEngine;
using TMPro;

public class LoginManager : MonoBehaviour
{
    [Header("Login UI")]
    public GameObject loginCanvas;
    public GameObject dashboardCanvas;

    [Header("Input Fields")]
    public TMP_InputField usernameInput;
    public TMP_InputField passwordInput;

    [Header("Message")]
    public TMP_Text errorText;

    [Header("Account Config")]
    [SerializeField] private string correctUsername = "admin";
    [SerializeField] private string correctPassword = "123456";

    private void Start()
    {
        loginCanvas.SetActive(true);
        dashboardCanvas.SetActive(false);

        if (errorText != null)
        {
            errorText.text = "";
        }
    }

    public void Login()
    {
        string username = usernameInput.text.Trim();
        string password = passwordInput.text.Trim();

        if (username == correctUsername && password == correctPassword)
        {
            loginCanvas.SetActive(false);
            dashboardCanvas.SetActive(true);

            if (errorText != null)
            {
                errorText.text = "";
            }
        }
        else
        {
            if (errorText != null)
            {
                errorText.text = "Sai tài khoản hoặc mật khẩu. Vui lòng thử lại.";
            }

            passwordInput.text = "";
            passwordInput.ActivateInputField();
        }
    }

    public void QuitApp()
    {
        Application.Quit();
    }
}
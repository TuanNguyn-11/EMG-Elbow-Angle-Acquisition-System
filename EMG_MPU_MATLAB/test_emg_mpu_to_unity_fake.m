clc;
clear;

% =====================================================
% TEST MATLAB -> UNITY KHÔNG CẦN PHẦN CỨNG
% Giả lập dữ liệu EMG + góc gập khuỷu tay
% =====================================================

% ==============================
% 1. Cấu hình UDP
% ==============================
unityIP = "127.0.0.1";     % chạy Unity cùng máy
unityPort = 55001;         % phải trùng với port bên Unity

u = udpport("datagram", "IPV4");

disp("========================================");
disp(" MATLAB FAKE EMG + MPU TEST SENDER");
disp(" Dang gui du lieu gia lap sang Unity...");
disp(" IP: " + unityIP);
disp(" Port: " + unityPort);
disp(" Nhan Ctrl + C de dung chuong trinh");
disp("========================================");

% ==============================
% 2. Thông số giả lập
% ==============================
fs = 50;                  % 50 Hz, gửi mỗi 20 ms
Ts = 1 / fs;

t = 0;
time_ms = 0;

% Giới hạn góc khuỷu tay
minAngle = 0;
maxAngle = 140;

% EMG giả lập
baseline = 80;
noiseLevel = 20;

% ==============================
% 3. Vòng lặp gửi dữ liệu
% ==============================
while true

    % ---------------------------------------------
    % Góc khuỷu tay giả lập
    % Dạng sóng sin: 0 -> 140 -> 0
    % ---------------------------------------------
    elbowAngle = (maxAngle / 2) * (1 - cos(2 * pi * 0.15 * t));

    % Góc gửi sang Unity
    % Nếu model Unity của bạn đang nhận góc âm thì dùng dấu trừ
    unityAngle = -elbowAngle;

    % ---------------------------------------------
    % EMG giả lập
    % Khi góc lớn thì cơ co mạnh hơn
    % ---------------------------------------------
    muscleLevel = elbowAngle / maxAngle;   % 0.0 -> 1.0

    emgRaw = baseline ...
        + 600 * muscleLevel ...
        + noiseLevel * randn();

    emgRaw = max(0, min(1023, emgRaw));

    % EMG amplitude giả lập sau lọc
    emgAmplitude = 600 * muscleLevel;

    % Trạng thái cơ
    if muscleLevel < 0.25
        muscleState = "Relax";
    elseif muscleLevel < 0.65
        muscleState = "Medium";
    else
        muscleState = "Contract";
    end

    % ---------------------------------------------
    % Gói dữ liệu gửi sang Unity
    % Format CSV:
    % time_ms,elbowAngle,unityAngle,emgRaw,emgAmplitude,muscleLevel,muscleState
    % ---------------------------------------------
    msg = sprintf("%d,%.2f,%.2f,%.2f,%.2f,%.3f,%s", ...
        round(time_ms), ...
        elbowAngle, ...
        unityAngle, ...
        emgRaw, ...
        emgAmplitude, ...
        muscleLevel, ...
        muscleState);

    % Gửi UDP
    write(u, uint8(char(msg)), unityIP, unityPort);

    % In ra Command Window để kiểm tra
    fprintf("t=%6d ms | elbow=%6.2f deg | unity=%7.2f | emg=%7.2f | level=%.2f | %s\n", ...
        round(time_ms), elbowAngle, unityAngle, emgRaw, muscleLevel, muscleState);

    % Cập nhật thời gian
    pause(Ts);
    t = t + Ts;
    time_ms = time_ms + Ts * 1000;
end
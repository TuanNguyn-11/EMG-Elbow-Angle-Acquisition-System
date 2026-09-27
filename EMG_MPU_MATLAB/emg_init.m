%% emg_init.m
% Thông số giữ nguyên theo run_emg_live_local.m

PORT = "COM9";
BAUD = 115200;

UNITY_IP = "127.0.0.1";
UNITY_PORT = 55001;
SEND_TO_UNITY = true;

UDP_RATE_HZ = 30;
PRINT_RATE_HZ = 1;

fs = 100;
N = 2;
fc = 5;

BASELINE_SECONDS = 2.0;
THRESHOLD_DELTA = 35;
LEVEL_SCALE = 512.0;

[b_emg, a_emg] = butter(N, fc/(fs/2), "low");
zi_emg = zeros(max(length(a_emg), length(b_emg)) - 1, 1);

fprintf("EMG Simulink parameters loaded.\n");
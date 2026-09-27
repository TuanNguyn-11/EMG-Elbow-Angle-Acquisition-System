
clc; clear; close all;

%% ==============================
% 1. User settings
% ===============================
PORT = "COM9";          % CHANGE THIS: e.g. "COM5", "COM7", "COM9"
BAUD = 115200;

UNITY_IP   = "127.0.0.1";
UNITY_PORT = 55001;
SEND_TO_UNITY = true;

UDP_RATE_HZ   = 30;      % Unity receives at 30 Hz, enough for smooth motion
PRINT_RATE_HZ = 1;       % health status every 1 second

%% ==============================
% 2. EMG filter
% ===============================
fs = 100;
N  = 2;
fc = 5;
[b, a] = butter(N, fc/(fs/2), "low");
zi = zeros(max(length(a), length(b)) - 1, 1);

BASELINE_SECONDS = 2.0;
THRESHOLD_DELTA  = 35;
LEVEL_SCALE      = 512.0;

%% ==============================
% 3. Open Serial + UDP
% ===============================
fprintf("\n==============================================\n");
fprintf(" EMG + MPU6050 -> MATLAB -> Unity UDP\n");
fprintf(" NO FIGURE | DIAGNOSTIC MODE\n");
fprintf("==============================================\n");
fprintf("[1/4] Opening Serial %s at %d baud...\n", PORT, BAUD);

try
    ser = serialport(PORT, BAUD, "Timeout", 0.02);
    configureTerminator(ser, "LF");
    flush(ser);
catch ME
    error("Cannot open Serial %s. Close Arduino Serial Monitor or check COM. Detail: %s", PORT, ME.message);
end

fprintf("[OK] Serial opened. Arduino Serial Monitor must be CLOSED.\n");

if SEND_TO_UNITY
    fprintf("[2/4] Opening UDP sender to Unity %s:%d...\n", UNITY_IP, UNITY_PORT);
    udpObj = udpport("datagram", "IPV4");
    fprintf("[OK] UDP ready. Start Unity Play Mode before or after this script.\n");
else
    udpObj = [];
    fprintf("[INFO] UDP disabled. MATLAB only reads Serial and filters EMG.\n");
end

cleanupObj = onCleanup(@() cleanupAll(ser, udpObj)); %#ok<NASGU>

fprintf("[3/4] Waiting for Arduino CSV data...\n");
fprintf("Expected main format: time_ms,elbow_vector_filtered_deg,emg_raw\n");
fprintf("Also accepts test format: time_ms,upper_ok,forearm_ok,elbow_raw_deg,elbow_filtered_deg,emg_raw,...\n");
fprintf("Press Ctrl+C in MATLAB to stop.\n\n");

%% ==============================
% 4. Live variables
% ===============================
serialBuffer = "";
rawBuf = [];
baseline = NaN;

sampleCount = 0;
invalidLineCount = 0;
textLogCount = 0;

lastAnyByteWallTime = tic;
lastValidWallTime = tic;
lastUdpSend = tic;
lastPrint = tic;
lastWaitPrint = tic;
scriptStart = tic;

minUdpPeriod = 1 / UDP_RATE_HZ;
minPrintPeriod = 1 / PRINT_RATE_HZ;

latest.t_ms = NaN;
latest.elbow = NaN;
latest.raw = NaN;
latest.rect = NaN;
latest.env = NaN;
latest.level = NaN;
latest.active = false;
latest.upperOk = NaN;
latest.forearmOk = NaN;
latest.format = "none";

fprintf("[4/4] Runtime check starting...\n");
fprintf("    - MATLAB will print MPU/EMG/UDP health every %.1f s.\n", minPrintPeriod);
fprintf("    - Keep muscle relaxed for %.1f s after valid EMG samples appear.\n\n", BASELINE_SECONDS);

%% ==============================
% 5. Main loop
% ===============================
while true
    try
        if ser.NumBytesAvailable == 0
            if toc(lastWaitPrint) >= 1.0 && sampleCount == 0
                fprintf("[WAIT] No valid CSV yet | elapsed %.1fs | bytes available = %d\n", ...
                    toc(scriptStart), ser.NumBytesAvailable);
                lastWaitPrint = tic;
            end
            pause(0.002);
            continue;
        end

        lastAnyByteWallTime = tic;
        chunk = string(read(ser, ser.NumBytesAvailable, "char"));
        serialBuffer = serialBuffer + chunk;

        lines = splitlines(serialBuffer);
        if strlength(serialBuffer) > 0 && ~endsWith(serialBuffer, newline) && ~endsWith(serialBuffer, char(13))
            serialBuffer = lines(end);
            lines = lines(1:end-1);
        else
            serialBuffer = "";
        end

        for idxLine = 1:numel(lines)
            line = strtrim(lines(idxLine));
            if strlength(line) == 0
                continue;
            end

            % Header/log handling. Print useful Arduino status once in a while.
            lowLine = lower(line);
            if contains(lowLine, "time_ms") || startsWith(line, "CSV")
                fprintf("[ARDUINO HEADER] %s\n", line);
                continue;
            end

            if startsWith(line, "WAIT") || startsWith(line, "SKIP") || startsWith(line, "#") || startsWith(line, "[") || startsWith(line, "{") || contains(lowLine, "mpu") || contains(lowLine, "emg") || contains(lowLine, "dmp") || contains(lowLine, "ready") || contains(lowLine, "calibr") || contains(lowLine, "found") || contains(lowLine, "failed") || contains(lowLine, "error")
                textLogCount = textLogCount + 1;
                if textLogCount <= 30 || mod(textLogCount, 20) == 0
                    fprintf("[ARDUINO] %s\n", line);
                end
                continue;
            end

            [ok, t_ms, elbow_deg, emg_raw, upperOk, forearmOk, fmt] = parseArduinoLine(line);
            if ~ok
                invalidLineCount = invalidLineCount + 1;
                if invalidLineCount <= 5 || mod(invalidLineCount, 50) == 0
                    fprintf("[SKIP INVALID] %s\n", line);
                end
                continue;
            end

            sampleCount = sampleCount + 1;
            lastValidWallTime = tic;

            if isnan(baseline)
                rawBuf(end+1) = emg_raw; %#ok<SAGROW>
                tempBaseline = median(rawBuf);
                emg_rectified = abs(emg_raw - tempBaseline);
                emg_env = 0;
                emg_level = 0;
                emg_active = false;

                if numel(rawBuf) >= round(BASELINE_SECONDS * fs)
                    baseline = median(rawBuf);
                    fprintf("\n[OK] EMG baseline ready: %.2f ADC from %d samples.\n", baseline, numel(rawBuf));
                    fprintf("[RUN] MATLAB is now sending elbow + filtered EMG to Unity.\n\n");
                end
            else
                emg_rectified = abs(emg_raw - baseline);
                [emg_env, zi] = filter(b, a, emg_rectified, zi);
                emg_level = min(max(emg_env / LEVEL_SCALE, 0), 1);
                emg_active = emg_env > THRESHOLD_DELTA;
            end

            latest.t_ms = t_ms;
            latest.elbow = elbow_deg;
            latest.raw = emg_raw;
            latest.rect = emg_rectified;
            latest.env = emg_env;
            latest.level = emg_level;
            latest.active = emg_active;
            latest.upperOk = upperOk;
            latest.forearmOk = forearmOk;
            latest.format = fmt;

            if SEND_TO_UNITY && ~isnan(baseline) && toc(lastUdpSend) >= minUdpPeriod
                activeInt = double(emg_active);
                elbowForUnity = min(max(elbow_deg, 0), 140);
                msg = sprintf('%d,%.3f,%.0f,%.6f,%.6f,%.6f,%d', ...
                    round(t_ms), elbowForUnity, emg_raw, emg_env, baseline, emg_env, activeInt);
                write(udpObj, uint8(char(msg)), "uint8", UNITY_IP, UNITY_PORT);
                lastUdpSend = tic;
            end
        end

        if toc(lastPrint) >= minPrintPeriod
            printHealth(latest, sampleCount, invalidLineCount, baseline, rawBuf, SEND_TO_UNITY, lastValidWallTime, lastAnyByteWallTime);
            lastPrint = tic;
        end

        if toc(lastValidWallTime) > 3.0 && sampleCount > 0
            fprintf("[WARNING] No valid CSV for >3s. Arduino may be calibrating MPU, stuck, or output format changed.\n");
            lastValidWallTime = tic;
        end

        if toc(lastAnyByteWallTime) > 5.0
            fprintf("[WARNING] No Serial bytes for >5s. Check COM, cable, Arduino power, or close Serial Monitor.\n");
            lastAnyByteWallTime = tic;
        end

    catch ME
        warning("Runtime error: %s", ME.message);
        pause(0.05);
    end
end

%% ==============================
% Helper functions
% ===============================
function [ok, t_ms, elbow_deg, emg_raw, upperOk, forearmOk, fmt] = parseArduinoLine(line)
    ok = false;
    t_ms = NaN;
    elbow_deg = NaN;
    emg_raw = NaN;
    upperOk = NaN;
    forearmOk = NaN;
    fmt = "unknown";

    parts = split(line, ",");
    vals = nan(1, numel(parts));
    for k = 1:numel(parts)
        vals(k) = str2double(strtrim(parts(k)));
    end

    % MPU_success format with EMG appended, if you ever enable debug output:
    % time_ms, upper dXYZ, lower dXYZ, rel dXYZ, oldY, raw, median, filtered, emg_raw
    if numel(vals) >= 15 && all(~isnan(vals([1 14 15])))
        t_ms = vals(1);
        elbow_deg = vals(14);
        emg_raw = vals(15);
        upperOk = 1;
        forearmOk = 1;
        fmt = "mpu_success_15col";
        ok = true;
        return;
    end

    % Main format: time_ms,elbow_deg,emg_raw
    if numel(vals) >= 3 && all(~isnan(vals(1:3)))
        % Test format: time_ms,upper_ok,forearm_ok,elbow_raw_deg,elbow_filtered_deg,emg_raw,...
        % Detect it by column 2/3 being 0/1 and having at least 6 columns.
        if numel(vals) >= 6 && ismember(vals(2), [0 1]) && ismember(vals(3), [0 1]) && all(~isnan(vals(1:6)))
            t_ms = vals(1);
            upperOk = vals(2);
            forearmOk = vals(3);
            elbow_deg = vals(5);    % use filtered angle from diagnostic Arduino
            emg_raw = vals(6);
            fmt = "test_6col";
        else
            t_ms = vals(1);
            elbow_deg = vals(2);
            emg_raw = vals(3);
            fmt = "main_3col";
        end

        if ~isnan(t_ms) && ~isnan(elbow_deg) && ~isnan(emg_raw)
            ok = true;
        end
    end
end

function printHealth(latest, sampleCount, invalidLineCount, baseline, rawBuf, sendToUnity, lastValidWallTime, lastAnyByteWallTime)
    if sampleCount == 0
        fprintf("[CHECK] Serial bytes seen %.1fs ago | valid samples = 0 | waiting Arduino/MPU/CSV...\n", toc(lastAnyByteWallTime));
        return;
    end

    if isnan(baseline)
        emgStatus = sprintf("BASELINE %d/%.0f", numel(rawBuf), 200);
    else
        if latest.raw <= 2 || latest.raw >= 1021
            emgStatus = "EMG WARNING: ADC near rail";
        else
            emgStatus = "EMG OK";
        end
    end

    if latest.format == "test_6col" || latest.format == "mpu_success_15col"
        if latest.upperOk == 1 && latest.forearmOk == 1
            mpuStatus = "MPU OK: 0x68=1, 0x69=1";
        else
            mpuStatus = sprintf("MPU WARNING: 0x68=%.0f, 0x69=%.0f", latest.upperOk, latest.forearmOk);
        end
    else
        if ~isnan(latest.elbow)
            mpuStatus = "MPU DATA OK";
        else
            mpuStatus = "MPU WAIT";
        end
    end

    if sendToUnity && ~isnan(baseline)
        udpStatus = "UDP ON";
    elseif sendToUnity
        udpStatus = "UDP WAIT BASELINE";
    else
        udpStatus = "UDP OFF";
    end

    fprintf("[CHECK] %s | %s | %s | samples=%d | invalid=%d | t=%8.0f ms | elbow=%7.2f | raw=%4.0f | env=%7.2f | active=%d | noCSV=%.1fs\n", ...
        mpuStatus, emgStatus, udpStatus, sampleCount, invalidLineCount, latest.t_ms, latest.elbow, latest.raw, latest.env, latest.active, toc(lastValidWallTime));
end

function cleanupAll(ser, udpObj)
    try
        flush(ser);
        clear ser;
        fprintf("\nSerial closed.\n");
    catch
    end
    try
        if ~isempty(udpObj)
            clear udpObj;
            fprintf("UDP closed.\n");
        end
    catch
    end
end

%% mpu_dmp_to_unity_udp.m
% Arduino MPU -> MATLAB -> Unity qua UDP
%
% BAN SUA LOI:
% - Khong dung while + readline khi dong Serial chua gui xong.
% - configureCallback chi goi ham xu ly khi da nhan du mot dong ket thuc bang LF.
% - Tranh loi timeout/readline/strtrim khi Arduino dang reset hoac calibrate MPU.
%
% DINH DANG ARDUINO:
% time_ms,
% upper_dX_deg,upper_dY_deg,upper_dZ_deg,
% lower_dX_deg,lower_dY_deg,lower_dZ_deg,
% rel_dX_deg,rel_dY_deg,rel_dZ_deg,
% elbow_axisY_old_deg,
% elbow_vector_raw_deg,elbow_vector_median_deg,elbow_vector_filtered_deg
%
% GIA TRI GUI UNITY:
% elbow_vector_filtered_deg (cot 14)
%
% DUNG CHUONG TRINH:
% - Dong cua so figure, hoac nhan Ctrl+C.
% - Trong Unity: MatlabElbowUdpReceiver Listen Port = 55001.
% - Cung may: unityIP = "127.0.0.1".

clc;
clear;

%% ==================== CAU HINH ====================
serialPort = "COM9";          % Sua lai neu Arduino dang dung COM khac
baudRate   = 115200;

unityIP    = "127.0.0.1";     % KHONG dung 127.0.0.14 neu Unity chay cung may
unityPort  = 55001;

consolePeriod = 0.15;         % Chu ky in Command Window
windowSeconds = 6;            % Khoang thoi gian hien tren do thi

%% ==================== MO SERIAL VA UDP ====================
fprintf("Dang mo Serial %s tai %d baud...\n", serialPort, baudRate);

% Bao loi ro neu COM dang bi Serial Monitor chiem.
try
    ser = serialport(serialPort, baudRate);
catch ME
    fprintf(2, "Khong mo duoc %s. Hay dong PlatformIO/Arduino Serial Monitor.\n", serialPort);
    rethrow(ME);
end

configureTerminator(ser, "LF");

% Timeout khong con dung de doc vong lap; dat du rong de xu ly thu cong neu can.
ser.Timeout = 2.0;
flush(ser);

udpOut = udpport("datagram", "IPV4");

fprintf("Da ket noi Serial.\n");
fprintf("Dang gui goc khuyu toi Unity: %s:%d\n", unityIP, unityPort);
fprintf("Cho Arduino khoi dong/calibrate MPU; MATLAB se tu dong doc khi co dong du lieu hoan chinh.\n");
fprintf("Gia tri chinh: elbow_vector_filtered_deg\n\n");

%% ==================== TAO DO THI ====================
fig = figure( ...
    "Name", "MPU Elbow Angle -> Unity UDP", ...
    "NumberTitle", "off", ...
    "CloseRequestFcn", @closeFigureSafely);

ax = axes(fig);
hold(ax, "on");
grid(ax, "on");
title(ax, "Goc gap khuyu tay gui sang Unity");
xlabel(ax, "Thoi gian (s)");
ylabel(ax, "Goc (do)");
ylim(ax, [0 155]);
xlim(ax, [0 windowSeconds]);

rawLine = animatedline(ax, "DisplayName", "Vector raw");
filteredLine = animatedline(ax, "LineWidth", 1.5, "DisplayName", "Filtered -> Unity");
oldLine = animatedline(ax, "DisplayName", "Axis-Y cu");
legend(ax, "Location", "northwest");

%% ==================== LUU TRANG THAI CHO CALLBACK ====================
state.udpOut = udpOut;
state.unityIP = unityIP;
state.unityPort = unityPort;
state.consolePeriod = consolePeriod;
state.windowSeconds = windowSeconds;
state.rawLine = rawLine;
state.filteredLine = filteredLine;
state.oldLine = oldLine;
state.ax = ax;
state.lastConsoleTime = tic;
state.validCount = 0;
state.lastUnityAngle = 0;

setappdata(fig, "UDPState", state);

%% ==================== CHI DOC KHI DA CO DONG HOAN CHINH ====================
% Callback "terminator" chi chay sau khi nhan ky tu LF tu Arduino.
% Vi vay, readline khong bi goi som trong thoi gian Arduino chua gui xong dong.
configureCallback(ser, "terminator", @(src, evt) onSerialLine(src, fig));

fprintf("Dang lang nghe. Dong figure de dung chuong trinh.\n");

try
    waitfor(fig);
catch ME
    fprintf(2, "\nLOI TRONG KHI CHAY: %s\n", ME.message);
end

%% ==================== DUNG KET NOI ====================
try
    configureCallback(ser, "off");
catch
end

clear ser udpOut;
fprintf("Da dong Serial va UDP.\n");

%% ==================== CALLBACK DOC SERIAL ====================
function onSerialLine(ser, fig)
    if ~isvalid(fig)
        return;
    end

    state = getappdata(fig, "UDPState");

    try
        line = string(readline(ser));
    catch ME
        % Khong dung chuong trinh neu co mot goi Serial loi.
        fprintf(2, "Bo qua mot dong Serial loi: %s\n", ME.message);
        return;
    end

    if ismissing(line)
        return;
    end

    line = strtrim(line);

    if strlength(line) == 0 || startsWith(line, "#") || startsWith(line, "time_ms")
        return;
    end

    parts = split(line, ",");

    if numel(parts) < 14
        return;
    end

    values = str2double(parts);

    % Cot can dung: 1=time_ms, 11=oldY, 12=raw, 14=filtered
    if any(isnan(values([1 11 12 14])))
        return;
    end

    t_s            = values(1) / 1000.0;
    elbowAxisYOld  = values(11);
    elbowVectorRaw = values(12);
    elbowFiltered  = values(14);

    % Model Unity chi nhan goc hop ly cua khop khuuyu.
    elbowForUnity = min(max(elbowFiltered, 0.0), 140.0);

    packet = sprintf("ELBOW,%.2f", elbowForUnity);

    try
        write(state.udpOut, uint8(char(packet)), "uint8", char(state.unityIP), state.unityPort);
    catch ME
        fprintf(2, "UDP gui that bai: %s\n", ME.message);
        return;
    end

    state.validCount = state.validCount + 1;
    state.lastUnityAngle = elbowForUnity;

    addpoints(state.rawLine, t_s, elbowVectorRaw);
    addpoints(state.filteredLine, t_s, elbowForUnity);
    addpoints(state.oldLine, t_s, elbowAxisYOld);

    xRight = max(state.windowSeconds, t_s);
    xLeft = max(0, xRight - state.windowSeconds);
    xlim(state.ax, [xLeft xRight]);
    drawnow limitrate nocallbacks;

    if toc(state.lastConsoleTime) >= state.consolePeriod
        fprintf("t=%8.2fs | oldY=%7.2f deg | raw=%7.2f deg | UNITY=%7.2f deg\n", ...
            t_s, elbowAxisYOld, elbowVectorRaw, elbowForUnity);
        state.lastConsoleTime = tic;
    end

    setappdata(fig, "UDPState", state);
end

%% ==================== DONG FIGURE AN TOAN ====================
function closeFigureSafely(fig, ~)
    if isvalid(fig)
        state = getappdata(fig, "UDPState");
        if isstruct(state) && isfield(state, "validCount")
            fprintf("\nDang dong chuong trinh. Tong so mau gui Unity: %d\n", state.validCount);
        end
        delete(fig);
    end
end

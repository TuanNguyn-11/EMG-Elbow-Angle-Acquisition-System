classdef SerialCsvReader < matlab.System
    % SerialCsvReader
    % Doc Serial giong run_emg_live_local.m:
    % - doc tat ca byte dang co
    % - cong vao serialBuffer
    % - tach dong
    % - bo header/log
    % - parse 3 dinh dang CSV

    properties(Nontunable)
        PORT = "COM9"
        BAUD = 115200
        Timeout = 0.02
    end

    properties(Access = private)
        ser
        serialBuffer string = ""

        sampleCount double = 0
        invalidLineCount double = 0
        textLogCount double = 0

        lastAnyByteWallTime
        lastValidWallTime
        lastWaitPrint
        scriptStart

        latest_t_ms double = NaN
        latest_elbow double = NaN
        latest_raw double = NaN
        latest_upperOk double = NaN
        latest_forearmOk double = NaN
        latest_fmt double = 0
    end

    methods(Access = protected)

        function setupImpl(obj)
            fprintf("\n==============================================\n");
            fprintf(" EMG + MPU6050 -> Simulink -> Unity UDP\n");
            fprintf(" NO FIGURE | DIAGNOSTIC MODE\n");
            fprintf("==============================================\n");

            fprintf("[1/4] Opening Serial %s at %.0f baud...\n", char(obj.PORT), obj.BAUD);

            obj.ser = serialport(obj.PORT, obj.BAUD, "Timeout", obj.Timeout);
            configureTerminator(obj.ser, "LF");
            flush(obj.ser);

            fprintf("[OK] Serial opened. Arduino Serial Monitor must be CLOSED.\n");
            fprintf("[3/4] Waiting for Arduino CSV data...\n");
            fprintf("Expected main format: time_ms,elbow_vector_filtered_deg,emg_raw\n");
            fprintf("Also accepts test format: time_ms,upper_ok,forearm_ok,elbow_raw_deg,elbow_filtered_deg,emg_raw,...\n");
            fprintf("Press Stop in Simulink to stop.\n\n");

            obj.lastAnyByteWallTime = tic;
            obj.lastValidWallTime = tic;
            obj.lastWaitPrint = tic;
            obj.scriptStart = tic;
        end

        function [ok, t_ms, elbow_deg, emg_raw, upperOk, forearmOk, fmt, ...
                  sampleCount, invalidLineCount, textLogCount, noValidSec, noByteSec] = stepImpl(obj)

            ok = false;

            t_ms = obj.latest_t_ms;
            elbow_deg = obj.latest_elbow;
            emg_raw = obj.latest_raw;
            upperOk = obj.latest_upperOk;
            forearmOk = obj.latest_forearmOk;
            fmt = obj.latest_fmt;

            if obj.ser.NumBytesAvailable == 0
                if toc(obj.lastWaitPrint) >= 1.0 && obj.sampleCount == 0
                    fprintf("[WAIT] No valid CSV yet | elapsed %.1fs | bytes available = %.0f\n", ...
                        toc(obj.scriptStart), obj.ser.NumBytesAvailable);
                    obj.lastWaitPrint = tic;
                end

                sampleCount = obj.sampleCount;
                invalidLineCount = obj.invalidLineCount;
                textLogCount = obj.textLogCount;
                noValidSec = toc(obj.lastValidWallTime);
                noByteSec = toc(obj.lastAnyByteWallTime);
                return;
            end

            obj.lastAnyByteWallTime = tic;

            chunk = string(read(obj.ser, obj.ser.NumBytesAvailable, "char"));
            obj.serialBuffer = obj.serialBuffer + chunk;

            lines = splitlines(obj.serialBuffer);

            if strlength(obj.serialBuffer) > 0 && ...
               ~endsWith(obj.serialBuffer, newline) && ...
               ~endsWith(obj.serialBuffer, char(13))

                obj.serialBuffer = lines(end);
                lines = lines(1:end-1);
            else
                obj.serialBuffer = "";
            end

            for idxLine = 1:numel(lines)
                line = strtrim(lines(idxLine));

                if strlength(line) == 0
                    continue;
                end

                lowLine = lower(line);

                if contains(lowLine, "time_ms") || startsWith(line, "CSV")
                    fprintf("[ARDUINO HEADER] %s\n", char(line));
                    continue;
                end

                isTextLog = startsWith(line, "WAIT") || ...
                    startsWith(line, "SKIP") || ...
                    startsWith(line, "#") || ...
                    startsWith(line, "[") || ...
                    startsWith(line, "{") || ...
                    contains(lowLine, "mpu") || ...
                    contains(lowLine, "emg") || ...
                    contains(lowLine, "dmp") || ...
                    contains(lowLine, "ready") || ...
                    contains(lowLine, "calibr") || ...
                    contains(lowLine, "found") || ...
                    contains(lowLine, "failed") || ...
                    contains(lowLine, "error");

                if isTextLog
                    obj.textLogCount = obj.textLogCount + 1;

                    if obj.textLogCount <= 30 || mod(obj.textLogCount, 20) == 0
                        fprintf("[ARDUINO] %s\n", char(line));
                    end

                    continue;
                end

                [okLine, t0, elbow0, raw0, upper0, forearm0, fmt0] = obj.parseArduinoLine(line);

                if ~okLine
                    obj.invalidLineCount = obj.invalidLineCount + 1;

                    if obj.invalidLineCount <= 5 || mod(obj.invalidLineCount, 50) == 0
                        fprintf("[SKIP INVALID] %s\n", char(line));
                    end

                    continue;
                end

                obj.sampleCount = obj.sampleCount + 1;
                obj.lastValidWallTime = tic;

                obj.latest_t_ms = t0;
                obj.latest_elbow = elbow0;
                obj.latest_raw = raw0;
                obj.latest_upperOk = upper0;
                obj.latest_forearmOk = forearm0;
                obj.latest_fmt = fmt0;

                ok = true;
                t_ms = t0;
                elbow_deg = elbow0;
                emg_raw = raw0;
                upperOk = upper0;
                forearmOk = forearm0;
                fmt = fmt0;
            end

            if toc(obj.lastValidWallTime) > 3.0 && obj.sampleCount > 0
                fprintf("[WARNING] No valid CSV for >3s. Arduino may be calibrating MPU, stuck, or output format changed.\n");
                obj.lastValidWallTime = tic;
            end

            if toc(obj.lastAnyByteWallTime) > 5.0
                fprintf("[WARNING] No Serial bytes for >5s. Check COM, cable, Arduino power, or close Serial Monitor.\n");
                obj.lastAnyByteWallTime = tic;
            end

            sampleCount = obj.sampleCount;
            invalidLineCount = obj.invalidLineCount;
            textLogCount = obj.textLogCount;
            noValidSec = toc(obj.lastValidWallTime);
            noByteSec = toc(obj.lastAnyByteWallTime);
        end

        function releaseImpl(obj)
            if ~isempty(obj.ser)
                flush(obj.ser);
                obj.ser = [];
                fprintf("\nSerial closed.\n");
            end
        end

        % ============================================================
        % Quan trong cho Simulink:
        % Khai bao output de Simulink khong can code generation qua stepImpl
        % ============================================================

        function num = getNumOutputsImpl(~)
            num = 12;
        end

        function varargout = getOutputSizeImpl(~)
            varargout = cell(1, 12);
            for k = 1:12
                varargout{k} = [1 1];
            end
        end

        function varargout = getOutputDataTypeImpl(~)
            varargout = { ...
                'logical', ... % ok
                'double',  ... % t_ms
                'double',  ... % elbow_deg
                'double',  ... % emg_raw
                'double',  ... % upperOk
                'double',  ... % forearmOk
                'double',  ... % fmt
                'double',  ... % sampleCount
                'double',  ... % invalidLineCount
                'double',  ... % textLogCount
                'double',  ... % noValidSec
                'double'};     % noByteSec
        end

        function varargout = isOutputComplexImpl(~)
            varargout = cell(1, 12);
            for k = 1:12
                varargout{k} = false;
            end
        end

        function varargout = isOutputFixedSizeImpl(~)
            varargout = cell(1, 12);
            for k = 1:12
                varargout{k} = true;
            end
        end

        function varargout = getOutputNamesImpl(~)
            varargout = { ...
                'ok', ...
                't_ms', ...
                'elbow_deg', ...
                'emg_raw', ...
                'upperOk', ...
                'forearmOk', ...
                'fmt', ...
                'sampleCount', ...
                'invalidLineCount', ...
                'textLogCount', ...
                'noValidSec', ...
                'noByteSec'};
        end
    end

    methods(Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            flag = false;
        end
    end

    methods(Access = private)

        function [ok, t_ms, elbow_deg, emg_raw, upperOk, forearmOk, fmt] = parseArduinoLine(~, line)
            ok = false;
            t_ms = NaN;
            elbow_deg = NaN;
            emg_raw = NaN;
            upperOk = NaN;
            forearmOk = NaN;
            fmt = 0;

            % fmt:
            % 0 = unknown
            % 1 = mpu_success_15col
            % 2 = test_6col
            % 3 = main_3col

            parts = split(line, ",");
            vals = nan(1, numel(parts));

            for k = 1:numel(parts)
                vals(k) = str2double(strtrim(parts(k)));
            end

            % MPU_success format:
            % time_ms, upper dXYZ, lower dXYZ, rel dXYZ, oldY, raw, median, filtered, emg_raw
            if numel(vals) >= 15 && all(~isnan(vals([1 14 15])))
                t_ms = vals(1);
                elbow_deg = vals(14);
                emg_raw = vals(15);
                upperOk = 1;
                forearmOk = 1;
                fmt = 1;
                ok = true;
                return;
            end

            % Main format: time_ms,elbow_deg,emg_raw
            % Test format: time_ms,upper_ok,forearm_ok,elbow_raw_deg,elbow_filtered_deg,emg_raw,...
            if numel(vals) >= 3 && all(~isnan(vals(1:3)))
                if numel(vals) >= 6 && ...
                   ismember(vals(2), [0 1]) && ...
                   ismember(vals(3), [0 1]) && ...
                   all(~isnan(vals(1:6)))

                    t_ms = vals(1);
                    upperOk = vals(2);
                    forearmOk = vals(3);
                    elbow_deg = vals(5);
                    emg_raw = vals(6);
                    fmt = 2;
                else
                    t_ms = vals(1);
                    elbow_deg = vals(2);
                    emg_raw = vals(3);
                    fmt = 3;
                end

                if ~isnan(t_ms) && ~isnan(elbow_deg) && ~isnan(emg_raw)
                    ok = true;
                end
            end
        end
    end
end
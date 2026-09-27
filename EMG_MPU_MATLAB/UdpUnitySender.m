classdef UdpUnitySender < matlab.System
    % Gui UDP sang Unity giong run_emg_live_local.m

    properties(Nontunable)
        UNITY_IP = "127.0.0.1"
        UNITY_PORT = 55001
        SEND_TO_UNITY = true
        UDP_RATE_HZ = 30
    end

    properties(Access = private)
        udpObj
        lastUdpSend
    end

    methods(Access = protected)

        function setupImpl(obj)
            if obj.SEND_TO_UNITY
                fprintf("[2/4] Opening UDP sender to Unity %s:%.0f...\n", ...
                    char(obj.UNITY_IP), obj.UNITY_PORT);

                obj.udpObj = udpport("datagram", "IPV4");

                fprintf("[OK] UDP ready. Start Unity Play Mode before or after this model.\n");
            else
                obj.udpObj = [];
                fprintf("[INFO] UDP disabled. Simulink only reads Serial and filters EMG.\n");
            end

            obj.lastUdpSend = tic;
        end

        function stepImpl(obj, t_ms, elbow_deg, emg_raw, env, baseline, active, baselineReady)
            if ~obj.SEND_TO_UNITY
                return;
            end

            if ~baselineReady || isnan(baseline)
                return;
            end

            minUdpPeriod = 1 / obj.UDP_RATE_HZ;

            if toc(obj.lastUdpSend) >= minUdpPeriod
                activeInt = double(active);
                elbowForUnity = min(max(elbow_deg, 0), 140);

                msg = sprintf('%.0f,%.3f,%.0f,%.6f,%.6f,%.6f,%.0f', ...
                    round(t_ms), elbowForUnity, emg_raw, env, baseline, env, activeInt);

                write(obj.udpObj, uint8(char(msg)), "uint8", obj.UNITY_IP, obj.UNITY_PORT);

                obj.lastUdpSend = tic;
            end
        end

        function releaseImpl(obj)
            if ~isempty(obj.udpObj)
                obj.udpObj = [];
                fprintf("UDP closed.\n");
            end
        end

        % Khai bao input/output cho Simulink
        function num = getNumInputsImpl(~)
            num = 7;
        end

        function num = getNumOutputsImpl(~)
            num = 0;
        end

        function varargout = getInputNamesImpl(~)
            varargout = { ...
                't_ms', ...
                'elbow_deg', ...
                'emg_raw', ...
                'env', ...
                'baseline', ...
                'active', ...
                'baselineReady'};
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
end
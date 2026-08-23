classdef GimbalUDPReceiver < matlab.System
    % GimbalUDPReceiver  Receive [x,y,z,pitch,yaw,distance] pose packets
    %   from the Raspberry Pi ArUco tracker over UDP and expose them as
    %   a Simulink signal.
    %
    %   Packet format (must match pi_gimbal_udp_stream.py):
    %     <6d I   ->  6 little-endian doubles + 1 uint32 sequence number
    %                 (52 bytes total)
    %
    %   Outputs:
    %     pose  : 1x6 double  [x y z pitch yaw distance]
    %     seq   : uint32      sequence number of the last packet received
    %     isNew : boolean     true only on the sample where a fresh
    %                         packet actually arrived (false = holding
    %                         the previous value, e.g. marker out of view)

    properties (Nontunable)
        % UDP port the Raspberry Pi is sending to. Must match
        % SIMULINK_PORT in pi_gimbal_udp_stream.py.
        Port (1,1) double {mustBeInteger, mustBePositive} = 25000

        % Local IP to bind to. '0.0.0.0' accepts on any interface.
        LocalIP (1,:) char = '0.0.0.0'
    end

    properties (Access = private)
        UDPSocket
        LastPose
        LastSeq
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.UDPSocket = udpport("datagram", "LocalPort", obj.Port, ...
                "LocalHost", obj.LocalIP, "Timeout", 0);
            obj.LastPose = zeros(1, 6);
            obj.LastSeq  = uint32(0);
        end

        function [pose, seq, isNew] = stepImpl(obj)
            isNew = false;
            pose  = obj.LastPose;
            seq   = obj.LastSeq;

            % Drain the socket and keep only the newest packet so the
            % model never lags behind if Simulink runs slower than the
            % Pi is sending.
            while obj.UDPSocket.NumDatagramsAvailable > 0
                try
                    raw = read(obj.UDPSocket, 52, "uint8");
                catch
                    % Datagram didn't match the expected 52-byte packet
                    % size (e.g. a stray packet) — drop it and move on.
                    flush(obj.UDPSocket);
                    break
                end

                vals   = typecast(uint8(raw(1:48)), 'double');
                seqVal = typecast(uint8(raw(49:52)), 'uint32');

                obj.LastPose = vals(:)';
                obj.LastSeq  = seqVal;
                pose  = obj.LastPose;
                seq   = obj.LastSeq;
                isNew = true;
            end
        end

        function releaseImpl(obj)
            obj.UDPSocket = [];
        end

        function resetImpl(obj)
            obj.LastPose = zeros(1, 6);
            obj.LastSeq  = uint32(0);
        end

        % --- Fixed output specs so Simulink can size the signals ------
        function num = getNumOutputsImpl(~)
            num = 3;
        end
        function [n1, n2, n3] = getOutputNamesImpl(~)
            n1 = 'pose'; n2 = 'seq'; n3 = 'isNew';
        end
        function [s1, s2, s3] = getOutputSizeImpl(~)
            s1 = [1 6]; s2 = [1 1]; s3 = [1 1];
        end
        function [t1, t2, t3] = getOutputDataTypeImpl(~)
            t1 = 'double'; t2 = 'uint32'; t3 = 'boolean';
        end
        function [c1, c2, c3] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false;
        end
        function [f1, f2, f3] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true;
        end
    end
end

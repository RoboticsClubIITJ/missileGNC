classdef GimbalUDPReceiver < matlab.System
    % Receive [x,y,z,pitch,yaw,distance] packets from servo.py over UDP.
    % Packet format: six little-endian doubles followed by one uint32.

    properties (Nontunable)
        Port (1,1) double {mustBeInteger, mustBePositive} = 25000
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
            obj.LastSeq = uint32(0);
        end

        function [pose, seq, isNew] = stepImpl(obj)
            pose = obj.LastPose;
            seq = obj.LastSeq;
            isNew = false;

            while obj.UDPSocket.NumDatagramsAvailable > 0
                try
                    raw = read(obj.UDPSocket, 52, "uint8");
                catch
                    flush(obj.UDPSocket);
                    break
                end

                obj.LastPose = typecast(uint8(raw(1:48)), 'double');
                obj.LastSeq = typecast(uint8(raw(49:52)), 'uint32');
                pose = obj.LastPose(:)';
                seq = obj.LastSeq;
                isNew = true;
            end
        end

        function releaseImpl(obj)
            obj.UDPSocket = [];
        end

        function resetImpl(obj)
            obj.LastPose = zeros(1, 6);
            obj.LastSeq = uint32(0);
        end

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

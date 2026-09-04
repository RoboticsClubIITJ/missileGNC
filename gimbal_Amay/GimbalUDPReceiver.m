classdef GimbalUDPReceiver < matlab.System
    % Receive [x,y,z,pitch,yaw,distance] packets from servo.py over UDP.
    % Packet format: six little-endian doubles followed by one uint32.
    %
    % Uses Java's DatagramSocket directly (bundled with every MATLAB
    % install) instead of udpport, so no Instrument Control Toolbox
    % is required.

    properties (Nontunable)
        Port (1,1) double {mustBeInteger, mustBePositive} = 25000
    end

    properties (Access = private)
        UDPSocket
        LastPose
        LastSeq
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.UDPSocket = java.net.DatagramSocket(obj.Port);
            obj.UDPSocket.setSoTimeout(1);   % ms; non-blocking-ish poll
            obj.LastPose = zeros(1, 6);
            obj.LastSeq = uint32(0);
        end

        function [pose, seq, isNew] = stepImpl(obj)
            pose = obj.LastPose;
            seq = obj.LastSeq;
            isNew = false;

            while true
                buf = zeros(1, 1024, 'int8');
                packet = java.net.DatagramPacket(buf, numel(buf));

                try
                    obj.UDPSocket.receive(packet);
                catch
                    break
                end

                len = packet.getLength();
                data = packet.getData();
                raw = typecast(int8(data(1:len)), 'uint8');

                if numel(raw) == 52
                    obj.LastPose = reshape(typecast(uint8(raw(1:48)), 'double'), 1, 6);
                    obj.LastSeq = typecast(uint8(raw(49:52)), 'uint32');
                    pose = obj.LastPose;
                    seq = obj.LastSeq;
                    isNew = true;
                end
            end
        end

        function releaseImpl(obj)
            if ~isempty(obj.UDPSocket)
                obj.UDPSocket.close();
                obj.UDPSocket = [];
            end
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
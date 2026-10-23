% build_gimbal_model.m
%
% Builds a Simulink model (gimbal_model.slx) that receives the
% [x, y, z, pitch, yaw, distance] pose stream from the Raspberry Pi
% ArUco tracker over UDP and splits it into six scoped signals.
%
% Prerequisites:
%   - GimbalUDPReceiver.m on the MATLAB path (same folder is fine)
%   - pi_gimbal_udp_stream.py running on the Pi, sending to this
%     machine's IP on the port configured below
%
% Run this once, then open gimbal_model.slx in Simulink and hit Run.

modelName = 'gimbal_model';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

% --- MATLAB System block: UDP receiver ------------------------------
recvBlk = [modelName '/Gimbal UDP Receiver'];
add_block('sysobjblks/MATLAB System', recvBlk, 'Position', [40 80 220 160]);
set_param(recvBlk, 'System', 'GimbalUDPReceiver');

% To change the listening port (must match SIMULINK_PORT in the
% Python script), edit it after the block exists:
% set_param(recvBlk, 'Port', '25000');

% --- Demux: split the 6-element pose vector into 6 scalar signals ---
demuxBlk = [modelName '/Demux'];
add_block('simulink/Signal Routing/Demux', demuxBlk, 'Position', [280 70 310 170]);
set_param(demuxBlk, 'Outputs', '6');

add_line(modelName, 'Gimbal UDP Receiver/1', 'Demux/1');

% --- One scope per channel ------------------------------------------
chanNames = {'X', 'Y', 'Z', 'Pitch', 'Yaw', 'Distance'};
scopeTop = 40;
for k = 1:6
    scopeBlk = sprintf('%s/Scope_%s', modelName, chanNames{k});
    add_block('simulink/Sinks/Scope', scopeBlk, 'Position', [400 scopeTop 460 scopeTop+40]);
    add_line(modelName, sprintf('Demux/%d', k), sprintf('Scope_%s/1', chanNames{k}));
    scopeTop = scopeTop + 55;
end

% --- Diagnostics: sequence number + "new data" flag ------------------
add_block('simulink/Sinks/Scope', [modelName '/Scope_Seq'], 'Position', [400 400 460 440]);
add_line(modelName, 'Gimbal UDP Receiver/2', 'Scope_Seq/1');

add_block('simulink/Sinks/Scope', [modelName '/Scope_IsNew'], 'Position', [400 460 460 500]);
add_line(modelName, 'Gimbal UDP Receiver/3', 'Scope_IsNew/1');

save_system(modelName, fullfile(pwd, [modelName '.slx']));
disp(['Model saved to ' fullfile(pwd, [modelName '.slx'])]);

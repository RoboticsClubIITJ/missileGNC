% Build a local UDP monitor for the pose stream produced by servo.py.
% Add final-product/simulink to the MATLAB path, then run this script.
% Creates udp_monitor.slx in the current folder (ignored by git).

modelName = 'udp_monitor';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

new_system(modelName);
open_system(modelName);

receiver = [modelName '/Gimbal UDP Receiver'];
add_block('sysobjblks/MATLAB System', receiver, ...
    'Position', [40 100 220 180]);
set_param(receiver, 'System', 'GimbalUDPReceiver');

mux = [modelName '/Pose Demux'];
add_block('simulink/Signal Routing/Demux', mux, ...
    'Position', [280 90 320 210]);
set_param(mux, 'Outputs', '6');
add_line(modelName, 'Gimbal UDP Receiver/1', 'Pose Demux/1');

names = {'X', 'Y', 'Z', 'Pitch', 'Yaw', 'Distance'};
for index = 1:6
    scope = [modelName '/Scope_' names{index}];
    add_block('simulink/Sinks/Scope', scope, ...
        'Position', [400 40 + (index - 1) * 60 480 80 + (index - 1) * 60]);
    add_line(modelName, sprintf('Pose Demux/%d', index), ...
        sprintf('Scope_%s/1', names{index}));
end

save_system(modelName, fullfile(pwd, [modelName '.slx']));
open_system(modelName);

mdl = 'anjaneya_V12';
open_system(mdl);

fig = figure('Name','Live Target Trajectory');
h = animatedline('Color','b','LineWidth',1.5);
axis equal; grid on;
xlabel('X Target (m)'); ylabel('Y Target (m)');
title('Live Target Trajectory (running simulation)');

lastN = 0;

set_param(mdl, 'SimulationCommand', 'start');

while strcmp(get_param(mdl, 'SimulationStatus'), 'running') || ...
      strcmp(get_param(mdl, 'SimulationStatus'), 'paused')

    % Stop cleanly if the plot window was closed
    if ~isvalid(h) || ~isvalid(fig)
        disp('Plot window closed -- stopping live update (simulation keeps running).');
        break;
    end

    set_param(mdl, 'SimulationCommand', 'WriteDataLogs');

    if evalin('base', 'exist(''Xt'',''var'')') && evalin('base', 'exist(''Yt'',''var'')')
        xT = evalin('base', 'Xt.Data');
        yT = evalin('base', 'Yt.Data');

        xT = double(squeeze(xT)); xT = xT(:);
        yT = double(squeeze(yT)); yT = yT(:);

        n = min(length(xT), length(yT));
        if n > lastN
            addpoints(h, xT(lastN+1:n), yT(lastN+1:n));
            lastN = n;
            drawnow limitrate;
        end
    end

    pause(0.1);
end

disp('Loop finished.');
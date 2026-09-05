mdl = 'anjaneya_V9';   % <-- your Simulink model name
open_system(mdl);

% Set up the live plot
figure('Name','Live Target Trajectory');
h = animatedline('Color','b','LineWidth',1.5);
axis equal; grid on;
xlabel('X Target (m)'); ylabel('Y Target (m)');
title('Live Target Trajectory (running simulation)');

lastN = 0;

% Start the simulation WITHOUT blocking MATLAB
set_param(mdl, 'SimulationCommand', 'start');

% Poll while the model is running
while strcmp(get_param(mdl, 'SimulationStatus'), 'running') || ...
      strcmp(get_param(mdl, 'SimulationStatus'), 'paused')

    if evalin('base', 'exist(''Xt'',''var'')')
        xT = evalin('base', 'Xt.Data');
        yT = evalin('base', 'Yt.Data');

        n = length(xT);
        if n > lastN
            addpoints(h, xT(lastN+1:n), yT(lastN+1:n));
            lastN = n;
            drawnow limitrate;
        end
    end

    pause(0.05);   % polling interval — tune to taste
end

disp('Simulation finished.');
% =========================================================================
%  live_plot_target.m
% -------------------------------------------------------------------------
%  Live (animated) 2D plot of the TARGET trajectory, built from the
%  Xt / Yt / Vtx / Vty signals that anjaneya_V10.slx already logs via its
%  "To Workspace" blocks (SaveFormat = Timeseries).
%
%  WHAT IT DOES
%    1. Gets a Simulink.SimulationOutput ("out") -- either the one already
%       sitting in your base workspace from a previous Run, or by
%       simulating the model fresh.
%    2. Resamples Xt/Yt onto a uniform time grid (the solver is variable
%       step, so raw samples are not evenly spaced -- resampling gives
%       smooth, constant-speed playback).
%    3. Animates the target as a moving dot with a fading trail and a
%       live velocity arrow, plus a text readout of time/position/speed.
%
%  HOW TO USE
%    - Set MODEL_NAME below (no .slx extension).
%    - Just press Run. If "out" is not already in the base workspace,
%      the model is simulated automatically.
%    - Adjust PLAYBACK_SPEED / FRAME_DT to taste.
%
%  REQUIRES: base MATLAB + Simulink only (no extra toolboxes).
% =========================================================================

clear; clc;

%% ---------------------------- USER SETTINGS ----------------------------
MODEL_NAME     = 'anjaneya_V11';   % model to simulate if 'out' isn't present
PLAYBACK_SPEED = 1.0;              % 1.0 = real time, 2.0 = 2x faster, 0.5 = slower
FRAME_DT       = 0.02;             % animation frame period [s]
AXIS_PAD_FRAC  = 0.15;             % fractional padding around the trajectory bbox
TRAIL_COLOR    = [0.85 0.33 0.10]; % target trail / marker color (orange)
VEL_COLOR      = [0.00 0.45 0.74]; % velocity-arrow color (blue)

%% ---------------------------- GET SIM DATA ------------------------------
out = local_getSimOutput(MODEL_NAME);

t_raw  = out.Xt.Time;
Xt_raw = out.Xt.Data;
Yt_raw = out.Yt.Data;

haveVel = true;
try
    Vtx_raw = out.Vtx.Data;
    Vty_raw = out.Vty.Data;
catch
    haveVel = false;
end

%% ---------------------- RESAMPLE FOR SMOOTH PLAYBACK --------------------
t_uniform = (t_raw(1):FRAME_DT:t_raw(end))';
Xt = interp1(t_raw, Xt_raw, t_uniform, 'linear', 'extrap');
Yt = interp1(t_raw, Yt_raw, t_uniform, 'linear', 'extrap');
if haveVel
    Vtx = interp1(t_raw, Vtx_raw, t_uniform, 'linear', 'extrap');
    Vty = interp1(t_raw, Vty_raw, t_uniform, 'linear', 'extrap');
end

%% ------------------------------ FIGURE SETUP -----------------------------
fig = figure('Name', 'Target Trajectory - Live', 'Color', 'w', 'NumberTitle', 'off');
ax  = axes(fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X position (m)'); ylabel(ax, 'Y position (m)');
title(ax, 'Target Trajectory (Live)');

xSpan = max(Xt) - min(Xt); if xSpan == 0, xSpan = 1; end
ySpan = max(Yt) - min(Yt); if ySpan == 0, ySpan = 1; end
xlim(ax, [min(Xt) - AXIS_PAD_FRAC*xSpan, max(Xt) + AXIS_PAD_FRAC*xSpan]);
ylim(ax, [min(Yt) - AXIS_PAD_FRAC*ySpan, max(Yt) + AXIS_PAD_FRAC*ySpan]);

trailLine = animatedline(ax, 'Color', TRAIL_COLOR, 'LineWidth', 1.5);
targetDot = plot(ax, Xt(1), Yt(1), 'o', 'MarkerSize', 10, ...
                  'MarkerFaceColor', TRAIL_COLOR, 'MarkerEdgeColor', 'k');

if haveVel
    velArrow = quiver(ax, Xt(1), Yt(1), Vtx(1), Vty(1), 0, ...
                       'Color', VEL_COLOR, 'LineWidth', 1.5, 'MaxHeadSize', 2);
end

infoTxt = text(ax, 0.02, 0.98, '', 'Units', 'normalized', ...
               'VerticalAlignment', 'top', 'FontName', 'FixedWidth', ...
               'FontSize', 9, 'BackgroundColor', [1 1 1 0.7]);

%% -------------------------------- ANIMATE --------------------------------
for k = 1:numel(t_uniform)
    addpoints(trailLine, Xt(k), Yt(k));
    set(targetDot, 'XData', Xt(k), 'YData', Yt(k));

    if haveVel
        set(velArrow, 'XData', Xt(k), 'YData', Yt(k), ...
                       'UData', Vtx(k), 'VData', Vty(k));
        speedStr = sprintf('Speed : %7.2f m/s\n', hypot(Vtx(k), Vty(k)));
    else
        speedStr = '';
    end

    set(infoTxt, 'String', sprintf('t = %6.2f s\nX = %7.2f m\nY = %7.2f m\n%s', ...
                                    t_uniform(k), Xt(k), Yt(k), speedStr));

    drawnow limitrate;
    pause(FRAME_DT / PLAYBACK_SPEED);
end

disp('Target playback complete.');

%% ============================ LOCAL FUNCTIONS ============================
function out = local_getSimOutput(modelName)
% Returns a Simulink.SimulationOutput with Xt/Yt logged. Reuses "out" from
% the base workspace if it is already valid; otherwise simulates fresh.
    if evalin('base', 'exist(''out'',''var'')')
        candidate = evalin('base', 'out');
        try
            candidate.Xt; %#ok<VUNUS> -- just probing that the field exists
            out = candidate;
            fprintf('Using existing ''out'' found in base workspace.\n');
            return
        catch
            % fall through and simulate fresh
        end
    end
    fprintf('Simulating %s ...\n', modelName);
    load_system(modelName);
    out = sim(modelName);
    assignin('base', 'out', out);
end
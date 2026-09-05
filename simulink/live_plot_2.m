% =========================================================================
%  live_plot_engagement.m
% -------------------------------------------------------------------------
%  Live (animated) 2D plot of the full engagement: MISSILE chasing the
%  TARGET, built from the Xm/Ym (missile) and Xt/Yt (target) signals that
%  anjaneya_V10.slx already logs via its "To Workspace" blocks
%  (SaveFormat = Timeseries).
%
%  WHAT IT DOES
%    1. Gets a Simulink.SimulationOutput ("out") -- either the one already
%       sitting in your base workspace, or by simulating the model fresh.
%    2. Resamples Xm/Ym/Xt/Yt onto a common uniform time grid for smooth,
%       constant-speed playback (the solver is variable-step).
%    3. Animates both missile and target as moving dots with fading
%       trails, draws a live line-of-sight (LOS) line between them, and
%       shows a live readout of time / range / closing speed.
%    4. Detects intercept (range <= R_HIT) and freezes the animation with
%       an "INTERCEPT" marker + message. If the missile instead diverges
%       (range grows far beyond its initial value with no intercept), it
%       prints a clear warning at the end instead of silently finishing --
%       this is useful for *watching the current guidance bug happen*.
%
%  HOW TO USE
%    - Set MODEL_NAME below (no .slx extension).
%    - Just press Run. If "out" is not already in the base workspace,
%      the model is simulated automatically.
%    - Adjust PLAYBACK_SPEED / FRAME_DT / R_HIT to taste.
%    - If the missile is currently diverging (known open bug), set
%      AUTO_RESCALE = true (default) so the view keeps both trajectories
%      visible; set it false and fill in MANUAL_XLIM/YLIM for a fixed view.
%
%  REQUIRES: base MATLAB + Simulink only (no extra toolboxes).
% =========================================================================

clear; clc;

%% ---------------------------- USER SETTINGS ----------------------------
MODEL_NAME     = 'anjaneya_V10';   % model to simulate if 'out' isn't present
PLAYBACK_SPEED = 1.0;              % 1.0 = real time, 2.0 = 2x faster, 0.5 = slower
FRAME_DT       = 0.02;             % animation frame period [s]
R_HIT          = 0.5;              % intercept range threshold [m] (matches the
                                    % model's own stop-condition: R < 0.5)
AXIS_PAD_FRAC  = 0.15;             % fractional padding around the trajectory bbox

AUTO_RESCALE   = true;             % true: auto-fit axes to both trajectories
MANUAL_XLIM    = [-50 200];        % used only if AUTO_RESCALE = false
MANUAL_YLIM    = [-50 200];        % used only if AUTO_RESCALE = false

MISSILE_COLOR  = [0.00 0.45 0.74]; % blue
TARGET_COLOR   = [0.85 0.33 0.10]; % orange
LOS_COLOR      = [0.50 0.50 0.50]; % grey dashed line-of-sight

%% ---------------------------- GET SIM DATA ------------------------------
out = local_getSimOutput(MODEL_NAME);

tM_raw  = out.Xm.Time;   Xm_raw = out.Xm.Data;   Ym_raw = out.Ym.Data;
tT_raw  = out.Xt.Time;   Xt_raw = out.Xt.Data;   Yt_raw = out.Yt.Data;

%% ---------------------- RESAMPLE FOR SMOOTH PLAYBACK --------------------
t0 = max(tM_raw(1),   tT_raw(1));
t1 = min(tM_raw(end), tT_raw(end));
t  = (t0:FRAME_DT:t1)';

Xm = interp1(tM_raw, Xm_raw, t, 'linear', 'extrap');
Ym = interp1(tM_raw, Ym_raw, t, 'linear', 'extrap');
Xt = interp1(tT_raw, Xt_raw, t, 'linear', 'extrap');
Yt = interp1(tT_raw, Yt_raw, t, 'linear', 'extrap');

range_t = hypot(Xt - Xm, Yt - Ym);

hitIdx = find(range_t <= R_HIT, 1, 'first');

%% ------------------------------ FIGURE SETUP -----------------------------
fig = figure('Name', 'Engagement - Live', 'Color', 'w', 'NumberTitle', 'off');
ax  = axes(fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'X position (m)'); ylabel(ax, 'Y position (m)');
title(ax, 'Missile vs Target Engagement (Live)');

if AUTO_RESCALE
    allX = [Xm; Xt]; allY = [Ym; Yt];
    xSpan = max(allX) - min(allX); if xSpan == 0, xSpan = 1; end
    ySpan = max(allY) - min(allY); if ySpan == 0, ySpan = 1; end
    xlim(ax, [min(allX) - AXIS_PAD_FRAC*xSpan, max(allX) + AXIS_PAD_FRAC*xSpan]);
    ylim(ax, [min(allY) - AXIS_PAD_FRAC*ySpan, max(allY) + AXIS_PAD_FRAC*ySpan]);
else
    xlim(ax, MANUAL_XLIM);
    ylim(ax, MANUAL_YLIM);
end

missileTrail = animatedline(ax, 'Color', MISSILE_COLOR, 'LineWidth', 1.5);
targetTrail  = animatedline(ax, 'Color', TARGET_COLOR,  'LineWidth', 1.5);

missileDot = plot(ax, Xm(1), Ym(1), '^', 'MarkerSize', 11, ...
                   'MarkerFaceColor', MISSILE_COLOR, 'MarkerEdgeColor', 'k');
targetDot  = plot(ax, Xt(1), Yt(1), 'o', 'MarkerSize', 10, ...
                   'MarkerFaceColor', TARGET_COLOR, 'MarkerEdgeColor', 'k');

losLine = plot(ax, [Xm(1) Xt(1)], [Ym(1) Yt(1)], '--', 'Color', LOS_COLOR);

legend(ax, [missileDot, targetDot], {'Missile', 'Target'}, 'Location', 'best');

infoTxt = text(ax, 0.02, 0.98, '', 'Units', 'normalized', ...
               'VerticalAlignment', 'top', 'FontName', 'FixedWidth', ...
               'FontSize', 9, 'BackgroundColor', [1 1 1 0.7]);

%% -------------------------------- ANIMATE --------------------------------
nSteps = numel(t);
if ~isempty(hitIdx)
    nSteps = hitIdx;   % stop the animation right at intercept
end

for k = 1:nSteps
    addpoints(missileTrail, Xm(k), Ym(k));
    addpoints(targetTrail,  Xt(k), Yt(k));

    set(missileDot, 'XData', Xm(k), 'YData', Ym(k));
    set(targetDot,  'XData', Xt(k), 'YData', Yt(k));
    set(losLine,    'XData', [Xm(k) Xt(k)], 'YData', [Ym(k) Yt(k)]);

    set(infoTxt, 'String', sprintf(['t     = %6.2f s\n' ...
                                     'Range = %7.2f m\n' ...
                                     'Missile = (%7.2f, %7.2f)\n' ...
                                     'Target  = (%7.2f, %7.2f)'], ...
                                    t(k), range_t(k), Xm(k), Ym(k), Xt(k), Yt(k)));

    drawnow limitrate;
    pause(FRAME_DT / PLAYBACK_SPEED);
end

%% ------------------------------- OUTCOME ---------------------------------
if ~isempty(hitIdx)
    plot(ax, Xm(hitIdx), Ym(hitIdx), 'rx', 'MarkerSize', 20, 'LineWidth', 3);
    text(ax, Xm(hitIdx), Ym(hitIdx), '  INTERCEPT!', 'Color', 'r', ...
         'FontWeight', 'bold', 'FontSize', 12);
    fprintf('\nINTERCEPT at t = %.2f s, range = %.3f m.\n', t(hitIdx), range_t(hitIdx));
else
    initialRange = range_t(1);
    finalRange   = range_t(end);
    fprintf('\nNo intercept detected over the logged time span.\n');
    fprintf('  Initial range : %.2f m\n', initialRange);
    fprintf('  Final range   : %.2f m\n', finalRange);
    if finalRange > 5 * initialRange
        warning(['Range GREW substantially instead of closing -- this looks like ' ...
                 'the known missile-divergence bug rather than a normal miss. ' ...
                 'Check the Autopilot/Force-Resolver sign chain and the target ' ...
                 'velocity Derivative blocks before trusting guidance results.']);
    end
end

%% ============================ LOCAL FUNCTIONS ============================
function out = local_getSimOutput(modelName)
% Returns a Simulink.SimulationOutput with Xm/Ym/Xt/Yt logged. Reuses "out"
% from the base workspace if it is already valid; otherwise simulates fresh.
    if evalin('base', 'exist(''out'',''var'')')
        candidate = evalin('base', 'out');
        try
            candidate.Xm; candidate.Xt; %#ok<VUNUS> -- probe required fields
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
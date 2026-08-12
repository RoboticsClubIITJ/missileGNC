% ==========================================
% Data Extraction & Vectorization
% ==========================================
% Ensure data arrays are forced into column vectors
xM = out.Xm.Data(:);
yM = out.Ym.Data(:);

xT = out.Xt.Data(:);
yT = out.Yt.Data(:);

% ==========================================
% Collision Verification Check
% ==========================================
% Calculate relative distance between missile and target over time
dist = sqrt((xM - xT).^2 + (yM - yT).^2);
[minDist, minIdx] = min(dist);

fprintf('\n=== INTERCEPT DIAGNOSTICS ===\n');
fprintf('Minimum Distance (Miss Distance): %.4f m\n', minDist);
fprintf('Occurred at sample index: %d of %d\n', minIdx, length(dist));
if isprop(out.Xm, 'Time')
    fprintf('Occurred at time: %.4f s\n', out.Xm.Time(minIdx));
end
fprintf('=============================\n\n');

% ==========================================
% Figure & Axes Initialization
% ==========================================
% Close old windows to clear stuck graphics handles
close all;

fig = figure('Name', 'Missile-Target Engagement', 'Color', 'w', 'NumberTitle', 'off');
ax = axes('Parent', fig);

% Pre-calculate spatial boundaries for fixed axes limits
allX = [xM; xT];
allY = [yM; yT];
xMin = min(allX); xMax = max(allX);
yMin = min(allY); yMax = max(allY);

dx = max(xMax - xMin, 1);
dy = max(yMax - yMin, 1);
margin = 0.10 * max(dx, dy);

axis(ax, [xMin - margin, xMax + margin, yMin - margin, yMax + margin]);
grid(ax, 'on');
hold(ax, 'on');
axis(ax, 'equal');

xlabel(ax, 'X Position (m)');
ylabel(ax, 'Y Position (m)');
title(ax, 'Missile vs Target Trajectory Animation');

% Plot trailing path objects
hMissileLine = plot(ax, NaN, NaN, 'r-', 'LineWidth', 2, 'DisplayName', 'Missile Path');
hTargetLine  = plot(ax, NaN, NaN, 'b--', 'LineWidth', 2, 'DisplayName', 'Target Path');

% Plot moving heads
hMissileHead = plot(ax, xM(1), yM(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Missile');
hTargetHead  = plot(ax, xT(1), yT(1), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Target');

legend(ax, [hMissileLine, hTargetLine, hMissileHead, hTargetHead], 'Location', 'northeast');

% ==========================================
% Explicit Animation Loop
% ==========================================
numSamples = min([length(xM), length(yM), length(xT), length(yT)]);

% Adjust step size if sample count is large (1ms dt can result in tens of thousands of frames)
if numSamples > 2000
    stepSize = ceil(numSamples / 500); % Targets ~500 rendered frames
else
    stepSize = 5;
end

for k = 1:stepSize:numSamples
    % Update trailing paths
    set(hMissileLine, 'XData', xM(1:k), 'YData', yM(1:k));
    set(hTargetLine,  'XData', xT(1:k), 'YData', yT(1:k));
    
    % Move marker heads
    set(hMissileHead, 'XData', xM(k), 'YData', yM(k));
    set(hTargetHead,  'XData', xT(k), 'YData', yT(k));
    
    % Force render step (pause forces UI thread execution)
    pause(0.01);
end

% Render final frame explicitly
set(hMissileLine, 'XData', xM(1:numSamples), 'YData', yM(1:numSamples));
set(hTargetLine,  'XData', xT(1:numSamples), 'YData', yT(1:numSamples));
set(hMissileHead, 'XData', xM(numSamples), 'YData', yM(numSamples));
set(hTargetHead,  'XData', xT(numSamples), 'YData', yT(numSamples));

% Mark point of closest approach
plot(ax, xM(minIdx), yM(minIdx), 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'y', 'DisplayName', 'Impact Point');

% ==========================================
% Print Reference Statements (Preserved)
% ==========================================
fprintf('\n--- SIMULATION DATA ---\n');

fprintf('Samples Xm: %d\n', length(out.Xm.Data));
fprintf('Samples Ym: %d\n', length(out.Ym.Data));
fprintf('Samples Xt: %d\n', length(out.Xt.Data));
fprintf('Samples Yt: %d\n', length(out.Yt.Data));

fprintf('\n--- TIME ---\n');
fprintf('Start time: %g s\n', out.Xm.Time(1));
fprintf('End time:   %g s\n', out.Xm.Time(end));

fprintf('\n--- MISSILE ---\n');
fprintf('Xm: %g -> %g\n', out.Xm.Data(1), out.Xm.Data(end));
fprintf('Ym: %g -> %g\n', out.Ym.Data(1), out.Ym.Data(end));

fprintf('\n--- TARGET ---\n');
fprintf('Xt: %g -> %g\n', out.Xt.Data(1), out.Xt.Data(end));
fprintf('Yt: %g -> %g\n', out.Yt.Data(1), out.Yt.Data(end));

fprintf('\n--- NaN CHECK ---\n');
fprintf('Xm NaN: %d\n', any(isnan(out.Xm.Data)));
fprintf('Ym NaN: %d\n', any(isnan(out.Ym.Data)));
fprintf('Xt NaN: %d\n', any(isnan(out.Xt.Data)));
fprintf('Yt NaN: %d\n', any(isnan(out.Yt.Data)));

fprintf('\n--- Inf CHECK ---\n');
fprintf('Xm Inf: %d\n', any(isinf(out.Xm.Data)));
fprintf('Ym Inf: %d\n', any(isinf(out.Ym.Data)));
fprintf('Xt Inf: %d\n', any(isinf(out.Xt.Data)));
fprintf('Yt Inf: %d\n', any(isinf(out.Yt.Data)));
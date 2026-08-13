% ==========================================
% Data Extraction & Vectorization
% ==========================================
% Position data
xM = out.Xm.Data(:);
yM = out.Ym.Data(:);

xT = out.Xt.Data(:);
yT = out.Yt.Data(:);

% Thrust data (Magnitude and Angle relative to velocity)
% Adjust field names if they differ in your workspace/out struct
T_mag = out.Thrust.Data(:); 
T_ang = out.ThrustAngle.Data(:); % Angle relative to velocity direction

% Convert thrust angle to radians if provided in degrees
if max(abs(T_ang)) > 2*pi
    T_ang_rad = deg2rad(T_ang);
else
    T_ang_rad = T_ang;
end

% Compute missile velocity vector direction (heading angle gamma)
vx = [diff(xM); 0]; % Pad last element to preserve array length
vy = [diff(yM); 0];
heading_angle = atan2(vy, vx);

% Total Thrust Direction = Missile Heading + Thrust Deflection Angle
thrust_total_angle = heading_angle + T_ang_rad;

% Compute 2D Thrust Vector Components
Fx = T_mag .* cos(thrust_total_angle);
Fy = T_mag .* sin(thrust_total_angle);

% Scale factor for thrust vector visualization (adjust if arrow is too big/small)
maxPosRange = max(max(xM)-min(xM), max(yM)-min(yM));
maxThrust = max(T_mag);
if maxThrust > 0
    scaleFactor = 0.08 * maxPosRange / maxThrust;
else
    scaleFactor = 1;
end

% ==========================================
% Collision Verification Check
% ==========================================
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
close all;

fig = figure('Name', 'Missile-Target Engagement with Thrust Vector', 'Color', 'w', 'NumberTitle', 'off');
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
title(ax, 'Missile vs Target Trajectory & Thrust Vector Animation');

% Plot trailing path objects
hMissileLine = plot(ax, NaN, NaN, 'r-', 'LineWidth', 2, 'DisplayName', 'Missile Path');
hTargetLine  = plot(ax, NaN, NaN, 'b--', 'LineWidth', 2, 'DisplayName', 'Target Path');

% Plot moving heads
hMissileHead = plot(ax, xM(1), yM(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Missile');
hTargetHead  = plot(ax, xT(1), yT(1), 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Target');

% Plot dynamic Thrust Vector (Quiver object attached to missile position)
hThrustVec = quiver(ax, xM(1), yM(1), Fx(1)*scaleFactor, Fy(1)*scaleFactor, 0, ...
    'Color', [0.85 0.33 0.1], 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', 'Thrust Vector');

legend(ax, [hMissileLine, hTargetLine, hMissileHead, hTargetHead, hThrustVec], 'Location', 'northeast');

% ==========================================
% Explicit Animation Loop
% ==========================================
numSamples = min([length(xM), length(yM), length(xT), length(yT), length(Fx), length(Fy)]);

if numSamples > 2000
    stepSize = ceil(numSamples / 500); 
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
    
    % Update Thrust Vector position & components
    set(hThrustVec, 'XData', xM(k), 'YData', yM(k), ...
                    'UData', Fx(k)*scaleFactor, 'VData', Fy(k)*scaleFactor);
    
    % Force render step
    pause(0.01);
end

% Render final frame explicitly
set(hMissileLine, 'XData', xM(1:numSamples), 'YData', yM(1:numSamples));
set(hTargetLine,  'XData', xT(1:numSamples), 'YData', yT(1:numSamples));
set(hMissileHead, 'XData', xM(numSamples), 'YData', yM(numSamples));
set(hTargetHead,  'XData', xT(numSamples), 'YData', yT(numSamples));
set(hThrustVec,   'XData', xM(numSamples), 'YData', yM(numSamples), ...
                  'UData', Fx(numSamples)*scaleFactor, 'VData', Fy(numSamples)*scaleFactor);

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
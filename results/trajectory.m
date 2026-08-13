xM = out.Xm.Data(:);
yM = out.Ym.Data(:);

xT = out.Xt.Data(:);
yT = out.Yt.Data(:);

figure('Color','w');

plot(xM, yM, 'LineWidth', 2);
hold on;

plot(xT, yT, 'LineWidth', 2);

grid on;

xlabel('X Position (m)');
ylabel('Y Position (m)');

legend('Missile', 'Target');

title('Missile vs Target Trajectory');

axis tight;


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
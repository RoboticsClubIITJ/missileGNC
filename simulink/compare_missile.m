%% ============================================================================
%  MISSILE ENGAGEMENT COMPARISON : IDEAL  vs  LIMITED
%  ---------------------------------------------------------------------------
%  1. Runs both Simulink models via sim()
%  2. Side-by-side LIVE animation (trajectories + thrust vector arrows)
%  3. Signal comparison table in the Command Window
%  4. Accuracy matrix (intercept metrics + trajectory error stats)
%
%  Expects both models to log: Xm Ym Xt Yt Vmx Vmy Vm Vtx Vty Vt
%                              gammaT gammaH Thrust ThrustAngle
% ============================================================================
clear; clc; close all;

%% ------------------------------ USER CONFIG --------------------------------
MODEL_IDEAL   = 'anjaneya_V5';            % ideal model   (no .slx extension)
MODEL_LIMITED = 'anjaneya_V7';    % limited model (no .slx extension)

N_GRID        = 3000;    % samples on the common time grid (metrics resolution)
N_FRAMES      = 400;     % number of animation frames
FRAME_PAUSE   = 0.010;   % seconds between frames (playback speed)
ARROW_FRAC    = 0.08;    % thrust arrow length as fraction of plot span
HIT_RADIUS    = 5.0;     % m - miss distance below this counts as an intercept

%% ------------------------------ RUN THE MODELS -----------------------------
fprintf('Running IDEAL   model : %s ...\n', MODEL_IDEAL);
outI = runModel(MODEL_IDEAL);

fprintf('Running LIMITED model : %s ...\n', MODEL_LIMITED);
outL = runModel(MODEL_LIMITED);
fprintf('Both runs complete.\n');

%% ------------------------------ EXTRACT ------------------------------------
A = extractRun(outI, 'IDEAL');
B = extractRun(outL, 'LIMITED');

% Common time grid (same stop time, but solver step counts may differ)
t0 = max(A.t(1),   B.t(1));
tf = min(A.t(end), B.t(end));
tc = linspace(t0, tf, N_GRID)';

Ar = resampleRun(A, tc);
Br = resampleRun(B, tc);

% Derived kinematics on the common grid
Ar = deriveRun(Ar);
Br = deriveRun(Br);

%% ------------------------------ ANIMATION ----------------------------------
% Shared axis limits so the two panels are visually comparable
allX = [Ar.sig.xM; Ar.sig.xT; Br.sig.xM; Br.sig.xT];
allY = [Ar.sig.yM; Ar.sig.yT; Br.sig.yM; Br.sig.yT];
xMin = min(allX); xMax = max(allX);
yMin = min(allY); yMax = max(allY);
span = max([xMax-xMin, yMax-yMin, 1]);
mrg  = 0.10*span;
LIM  = [xMin-mrg, xMax+mrg, yMin-mrg, yMax+mrg];

% Shared thrust scaling so arrow lengths mean the same thing in both panels
maxT = max([max(abs(Ar.sig.Tmag)), max(abs(Br.sig.Tmag))]);
if maxT > 0, sf = ARROW_FRAC*span/maxT; else, sf = 1; end

fig = figure('Name','Ideal vs Limited - Engagement Animation', ...
             'Color','w','NumberTitle','off','Position',[80 80 1400 640]);
tl  = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');

R(1) = buildPanel(nexttile(tl), Ar, LIM, sf, true);
R(2) = buildPanel(nexttile(tl), Br, LIM, sf, false);

hSG = sgtitle(tl, sprintf('t = %6.3f s', tc(1)), ...
              'FontWeight','bold','FontSize',13);

frames = unique(round(linspace(1, N_GRID, N_FRAMES)));
for k = frames
    for p = 1:2
        S = R(p).S;
        set(R(p).hMline, 'XData', S.sig.xM(1:k), 'YData', S.sig.yM(1:k));
        set(R(p).hTline, 'XData', S.sig.xT(1:k), 'YData', S.sig.yT(1:k));
        set(R(p).hMhead, 'XData', S.sig.xM(k),   'YData', S.sig.yM(k));
        set(R(p).hThead, 'XData', S.sig.xT(k),   'YData', S.sig.yT(k));
        set(R(p).hThr,   'XData', S.sig.xM(k),   'YData', S.sig.yM(k), ...
                         'UData', S.Fx(k)*sf,    'VData', S.Fy(k)*sf);
        set(R(p).hTxt,   'String', sprintf('R = %8.1f m   |T| = %6.0f N   \\delta = %+5.1f\\circ', ...
                                   S.range(k), S.sig.Tmag(k), rad2deg(S.TangRad(k))));
    end
    set(hSG,'String', sprintf('t = %6.3f s', tc(k)));
    drawnow limitrate; pause(FRAME_PAUSE);
end

% Final frame + closest-approach markers
for p = 1:2
    S = R(p).S; kEnd = N_GRID; kMin = S.minIdx;
    set(R(p).hMline,'XData',S.sig.xM,'YData',S.sig.yM);
    set(R(p).hTline,'XData',S.sig.xT,'YData',S.sig.yT);
    set(R(p).hMhead,'XData',S.sig.xM(kEnd),'YData',S.sig.yM(kEnd));
    set(R(p).hThead,'XData',S.sig.xT(kEnd),'YData',S.sig.yT(kEnd));
    plot(R(p).ax, S.sig.xM(kMin), S.sig.yM(kMin), 'kp', ...
         'MarkerSize',16,'MarkerFaceColor','y','DisplayName','Closest Approach');
    title(R(p).ax, sprintf('%s   -   miss = %.3f m @ t = %.3f s', ...
          S.label, S.minDist, tc(kMin)), 'FontWeight','bold');
end
set(hSG,'String','Engagement Complete');
drawnow;

%% ------------------------------ SIGNAL COMPARISON TABLE --------------------
fprintf('\n');
fprintf('================================================================\n');
fprintf('                    SIMULATION DATA COMPARISON                  \n');
fprintf('================================================================\n');

rowN = { 'Samples Xm'; 'Samples Ym'; 'Samples Xt'; 'Samples Yt'; ...
         'Start time (s)'; 'End time (s)'; ...
         'Xm  start -> end'; 'Ym  start -> end'; ...
         'Xt  start -> end'; 'Yt  start -> end'; ...
         'NaN present (Xm,Ym,Xt,Yt)'; 'Inf present (Xm,Ym,Xt,Yt)' };

Ideal   = rawColumn(A);
Limited = rawColumn(B);
disp(table(Ideal, Limited, 'RowNames', rowN));

%% ------------------------------ ACCURACY MATRIX ----------------------------
fprintf('================================================================\n');
fprintf('                        ACCURACY MATRIX                         \n');
fprintf('     (Delta = Limited - Ideal ;  %%Chg relative to Ideal)        \n');
fprintf('================================================================\n');

mI = metrics(Ar, tc, HIT_RADIUS);
mL = metrics(Br, tc, HIT_RADIUS);
err = errorStats(Ar, Br, tc);

Metric = strings(0,1); Ideal = strings(0,1);
Limited = strings(0,1); Delta = strings(0,1); PctChg = strings(0,1);

    function addRow(name, vi, vl, nd, invert)
        if nargin < 5, invert = false; end
        Metric(end+1,1)  = name;
        if isnan(vi)
            Ideal(end+1,1) = "--"; Delta(end+1,1) = "--"; PctChg(end+1,1) = "--";
        else
            Ideal(end+1,1) = sprintf('%.*f', nd, vi);
            d = vl - vi;
            Delta(end+1,1) = sprintf('%+.*f', nd, d);
            if abs(vi) > eps
                p = 100*d/abs(vi);
                if invert, p = -p; end
                PctChg(end+1,1) = sprintf('%+.2f %%', p);
            else
                PctChg(end+1,1) = "--";
            end
        end
        Limited(end+1,1) = sprintf('%.*f', nd, vl);
    end

addRow("Miss distance                (m)",   mI.miss,     mL.miss,     4);
addRow("Time of closest approach     (s)",   mI.tMin,     mL.tMin,     4);
addRow("Missile path length to CPA   (m)",   mI.pathLen,  mL.pathLen,  2);
addRow("Mean missile speed           (m/s)", mI.vAvg,     mL.vAvg,     2);
addRow("Terminal missile speed       (m/s)", mI.vEnd,     mL.vEnd,     2);
addRow("Terminal closing rate        (m/s)", mI.vClose,   mL.vClose,   2);
addRow("Total thrust impulse         (N.s)", mI.impulse,  mL.impulse,  2);
addRow("Peak thrust deflection       (deg)", mI.dMax,     mL.dMax,     3);
addRow("Mean |thrust deflection|     (deg)", mI.dMean,    mL.dMean,    3);
addRow("Peak lateral acceleration    (g)",   mI.gMax,     mL.gMax,     3);
addRow("Peak heading rate            (deg/s)", mI.hrMax,  mL.hrMax,    3);
addRow("Position RMSE vs ideal       (m)",   0,           err.rmsePos, 4);
addRow("Max position deviation       (m)",   0,           err.maxPos,  4);
addRow("Heading RMSE vs ideal        (deg)", 0,           err.rmseHdg, 4);
addRow("Terminal position offset     (m)",   0,           err.endPos,  4);

AccuracyMatrix = table(Ideal, Limited, Delta, PctChg, 'RowNames', cellstr(Metric));
disp(AccuracyMatrix);

fprintf('Intercept (< %.2f m) : IDEAL = %s   |   LIMITED = %s\n', ...
        HIT_RADIUS, yn(mI.miss < HIT_RADIUS), yn(mL.miss < HIT_RADIUS));
fprintf('Guidance degradation : %+.2f m of additional miss distance\n', mL.miss - mI.miss);
fprintf('================================================================\n\n');

%% ============================================================================
%                              LOCAL FUNCTIONS
%% ============================================================================
function out = runModel(mdl)
    if ~bdIsLoaded(mdl), load_system(mdl); end
    try
        out = sim(Simulink.SimulationInput(mdl));
    catch
        out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    end
end

% ---------------------------------------------------------------------------
function S = extractRun(out, label)
    S.label = label;
    names = { 'xM','Xm'; 'yM','Ym'; 'xT','Xt'; 'yT','Yt'; ...
              'vMx','Vmx'; 'vMy','Vmy'; 'vM','Vm'; ...
              'vTx','Vtx'; 'vTy','Vty'; 'vT','Vt'; ...
              'gT','gammaT'; 'gH','gammaH'; ...
              'Tmag','Thrust'; 'Tang','ThrustAngle' };
    S.t = [];
    for i = 1:size(names,1)
        [t, d] = getSig(out, names{i,2});
        S.sig.(names{i,1}) = d;
        if isempty(S.t) && ~isempty(t), S.t = t; end
    end
    if isempty(S.t)
        error('No time vector found in "%s" - log signals as timeseries.', label);
    end
    S.raw = S.sig;   % untouched copy for the raw-data table
    S.rawT = S.t;
end

% ---------------------------------------------------------------------------
function [t, d] = getSig(out, name)
    t = []; d = [];
    try
        s = out.(name);
    catch
        warning('Signal "%s" not found - skipped.', name); return;
    end
    if isa(s, 'timeseries')
        t = s.Time(:); d = s.Data(:);
    elseif isstruct(s) && isfield(s,'time') && isfield(s,'signals')
        t = s.time(:);  d = s.signals.values(:);
    else
        d = s(:);
    end
end

% ---------------------------------------------------------------------------
function R = resampleRun(S, tc)
    R = S; R.t = tc;
    [tu, ia] = unique(S.rawT);
    fn = fieldnames(S.sig);
    for i = 1:numel(fn)
        d = S.sig.(fn{i});
        if isempty(d) || numel(d) ~= numel(S.rawT)
            R.sig.(fn{i}) = [];
        else
            R.sig.(fn{i}) = interp1(tu, d(ia), tc, 'linear', 'extrap');
        end
    end
end

% ---------------------------------------------------------------------------
function S = deriveRun(S)
    n = numel(S.t);

    % --- thrust deflection, auto deg/rad detection -------------------------
    Tang = S.sig.Tang;
    if isempty(Tang), Tang = zeros(n,1); end
    if max(abs(Tang)) > 2*pi, S.TangRad = deg2rad(Tang); else, S.TangRad = Tang; end
    if isempty(S.sig.Tmag), S.sig.Tmag = zeros(n,1); end

    % --- missile heading: prefer velocity components ------------------------
    if ~isempty(S.sig.vMx) && ~isempty(S.sig.vMy)
        S.heading = atan2(S.sig.vMy, S.sig.vMx);
    elseif ~isempty(S.sig.gH)
        gH = S.sig.gH;
        if max(abs(gH)) > 2*pi, gH = deg2rad(gH); end
        S.heading = gH;
    else
        S.heading = atan2(gradient(S.sig.yM, S.t), gradient(S.sig.xM, S.t));
    end

    % --- thrust vector in the inertial frame --------------------------------
    ang  = S.heading + S.TangRad;
    S.Fx = S.sig.Tmag .* cos(ang);
    S.Fy = S.sig.Tmag .* sin(ang);

    % --- speed --------------------------------------------------------------
    if ~isempty(S.sig.vM)
        S.speed = S.sig.vM;
    elseif ~isempty(S.sig.vMx)
        S.speed = hypot(S.sig.vMx, S.sig.vMy);
    else
        S.speed = hypot(gradient(S.sig.xM,S.t), gradient(S.sig.yM,S.t));
    end

    % --- range to target & closest point of approach ------------------------
    S.range = hypot(S.sig.xM - S.sig.xT, S.sig.yM - S.sig.yT);
    [S.minDist, S.minIdx] = min(S.range);
end

% ---------------------------------------------------------------------------
function P = buildPanel(ax, S, LIM, sf, showLegend)
    P.ax = ax; P.S = S;
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    axis(ax, LIM); axis(ax,'equal'); axis(ax, LIM);
    xlabel(ax,'X Position (m)'); ylabel(ax,'Y Position (m)');
    title(ax, S.label, 'FontWeight','bold');

    P.hMline = plot(ax,NaN,NaN,'r-', 'LineWidth',2,'DisplayName','Missile Path');
    P.hTline = plot(ax,NaN,NaN,'b--','LineWidth',2,'DisplayName','Target Path');
    P.hMhead = plot(ax,S.sig.xM(1),S.sig.yM(1),'ro','MarkerFaceColor','r', ...
                    'MarkerSize',8,'DisplayName','Missile');
    P.hThead = plot(ax,S.sig.xT(1),S.sig.yT(1),'bs','MarkerFaceColor','b', ...
                    'MarkerSize',8,'DisplayName','Target');
    P.hThr   = quiver(ax,S.sig.xM(1),S.sig.yM(1),S.Fx(1)*sf,S.Fy(1)*sf,0, ...
                    'Color',[0.85 0.33 0.10],'LineWidth',2.2,'MaxHeadSize',2, ...
                    'DisplayName','Thrust Vector');
    P.hTxt = text(ax,0.02,0.96,'','Units','normalized','FontName','Consolas', ...
                    'FontSize',9,'VerticalAlignment','top', ...
                    'BackgroundColor',[1 1 1 ],'EdgeColor',[0.8 0.8 0.8]);
    if showLegend
        legend(ax,[P.hMline P.hTline P.hMhead P.hThead P.hThr], ...
               'Location','southoutside','Orientation','horizontal');
    end
end

% ---------------------------------------------------------------------------
function m = metrics(S, tc, ~)
    m.miss = S.minDist;
    m.tMin = tc(S.minIdx);

    k = S.minIdx;
    dseg = hypot(diff(S.sig.xM(1:k)), diff(S.sig.yM(1:k)));
    m.pathLen = sum(dseg);

    m.vAvg = mean(S.speed(1:k));
    m.vEnd = S.speed(k);

    dr = gradient(S.range, tc);
    m.vClose = -dr(k);

    m.impulse = trapz(tc, S.sig.Tmag);

    dd = rad2deg(S.TangRad);
    m.dMax  = max(abs(dd));
    m.dMean = mean(abs(dd));

    hdot   = gradient(unwrap(S.heading), tc);
    m.hrMax = max(abs(rad2deg(hdot)));
    m.gMax  = max(abs(S.speed .* hdot)) / 9.80665;
end

% ---------------------------------------------------------------------------
function e = errorStats(A, B, tc)
    dx = B.sig.xM - A.sig.xM;
    dy = B.sig.yM - A.sig.yM;
    d  = hypot(dx, dy);
    e.rmsePos = sqrt(mean(d.^2));
    e.maxPos  = max(d);
    e.endPos  = d(end);
    dh = rad2deg(wrapToPiLocal(B.heading - A.heading));
    e.rmseHdg = sqrt(mean(dh.^2));
    e.t = tc;
end

% ---------------------------------------------------------------------------
function w = wrapToPiLocal(a)
    w = mod(a + pi, 2*pi) - pi;
end

% ---------------------------------------------------------------------------
function c = rawColumn(S)
    r = S.raw; t = S.rawT;
    nanAny = any(isnan(r.xM)) || any(isnan(r.yM)) || any(isnan(r.xT)) || any(isnan(r.yT));
    infAny = any(isinf(r.xM)) || any(isinf(r.yM)) || any(isinf(r.xT)) || any(isinf(r.yT));
    c = [ string(numel(r.xM))
          string(numel(r.yM))
          string(numel(r.xT))
          string(numel(r.yT))
          sprintf('%g', t(1))
          sprintf('%g', t(end))
          sprintf('%g -> %g', r.xM(1), r.xM(end))
          sprintf('%g -> %g', r.yM(1), r.yM(end))
          sprintf('%g -> %g', r.xT(1), r.xT(end))
          sprintf('%g -> %g', r.yT(1), r.yT(end))
          yn(nanAny)
          yn(infAny) ];
end

% ---------------------------------------------------------------------------
function s = yn(b)
    if b, s = "YES"; else, s = "NO"; end
end
%% ============================================================================
%  MISSILE ENGAGEMENT COMPARISON : IDEAL  vs  LIMITED
%  ---------------------------------------------------------------------------
%  1. Runs both Simulink models via sim()
%  2. Side-by-side LIVE animation (trajectories + thrust vector arrows).
%     Each run uses its OWN time base and plays to its OWN end time; the
%     shorter run freezes on its final frame and is flagged COMPLETE.
%  3. Signal comparison table in the Command Window
%  4. Accuracy matrix (miss distance, time to CPA, path length, peak deflection)
%
%  Expects both models to log: Xm Ym Xt Yt Vmx Vmy Vm Vtx Vty Vt
%                              gammaT gammaM Thrust ThrustAngle
% ============================================================================
clear; clc; close all;

%% ------------------------------ USER CONFIG --------------------------------
% Give FULL PATHS. Bare names are resolved by the MATLAB path and can
% silently pick up a shadowed copy of the model from another folder.
MODEL_IDEAL   = 'anjaneya_V5.slx';
MODEL_LIMITED = 'anjaneya_V7.slx';

N_GRID        = 3000;    % samples per run on its own uniform grid
N_FRAMES      = 400;     % animation frames across the LONGER run
FRAME_PAUSE   = 0.010;   % seconds between frames (playback speed)
HIT_RADIUS    = 5.0;     % m - miss distance below this counts as an intercept

ARROW_MODE    = 'scaled';% 'scaled' = length proportional to thrust
                         % 'fixed'  = constant length, shows direction only
ARROW_FRAC    = 0.08;    % arrow length as fraction of plot span (at max thrust)
ARROW_BEHIND  = true;    % true  = arrowhead ON the missile, tail trailing back
                         % false = tail on the missile, head pointing forward

%% ------------------------------ RUN THE MODELS -----------------------------
fprintf('Running IDEAL   model : %s ...\n', MODEL_IDEAL);
outI = runModel(MODEL_IDEAL);

fprintf('Running LIMITED model : %s ...\n', MODEL_LIMITED);
outL = runModel(MODEL_LIMITED);
fprintf('Both runs complete.\n');

%% ------------------------------ EXTRACT ------------------------------------
A = extractRun(outI, 'IDEAL');
B = extractRun(outL, 'LIMITED');

% --- Each run keeps its OWN uniform time base, spanning its OWN duration ----
tA = linspace(A.t(1), A.t(end), N_GRID)';
tB = linspace(B.t(1), B.t(end), N_GRID)';
Ar = deriveRun(resampleRun(A, tA));
Br = deriveRun(resampleRun(B, tB));

% --- A separate COMMON grid, used only for point-to-point error stats -------
tc = linspace(max(A.t(1),B.t(1)), min(A.t(end),B.t(end)), N_GRID)';
Ac = deriveRun(resampleRun(A, tc));
Bc = deriveRun(resampleRun(B, tc));

%% ------------------------------ RUN DIFF CHECK -----------------------------
fprintf('\n--- RUN DIFF CHECK (max |Ideal - Limited| on common grid) ---\n');
chk = {'xM','yM','vM','Tmag','Tang'};
identical = true;
for i = 1:numel(chk)
    a = Ac.sig.(chk{i}); b = Bc.sig.(chk{i});
    if isempty(a) || isempty(b)
        fprintf('  %-6s : signal missing in one run\n', chk{i});
    else
        dmax = max(abs(a - b));
        fprintf('  %-6s : %.6g\n', chk{i}, dmax);
        if dmax > 1e-9, identical = false; end
    end
end
if identical
    fprintf('\n  >> WARNING: the two runs are NUMERICALLY IDENTICAL.\n\n');
end

fprintf('\n--- THRUST SIGNAL CHECK (drives the arrow) ---\n');
thrustReport(Ar); thrustReport(Br);

%% ------------------------------ ANIMATION ----------------------------------
% Shared axis limits so the two panels are visually comparable
allX = [Ar.sig.xM; Ar.sig.xT; Br.sig.xM; Br.sig.xT];
allY = [Ar.sig.yM; Ar.sig.yT; Br.sig.yM; Br.sig.yT];
xMin = min(allX); xMax = max(allX);
yMin = min(allY); yMax = max(allY);
span = max([xMax-xMin, yMax-yMin, 1]);
mrg  = 0.10*span;
LIM  = [xMin-mrg, xMax+mrg, yMin-mrg, yMax+mrg];

% Shared thrust scaling so arrow length means the same in both panels
maxT = max([max(abs(Ar.sig.Tmag)), max(abs(Br.sig.Tmag))]);
if maxT > 0, sf = ARROW_FRAC*span/maxT; else, sf = 1; end
arrowFixedLen = ARROW_FRAC*span;

% --- Arrow diagnostics: if the arrow is invisible, the answer is here ------
fprintf('\n--- ARROW SCALING ---\n');
fprintf('  Plot span            : %.1f m\n', span);
fprintf('  Max thrust (both runs): %.3f N\n', maxT);
fprintf('  Arrow length at max T : %.1f m  (%.1f%% of span)\n', ...
        maxT*sf, 100*maxT*sf/span);
if maxT <= 0
    warning(['Thrust is zero everywhere - forcing ARROW_MODE = ''fixed'' ' ...
             'so the arrow at least shows direction.']);
    ARROW_MODE = 'fixed';
end
fprintf('  Arrow mode           : %s\n\n', ARROW_MODE);

qcfg = struct('sf',sf,'fixedLen',arrowFixedLen,'mode',ARROW_MODE,'behind',ARROW_BEHIND);

fig = figure('Name','Ideal vs Limited - Engagement Animation', ...
             'Color','w','NumberTitle','off','Position',[80 80 1400 640]);
tl  = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');

R(1) = buildPanel(nexttile(tl), Ar, LIM, true,  qcfg);
R(2) = buildPanel(nexttile(tl), Br, LIM, false, qcfg);

hSG = sgtitle(tl, '', 'FontWeight','bold','FontSize',13);

% Global animation clock spans the LONGER of the two runs
tStart  = min(Ar.t(1),   Br.t(1));
tEndMax = max(Ar.t(end), Br.t(end));
tAnim   = linspace(tStart, tEndMax, N_FRAMES);

for f = 1:N_FRAMES
    for p = 1:2
        S = R(p).S;
        done = tAnim(f) >= S.t(end);
        k = timeToIndex(S, tAnim(f), N_GRID);   % clamps at this run's end

        set(R(p).hMline, 'XData', S.sig.xM(1:k), 'YData', S.sig.yM(1:k));
        set(R(p).hTline, 'XData', S.sig.xT(1:k), 'YData', S.sig.yT(1:k));
        set(R(p).hMhead, 'XData', S.sig.xM(k),   'YData', S.sig.yM(k));
        set(R(p).hThead, 'XData', S.sig.xT(k),   'YData', S.sig.yT(k));

        [ax0, ay0, u, v] = arrowGeom(S, k, sf, arrowFixedLen, ...
                                     ARROW_MODE, ARROW_BEHIND);
        set(R(p).hThr, 'XData',ax0, 'YData',ay0, 'UData',u, 'VData',v);

        if done, tag = '   [COMPLETE]'; else, tag = ''; end
        set(R(p).hTxt, 'String', sprintf( ...
            't = %6.3f s%s\nR = %8.1f m\n|T| = %8.1f N\n\\delta = %+6.2f\\circ', ...
            min(tAnim(f),S.t(end)), tag, S.range(k), ...
            S.sig.Tmag(k), rad2deg(S.TangRad(k))));
    end
    set(hSG,'String', sprintf('t = %6.3f s   /   %6.3f s', tAnim(f), tEndMax));
    drawnow limitrate; pause(FRAME_PAUSE);
end

% Final frame + closest-approach markers
for p = 1:2
    S = R(p).S; kMin = S.minIdx;
    set(R(p).hMline,'XData',S.sig.xM,'YData',S.sig.yM);
    set(R(p).hTline,'XData',S.sig.xT,'YData',S.sig.yT);
    set(R(p).hMhead,'XData',S.sig.xM(end),'YData',S.sig.yM(end));
    set(R(p).hThead,'XData',S.sig.xT(end),'YData',S.sig.yT(end));
    plot(R(p).ax, S.sig.xM(kMin), S.sig.yM(kMin), 'kp', ...
         'MarkerSize',16,'MarkerFaceColor','y','DisplayName','Closest Approach');
    title(R(p).ax, sprintf('%s   -   miss = %.3f m @ t = %.3f s  (T_{end} = %.4f s)', ...
          S.label, S.minDist, S.t(kMin), S.t(end)), 'FontWeight','bold');
end
set(hSG,'String','Engagement Complete');
drawnow;

%% ------------------------------ SIGNAL COMPARISON TABLE --------------------
fprintf('\n================================================================\n');
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

mI  = metrics(Ar);           % per-run metrics on each run's own time base
mL  = metrics(Br);

% {label, ideal value, limited value, decimals}
rows = { ...
  "Miss distance                (m)",     mI.miss,    mL.miss,    4; ...
  "Time of closest approach     (s)",     mI.tMin,    mL.tMin,    4; ...
  "Missile path length to CPA   (m)",     mI.pathLen, mL.pathLen, 2; ...
  "Peak thrust deflection       (deg)",   mI.dMax,    mL.dMax,    3  };

nR = size(rows,1);
Metric  = strings(nR,1); Ideal  = strings(nR,1);
Limited = strings(nR,1); Delta  = strings(nR,1); PctChg = strings(nR,1);

for i = 1:nR
    name = rows{i,1}; vi = rows{i,2}; vl = rows{i,3}; nd = rows{i,4};
    Metric(i)  = name;
    Ideal(i)   = sprintf('%.*f', nd, vi);
    Limited(i) = sprintf('%.*f', nd, vl);
    d = vl - vi;
    Delta(i)   = sprintf('%+.*f', nd, d);
    if abs(vi) > eps
        PctChg(i) = sprintf('%+.3f %%', 100*d/abs(vi));
    else
        PctChg(i) = "--";
    end
end

AccuracyMatrix = table(Ideal, Limited, Delta, PctChg, 'RowNames', cellstr(Metric));
disp(AccuracyMatrix);

fprintf('Intercept (< %.2f m) : IDEAL = %s   |   LIMITED = %s\n', ...
        HIT_RADIUS, yn(mI.miss < HIT_RADIUS), yn(mL.miss < HIT_RADIUS));
fprintf('Guidance degradation : %+.4f m of additional miss distance\n', mL.miss - mI.miss);
fprintf('================================================================\n\n');

%% ============================================================================
%                              LOCAL FUNCTIONS
%% ============================================================================
function out = runModel(mdlPath)
    [pth, name, ext] = fileparts(mdlPath);
    if isempty(ext), ext = '.slx'; end

    if ~isempty(pth)
        f = fullfile(pth, [name ext]);
        if ~isfile(f), error('Model file not found: %s', f); end
        if bdIsLoaded(name)
            if ~strcmp(get_param(name,'FileName'), f)
                fprintf('   evicting shadowed copy: %s\n', get_param(name,'FileName'));
                close_system(name, 0);
            end
        end
        if ~bdIsLoaded(name), load_system(f); end
    else
        if ~bdIsLoaded(name), load_system(name); end
    end

    fprintf('   file actually loaded: %s\n', get_param(name,'FileName'));
    try
        out = sim(Simulink.SimulationInput(name));
    catch
        out = sim(name, 'ReturnWorkspaceOutputs', 'on');
    end
end

% ---------------------------------------------------------------------------
function S = extractRun(out, label)
    S.label = label;
    names = { 'xM','Xm'; 'yM','Ym'; 'xT','Xt'; 'yT','Yt'; ...
              'vMx','Vmx'; 'vMy','Vmy'; 'vM','Vm'; ...
              'vTx','Vtx'; 'vTy','Vty'; 'vT','Vt'; ...
              'gT','gammaT'; 'gM','gammaM'; ...
              'Tmag','Thrust'; 'Tang','ThrustAngle' };
    S.t = [];
    for i = 1:size(names,1)
        [t, d] = getSig(out, names{i,2});
        S.sig.(names{i,1})  = d;
        S.tsig.(names{i,1}) = t;      % each signal keeps its OWN time base
        if isempty(S.t) && ~isempty(t), S.t = t; end
    end
    if isempty(S.t)
        error('No time vector found in "%s" - log signals as timeseries.', label);
    end
    S.raw = S.sig; S.rawT = S.t;
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
function R = resampleRun(S, tq)
    R = S; R.t = tq;
    fn = fieldnames(S.sig);
    for i = 1:numel(fn)
        d  = S.sig.(fn{i});
        ts = S.tsig.(fn{i});
        if isempty(ts), ts = S.rawT; end

        if isempty(d)
            R.sig.(fn{i}) = [];
        elseif numel(d) == 1
            % constant signal logged as a scalar - hold it across the grid
            R.sig.(fn{i}) = repmat(d, numel(tq), 1);
        elseif numel(d) ~= numel(ts)
            warning(['Signal "%s" in run %s: %d samples vs %d timestamps - ' ...
                     'IGNORED. Log it as a scalar timeseries.'], ...
                     fn{i}, S.label, numel(d), numel(ts));
            R.sig.(fn{i}) = [];
        else
            [tu, ia] = unique(ts);
            R.sig.(fn{i}) = interp1(tu, d(ia), tq, 'linear', 'extrap');
        end
    end
end

% ---------------------------------------------------------------------------
function S = deriveRun(S)
    n = numel(S.t);

    % --- thrust magnitude ---------------------------------------------------
    if isempty(S.sig.Tmag)
        warning(['Thrust magnitude MISSING in run %s - arrows will have ' ...
                 'zero length.'], S.label);
        S.sig.Tmag = zeros(n,1);
    end

    % --- thrust deflection, auto deg/rad detection --------------------------
    Tang = S.sig.Tang;
    if isempty(Tang)
        warning('ThrustAngle MISSING in run %s - assuming zero deflection.', S.label);
        Tang = zeros(n,1);
    end
    if max(abs(Tang)) > 2*pi, S.TangRad = deg2rad(Tang); else, S.TangRad = Tang; end

    % --- missile heading: prefer velocity components ------------------------
    if ~isempty(S.sig.vMx) && ~isempty(S.sig.vMy)
        S.heading = atan2(S.sig.vMy, S.sig.vMx);
    elseif ~isempty(S.sig.gM)
        gM = S.sig.gM;
        if max(abs(gM)) > 2*pi, gM = deg2rad(gM); end
        S.heading = gM;
    else
        S.heading = atan2(gradient(S.sig.yM, S.t), gradient(S.sig.xM, S.t));
    end

    % --- thrust vector in the inertial frame --------------------------------
    ang  = S.heading + S.TangRad;
    S.Fx = S.sig.Tmag .* cos(ang);
    S.Fy = S.sig.Tmag .* sin(ang);
    S.dirX = cos(ang);  S.dirY = sin(ang);

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
function thrustReport(S)
    T = S.sig.Tmag;
    fprintf(['  %-8s : Thrust min = %10.3f N   max = %10.3f N   ' ...
             'zero for %5.1f%% of flight\n'], S.label, min(T), max(T), ...
             100*sum(abs(T) < 1e-6)/numel(T));
end

% ---------------------------------------------------------------------------
function k = timeToIndex(S, tNow, N)
    tq = min(max(tNow, S.t(1)), S.t(end));
    frac = (tq - S.t(1)) / max(S.t(end) - S.t(1), eps);
    k = min(N, max(1, round(1 + frac*(N-1))));
end

% ---------------------------------------------------------------------------
function [x0, y0, u, v] = arrowGeom(S, k, sf, fixedLen, mode, behind)
    if strcmpi(mode, 'fixed')
        u = S.dirX(k)*fixedLen;   v = S.dirY(k)*fixedLen;
    else
        u = S.Fx(k)*sf;           v = S.Fy(k)*sf;
    end
    if behind
        % arrowhead lands ON the missile, tail trails behind it
        x0 = S.sig.xM(k) - u;     y0 = S.sig.yM(k) - v;
    else
        x0 = S.sig.xM(k);         y0 = S.sig.yM(k);
    end
end

% ---------------------------------------------------------------------------
function P = buildPanel(ax, S, LIM, showLegend, q)
    P.ax = ax; P.S = S;
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    axis(ax, LIM); axis(ax,'equal'); axis(ax, LIM);
    xlabel(ax,'X Position (m)'); ylabel(ax,'Y Position (m)');
    title(ax, S.label, 'FontWeight','bold');

    P.hMline = plot(ax,NaN,NaN,'r-', 'LineWidth',2,'DisplayName','Missile Path');
    P.hTline = plot(ax,NaN,NaN,'b--','LineWidth',2,'DisplayName','Target Path');

    % Initialise the quiver with REAL data (a quiver created from NaN/zero
    % data does not reliably initialise and can stay invisible afterwards).
    [x0,y0,u0,v0] = arrowGeom(S, 1, q.sf, q.fixedLen, q.mode, q.behind);
    P.hThr = quiver(ax, x0, y0, u0, v0, 0, ...
                    'Color',[0.85 0.33 0.10],'LineWidth',2.5, ...
                    'MaxHeadSize',1.5,'AutoScale','off', ...
                    'DisplayName','Thrust Vector');

    P.hMhead = plot(ax,S.sig.xM(1),S.sig.yM(1),'ro','MarkerFaceColor','r', ...
                    'MarkerSize',8,'DisplayName','Missile');
    P.hThead = plot(ax,S.sig.xT(1),S.sig.yT(1),'bs','MarkerFaceColor','b', ...
                    'MarkerSize',8,'DisplayName','Target');
    P.hTxt = text(ax,0.02,0.97,'','Units','normalized','FontName','Consolas', ...
                    'FontSize',9,'VerticalAlignment','top', ...
                    'BackgroundColor','w','EdgeColor',[0.8 0.8 0.8]);
    if showLegend
        legend(ax,[P.hMline P.hTline P.hMhead P.hThead P.hThr], ...
               'Location','southoutside','Orientation','horizontal');
    end
end

% ---------------------------------------------------------------------------
function m = metrics(S)
    m.miss = S.minDist;
    m.tMin = S.t(S.minIdx);

    k = S.minIdx;
    m.pathLen = sum(hypot(diff(S.sig.xM(1:k)), diff(S.sig.yM(1:k))));

    m.dMax = max(abs(rad2deg(S.TangRad)));
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
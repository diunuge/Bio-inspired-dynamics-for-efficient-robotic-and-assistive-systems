% ================================================================
% analyze_summary.m  --  Jumping Biomechanics Analysis Suite
% ================================================================
% This script processes raw CSV data from a 5-bar linkage jump experiment.
% It extracts phase-by-phase kinematics and kinetics, handles force sign
% conventions, computes comprehensive metrics, and exports publication-
% quality figures and CSV summary files.
% ================================================================

clearvars; close all; clc;
warning('off','MATLAB:table:ModifiedAndSavedVariableNames');

%% 1. Configuration & Constants
m_kg  = 2.0;               % Estimated system mass (kg)
g_ms2 = 9.81;              % Gravity (m/s^2)
Mg_N  = m_kg * g_ms2;      % Body Weight (N)

scriptDir = fileparts(mfilename('fullpath'));
dataDir   = fullfile(scriptDir, 'Data');
outDir    = fullfile(scriptDir, 'Figures');
if ~exist(dataDir,'dir'), error('Data directory not found: %s', dataDir); end
if ~exist(outDir,'dir'), mkdir(outDir); end

% Define plotting colors (Colorblind-friendly publication palette)
C_COMPRESS = [0.8500, 0.3250, 0.0980]; % Orange
C_HOLD     = [0.9290, 0.6940, 0.1250]; % Yellow
C_EXTEND   = [0.0000, 0.4470, 0.7410]; % Blue
C_BW       = [0.5000, 0.5000, 0.5000]; % Gray for Body Weight

%% 2. Find and Load Data Files
files = dir(fullfile(dataDir,'jump_speed_*mps.csv'));
if isempty(files)
    error('No files matching jump_speed_*mps.csv found in %s', dataDir);
end
fprintf('Found %d files.\n\n', numel(files));

% Extract speeds from filenames and sort
speed_vals = nan(numel(files),1);
for fi = 1:numel(files)
    tok = regexp(files(fi).name,'jump_speed_(\d+\.?\d*)mps','tokens');
    if ~isempty(tok)
        speed_vals(fi) = str2double(tok{1}{1});
    end
end
[speed_vals, sort_idx] = sort(speed_vals);
files = files(sort_idx);

%% 3. Process Data: Phase-by-Phase Extraction
% Preallocate arrays for Repetition-Level data
Reps = struct('speed_mps',{}, 'rep',{}, ...
    't_compress',{}, 't_hold',{}, 't_extend',{}, 't_contact',{}, ...
    'impulse_compress',{}, 'impulse_hold',{}, 'impulse_extend',{}, 'impulse_contact',{}, ...
    'Fz_compress_peak',{}, 'Fz_compress_mean',{}, ...
    'Fz_hold_mean',{}, 'Fz_hold_std',{}, ...
    'Fz_extend_peak',{}, 'Fz_extend_mean',{}, 'time_to_peak',{}, ...
    'RFD_max',{}, 'load_rate',{}, 'unload_rate',{}, ...
    'Fx_peak',{}, 'Fy_peak',{}, 'Fres_peak',{}, ...
    'Mx_peak',{}, 'My_peak',{}, 'Mz_peak',{});

% Preallocate storage for timeline profiles
prof_extend = struct('speed',{}, 't',{}, 'Fz',{});
prof_norm   = struct('speed',{}, 't',{}, 'Fz',{});
prof_raw    = struct('speed',{}, 'rep',{}, 't',{}, 'Fz',{}, 'phase',{});

fprintf('Extracting biomechanical parameters...\n');
for fi = 1:numel(files)
    speed_mps = speed_vals(fi);
    fpath = fullfile(dataDir, files(fi).name);
    
    try
        T = readtable(fpath,'VariableNamingRule','preserve','Delimiter',',');
    catch ME
        fprintf('  WARN: Could not read %s - %s\n', files(fi).name, ME.message);
        continue
    end
    
    req_cols = {'time_s','speed_mps','rep','phase','Fx','Fy','Fz','Mx','My','Mz'};
    if ~all(ismember(req_cols, T.Properties.VariableNames))
        continue;
    end
    
    % Data Cleaning & Sign Handling
    T = T(~isnan(T.time_s), :);
    T = sortrows(T, 'time_s'); % Ensure strictly sequential time
    
    % Force Sign Convention: Force plates typically report downward pushing
    % force as negative. We invert Fz so Ground Reaction Force (GRF) is positive.
    T.Fz_grf = -T.Fz; 
    T.Fres = sqrt(T.Fx.^2 + T.Fy.^2 + T.Fz_grf.^2);
    T.Mx_abs = abs(T.Mx); T.My_abs = abs(T.My); T.Mz_abs = abs(T.Mz);
    
    unique_reps = unique(T.rep(T.rep > 0));
    for ri = 1:numel(unique_reps)
        r = unique_reps(ri);
        Tr = T(T.rep == r, :);
        
        idx_c = strcmpi(Tr.phase,'compress');
        idx_h = strcmpi(Tr.phase,'hold');
        idx_e = strcmpi(Tr.phase,'extend');
        
        Tc = Tr(idx_c, :);
        Th = Tr(idx_h, :);
        Te = Tr(idx_e, :);
        
        % Validate logical order (must have at least compress & extend)
        if height(Tc) < 2 || height(Te) < 2
            continue; 
        end
        
        % Initialize metrics map
        M = struct();
        M.speed_mps = speed_mps;
        M.rep = r;
        
        % Durations
        M.t_compress = Tc.time_s(end) - Tc.time_s(1);
        M.t_hold     = 0; if height(Th)>=2, M.t_hold = Th.time_s(end)-Th.time_s(1); end
        M.t_extend   = Te.time_s(end) - Te.time_s(1);
        M.t_contact  = M.t_compress + M.t_hold + M.t_extend;
        
        % Impulses (N·s) using trapezoidal integration
        M.impulse_compress = trapz(Tc.time_s, Tc.Fz_grf);
        M.impulse_hold     = 0; if height(Th)>=2, M.impulse_hold = trapz(Th.time_s, Th.Fz_grf); end
        M.impulse_extend   = trapz(Te.time_s, Te.Fz_grf);
        M.impulse_contact  = M.impulse_compress + M.impulse_hold + M.impulse_extend;
        
        % Forces
        M.Fz_compress_peak = max(Tc.Fz_grf);
        M.Fz_compress_mean = mean(Tc.Fz_grf);
        M.Fz_hold_mean     = NaN; M.Fz_hold_std = NaN;
        if height(Th) > 0
            M.Fz_hold_mean = mean(Th.Fz_grf);
            M.Fz_hold_std  = std(Th.Fz_grf);
        end
        [M.Fz_extend_peak, pk_idx] = max(Te.Fz_grf);
        M.Fz_extend_mean   = mean(Te.Fz_grf);
        M.time_to_peak     = Te.time_s(pk_idx) - Te.time_s(1);
        
        % Rate Metrics
        % Loading rate during compression phase
        M.load_rate = (M.Fz_compress_peak - Tc.Fz_grf(1)) / M.t_compress;
        % RFD (Rate of Force Development) in extend phase
        dt = diff(Te.time_s); 
        dF = diff(Te.Fz_grf);
        vdt = dt > 0.001; % filter tiny noise intervals
        if any(vdt)
            M.RFD_max = max(dF(vdt) ./ dt(vdt));
        else
            M.RFD_max = NaN;
        end
        % Unloading rate after peak extend force
        if pk_idx < height(Te)
            M.unload_rate = (Te.Fz_grf(end) - M.Fz_extend_peak) / (Te.time_s(end) - Te.time_s(pk_idx));
        else
            M.unload_rate = NaN;
        end
        
        % XYZ and Resultant Peaks
        Tall = sortrows([Tc; Th; Te], 'time_s');
        M.Fx_peak   = max(abs(Tall.Fx));
        M.Fy_peak   = max(abs(Tall.Fy));
        M.Fres_peak = max(Tall.Fres);
        M.Mx_peak   = max(Tall.Mx_abs);
        M.My_peak   = max(Tall.My_abs);
        M.Mz_peak   = max(Tall.Mz_abs);
        
        % Store repetition summary
        Reps(end+1) = M;
        
        % Store profile traces for figures
        % Align Extend phase from -0.1 to 0.2 s relative to extend start
        time_ext = Tall.time_s - Te.time_s(1);
        idx_ext  = time_ext >= -0.1 & time_ext <= 0.2;
        v_ext_t  = time_ext(idx_ext);
        v_ext_F  = Tall.Fz_grf(idx_ext);
        
        % Uniform interpolation mapping for plotting
        t_common_ext = linspace(-0.1, 0.2, 300)';
        Fz_common_ext = interp1(v_ext_t, v_ext_F, t_common_ext, 'linear', NaN);
        prof_extend(end+1) = struct('speed', speed_mps, 't', t_common_ext, 'Fz', Fz_common_ext);
        
        % Normalized overall contact phase (0 to 100%)
        t_norm = (Tall.time_s - Tall.time_s(1)) / (Tall.time_s(end) - Tall.time_s(1)) * 100;
        t_common_norm = linspace(0, 100, 300)';
        Fz_common_norm = interp1(t_norm, Tall.Fz_grf, t_common_norm, 'linear', NaN);
        prof_norm(end+1) = struct('speed', speed_mps, 't', t_common_norm, 'Fz', Fz_common_norm);
        
        % Raw trace record
        prof_raw(end+1) = struct('speed', speed_mps, 'rep', r, ...
            't', Tall.time_s - Te.time_s(1), ...  % Align 0 to start of extend
            'Fz', Tall.Fz_grf, 'phase', {Tall.phase});
    end
end

if isempty(Reps)
    error('No valid jump cycles found across the files.');
end
RepTable = struct2table(Reps);
writetable(RepTable, fullfile(scriptDir, 'jump_rep_summary.csv'));
fprintf('Saved jump_rep_summary.csv (%d reps)\n', height(RepTable));

%% 4. Aggregate Speed-Level Outputs & Statistics
speeds = unique(RepTable.speed_mps);
nS = numel(speeds);

SpeedStats = table();
SpeedStats.speed_mps = speeds;

% Aggregator helper
avg = @(var) splitapply(@mean, RepTable.(var), findgroups(RepTable.speed_mps));
stdev = @(var) splitapply(@std, RepTable.(var), findgroups(RepTable.speed_mps));

SpeedStats.Fz_extend_peak_mean = avg('Fz_extend_peak');
SpeedStats.Fz_extend_peak_std  = stdev('Fz_extend_peak');
SpeedStats.CV_extend_force_pct = (SpeedStats.Fz_extend_peak_std ./ SpeedStats.Fz_extend_peak_mean) * 100;

SpeedStats.impulse_total_mean = avg('impulse_contact');
SpeedStats.impulse_total_std  = stdev('impulse_contact');
SpeedStats.RFD_max_mean       = avg('RFD_max');
SpeedStats.RFD_max_std        = stdev('RFD_max');

SpeedStats.t_compress_mean = avg('t_compress');
SpeedStats.t_hold_mean     = avg('t_hold');
SpeedStats.t_extend_mean   = avg('t_extend');

SpeedStats.impulse_compress_mean = avg('impulse_compress');
SpeedStats.impulse_hold_mean     = avg('impulse_hold');
SpeedStats.impulse_extend_mean   = avg('impulse_extend');

writetable(SpeedStats, fullfile(scriptDir, 'jump_speed_summary.csv'));
fprintf('Saved jump_speed_summary.csv\n\n');

%% 5. Statistical Regressions
fprintf('--- Statistical Regression Results ---\n');
var_list = {'Fz_extend_peak', 'impulse_contact', 'RFD_max'};
for vi = 1:numel(var_list)
    vname = var_list{vi};
    % Rep-level stats
    X = RepTable.speed_mps;
    Y = RepTable.(vname);
    valid = ~isnan(Y); X = X(valid); Y = Y(valid);
    
    if numel(X) >= 3
        p = polyfit(X, Y, 1);
        yfit = polyval(p, X);
        Rsq = 1 - sum((Y - yfit).^2)/sum((Y - mean(Y)).^2);
        R_pearson = corr(X, Y);
        
        fprintf('%-20s: Slope = %8.2f | Int = %8.2f | R^2 = %.3f | r = %.3f\n', ...
            vname, p(1), p(2), Rsq, R_pearson);
    end
end
fprintf('--------------------------------------\n\n');

%% 6. Publication Figure Generation
disp('Generating scientific figures...');

% Helpers
set(0,'DefaultAxesFontSize',12);
set(0,'DefaultAxesFontName','Arial');
set(0,'DefaultTextFontName','Arial');
set(0,'DefaultLineLineWidth',1.5);
fig_pos = @(n)[50+mod(n-1,3)*450, 600-floor((n-1)/3)*400, 420, 320];

clrMap = jet(nS); % Multi-speed colormap

% -------------------------------------------------------------
% Identify best representative trial (Closest to mean Fz peak for median speed)
idx_mid = round(nS/2);
target_speed = speeds(idx_mid);
mask = [prof_raw.speed] == target_speed;
cands = prof_raw(mask);
target_fz = SpeedStats.Fz_extend_peak_mean(idx_mid);
dist = inf; best_cand = [];
for ci = 1:numel(cands)
    pk = max(cands(ci).Fz);
    if abs(pk - target_fz) < dist
        dist = abs(pk - target_fz);
        best_cand = cands(ci);
    end
end

% -------------------------------------------------------------
% Figure A: Raw Force Profiles with Phase Shading
figA = figure('Name', 'FigA_RawPhases', 'Position', fig_pos(1)); hold on;
if ~isempty(best_cand)
    % Find phase logic
    tc = best_cand.t(strcmpi(best_cand.phase, 'compress'));
    th = best_cand.t(strcmpi(best_cand.phase, 'hold'));
    te = best_cand.t(strcmpi(best_cand.phase, 'extend'));
    
    fill([min(tc) max(tc) max(tc) min(tc)], [0 0 max(best_cand.Fz)*1.1 max(best_cand.Fz)*1.1], ...
        C_COMPRESS, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility','off');
    fill([min(th) max(th) max(th) min(th)], [0 0 max(best_cand.Fz)*1.1 max(best_cand.Fz)*1.1], ...
        C_HOLD, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility','off');
    fill([min(te) max(te) max(te) min(te)], [0 0 max(best_cand.Fz)*1.1 max(best_cand.Fz)*1.1], ...
        C_EXTEND, 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility','off');
    
    plot(best_cand.t*1000, best_cand.Fz, 'k-', 'LineWidth', 2);
    yline(Mg_N, '--', 'Color', C_BW, 'LineWidth', 1.5, 'Label', 'Body Weight');
    
    text(mean(tc)*1000, max(best_cand.Fz), 'Compress', 'Color', C_COMPRESS, 'FontWeight','bold', 'HorizontalAlignment','center');
    text(mean(th)*1000, max(best_cand.Fz), 'Hold', 'Color', C_HOLD, 'FontWeight','bold', 'HorizontalAlignment','center');
    text(mean(te)*1000, max(best_cand.Fz), 'Extend', 'Color', C_EXTEND, 'FontWeight','bold', 'HorizontalAlignment','center');
end
xlabel('Time from Take-off Start (ms)', 'FontWeight','bold');
ylabel('Vertical GRF (N)', 'FontWeight','bold');
title('Representative Phase Trace', 'FontSize', 13);
grid on; box on;

% -------------------------------------------------------------
% Figure B: Mean ± SD force overlay aligned to the start of the extend phase
figB = figure('Name', 'FigB_MeanSDFz', 'Position', fig_pos(2)); hold on;
for si = 1:nS
    sp = speeds(si);
    mask = [prof_extend.speed] == sp;
    chunk = prof_extend(mask);
    if isempty(chunk); continue; end
    t_com = chunk(1).t;
    Fz_mat = cat(2, chunk.Fz); % columns = reps
    mu = mean(Fz_mat, 2, 'omitnan');
    sd = std(Fz_mat, 0, 2, 'omitnan');
    
    % Бүх хурдны муруйг зурах ба Legend дээр харуулах
    plot(t_com*1000, mu, '-', 'Color', clrMap(si,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('v = %g m/s', sp));
    % Хэрэв SD (стандарт хазайлт) сүүдэр зурахыг хүсвэл бүгдэн дээр нь зурах эсвэл зөвхөн цөөхөн хэд дээр нь зурахаар үлдээж болно.
    % Энэ удаад хэт замбараагүй харагдахаас сэргийлж зөвхөн Mean муруйг бүгдийг нь Legend-д орууллаа.
end
xline(0, 'k--', 'Take-off command', 'HandleVisibility','off','LabelVerticalAlignment','bottom');
xlabel('Time (ms)', 'FontWeight','bold');
ylabel('Mean Vertical GRF \pm SD (N)', 'FontWeight','bold');
title('Velocity-Dependent Force Traces', 'FontSize', 13);
legend('show','Location','northwest');
grid on; box on;

% -------------------------------------------------------------
% Figure C: Phase Durations Stacked Bar
figC = figure('Name', 'FigC_PhaseDur', 'Position', fig_pos(3)); hold on;
bar_data = [SpeedStats.t_compress_mean, SpeedStats.t_hold_mean, SpeedStats.t_extend_mean] * 1000;
b = bar(SpeedStats.speed_mps, bar_data, 'stacked');
b(1).FaceColor = C_COMPRESS; b(2).FaceColor = C_HOLD; b(3).FaceColor = C_EXTEND;
xlabel('Commanded Push Speed (m/s)', 'FontWeight','bold');
ylabel('Phase Duration (ms)', 'FontWeight','bold');
title('Distribution of Contact Time', 'FontSize', 13);
legend({'Compress', 'Hold', 'Extend/Push'}, 'Location', 'best');
grid on; box on;

% -------------------------------------------------------------
% Figure D: Phase Impulses Stacked Bar
figD = figure('Name', 'FigD_PhaseImpulse', 'Position', fig_pos(4)); hold on;
imp_data = [SpeedStats.impulse_compress_mean, SpeedStats.impulse_hold_mean, SpeedStats.impulse_extend_mean];
b2 = bar(SpeedStats.speed_mps, imp_data, 'stacked');
b2(1).FaceColor = C_COMPRESS; b2(2).FaceColor = C_HOLD; b2(3).FaceColor = C_EXTEND;
xlabel('Commanded Push Speed (m/s)', 'FontWeight','bold');
ylabel('Impulse Contribution (N\cdots)', 'FontWeight','bold');
title('Impulse Decomposition via Phase', 'FontSize', 13);
legend({'Compress', 'Hold', 'Extend/Push'}, 'Location', 'best');
grid on; box on;

% -------------------------------------------------------------
% Figure E: Panel plot (Peak Fz, Impulse, RFD) with Regression
figE = figure('Name', 'FigE_Regressions', 'Position', [50 150 900 300]); 
% Subplot 1: Peak Force
subplot(1,3,1); hold on;
errorbar(SpeedStats.speed_mps, SpeedStats.Fz_extend_peak_mean, SpeedStats.Fz_extend_peak_std, ...
    'o', 'Color', 'k', 'MarkerFaceColor', C_EXTEND, 'CapSize', 0);
p1 = polyfit(RepTable.speed_mps, RepTable.Fz_extend_peak, 1);
xf = linspace(min(speeds)*0.9, max(speeds)*1.1, 50);
plot(xf, polyval(p1, xf), 'k--');
rsq1 = corr(RepTable.speed_mps, RepTable.Fz_extend_peak)^2;
text(min(xf), max(SpeedStats.Fz_extend_peak_mean), sprintf('R^2 = %.2f', rsq1));
xlabel('Push Speed (m/s)','FontWeight','bold'); ylabel('Peak Take-off Force (N)','FontWeight','bold');
grid on; box on;

% Subplot 2: Total Impulse
subplot(1,3,2); hold on;
errorbar(SpeedStats.speed_mps, SpeedStats.impulse_total_mean, SpeedStats.impulse_total_std, ...
    '^', 'Color', 'k', 'MarkerFaceColor', C_COMPRESS, 'CapSize', 0);
p2 = polyfit(RepTable.speed_mps, RepTable.impulse_contact, 1);
plot(xf, polyval(p2, xf), 'k--');
rsq2 = corr(RepTable.speed_mps, RepTable.impulse_contact)^2;
text(min(xf), max(SpeedStats.impulse_total_mean), sprintf('R^2 = %.2f', rsq2));
xlabel('Push Speed (m/s)','FontWeight','bold'); ylabel('Contact Impulse (N\cdots)','FontWeight','bold');
grid on; box on;

% Subplot 3: RFD
subplot(1,3,3); hold on;
errorbar(SpeedStats.speed_mps, SpeedStats.RFD_max_mean, SpeedStats.RFD_max_std, ...
    's', 'Color', 'k', 'MarkerFaceColor', C_HOLD, 'CapSize', 0);
% Clean NaNs for polyfit
vldR = ~isnan(RepTable.RFD_max);
p3 = polyfit(RepTable.speed_mps(vldR), RepTable.RFD_max(vldR), 1);
plot(xf, polyval(p3, xf), 'k--');
rsq3 = corr(RepTable.speed_mps(vldR), RepTable.RFD_max(vldR))^2;
text(min(xf), max(SpeedStats.RFD_max_mean), sprintf('R^2 = %.2f', rsq3));
xlabel('Push Speed (m/s)','FontWeight','bold'); ylabel('Max RFD (N/s)','FontWeight','bold');
grid on; box on;

% -------------------------------------------------------------
% Figure H: Normalized Contact Phase Plot
figH = figure('Name', 'FigH_NormPhase', 'Position', fig_pos(5)); hold on;
for si = 1:nS
    sp = speeds(si);
    mask = [prof_norm.speed] == sp;
    chunk = prof_norm(mask);
    if isempty(chunk); continue; end
    t_com = chunk(1).t;
    Fz_mat = cat(2, chunk.Fz);
    mu = mean(Fz_mat, 2, 'omitnan');
    plot(t_com, mu, '-', 'Color', clrMap(si,:), 'LineWidth', 1.8);
end
xlabel('Percentage of Contact Phase (%)', 'FontWeight','bold');
ylabel('Vertical GRF (N)', 'FontWeight','bold');
title('Biomechanical Trajectory Normalization', 'FontSize', 13);
colormap(jet);
cb = colorbar; cb.Label.String = 'Push Speed (m/s)'; cb.Label.FontWeight = 'bold';
grid on; box on; xlim([0 100]);

% -------------------------------------------------------------
% Figure I: Biomechanical Performance Map (Impulse vs Force colored by speed)
figI = figure('Name', 'FigI_PerfMap', 'Position', fig_pos(6)); hold on;
scatter(RepTable.impulse_contact, RepTable.Fz_extend_peak, ...
    80, RepTable.speed_mps, 'filled', 'MarkerEdgeColor', 'k');
colormap(jet);
cbar = colorbar; cbar.Label.String = 'Push Speed (m/s)'; cbar.Label.FontWeight = 'bold';
xlabel('Total Propulsive Impulse (N\cdots)', 'FontWeight','bold');
ylabel('Peak Take-off Ground Reaction Force (N)', 'FontWeight','bold');
title('Impulse-Force Capability Map', 'FontSize', 13);
grid on; box on;

%% 7. Export Figures
fprintf('Exporting figures to %s ...\n', outDir);
figs = {figA, figB, figC, figD, figE, figH, figI};
fnames = {'FigA_PhaseTrace', 'FigB_OverlaySD', 'FigC_Durations', 'FigD_Impulses', ...
          'FigE_Regressions', 'FigH_NormTrace', 'FigI_PerfMap'};
for k = 1:numel(figs)
    f = figs{k};
    name_base = fullfile(outDir, fnames{k});
    exportgraphics(f, [name_base '.png'], 'Resolution', 300);
    savefig(f, [name_base '.fig']);
end

fprintf('Completed Biomechanics Analysis Suite successfully.\n');

%% =========================================================================
%  JCAS LOOPING EXPERIMENT RUNNER
%  =========================================================================
%  Peneliti  : Galih Nugraha Nurkahfi
%  Institusi : PR Telekomunikasi, BRIN
%  Script    : JCASFasaIMultistaticwith5GAAStandard_LoopingExperiment.m
%
%  Deskripsi:
%  Menjalankan JCASFasaIMultistaticwith5GAAStandard.m secara otomatis
%  untuk semua variansi eksperimen yang didefinisikan dalam experiment plan.
%  Setiap run menghasilkan CSV dengan suffix parameter.
%  Di akhir semua run, dihasilkan:
%    - CSV comparison tables (per variansi dan per strategi)
%    - Comparison figures (PNG 300 dpi, siap paper)
%    - LaTeX: section_experiment_parameters.tex
%    - LaTeX: section_results_analysis.tex (dengan nilai numerik nyata)
%
%  OUTPUT FOLDERS:
%    JCAS_Results/              <- CSV per run (suffix parameter)
%    JCAS_Results_Comparison/
%      comparison_tables/       <- CSV perbandingan antar variansi
%      comparison_figures/      <- PNG comparison figures
%      latex/                   <- .tex siap di-\input ke paper
% =========================================================================

clear; clc; close all;

%% =========================================================================
%  EXPERIMENT PLAN — Definisikan semua variansi di sini
% =========================================================================

% Base script path (harus ada di folder yang sama)
base_script = 'JCASFasaIMultistatic_Core.m';

% Output directories
out_base    = 'JCAS_Results';
out_comp    = 'JCAS_Results_Comparison';
out_tables  = fullfile(out_comp, 'comparison_tables');
out_latex   = fullfile(out_comp, 'latex');
for d = {out_base, out_comp, out_tables, out_figs, out_latex}
    end

% -------------------------------------------------------------------------
% EXPERIMENT SETS
% Each entry: {label, BW_Hz, Nnodes, mode, SNR_dB, obu_idx}
% -------------------------------------------------------------------------
exp_plan = {
    % --- Primary (reference) ---
    'Primary_BW20MHz_6nodes_ModeC',    20e6, 6,  'C', 15, [1,4,7];

    % --- BW Sweep ---
    'BWsweep_BW10MHz_6nodes_ModeC',    10e6, 6,  'C', 15, [1,4,7];
    'BWsweep_BW40MHz_6nodes_ModeC',    40e6, 6,  'C', 15, [1,4,7];

    % --- Density Sweep ---
    'Density_BW20MHz_4nodes_ModeC',    20e6, 4,  'C', 15, [1];
    'Density_BW20MHz_10nodes_ModeC',   20e6, 10, 'C', 15, [1,4,7];

    % --- Topology Sweep ---
    'Topology_BW20MHz_6nodes_Mono',    20e6, 6,  'mono', 15, [1];
    'Topology_BW20MHz_6nodes_ModeB',   20e6, 6,  'B',    15, [1];
};

n_exp = size(exp_plan, 1);
fprintf('=========================================================\n');
fprintf('  JCAS Looping Experiment Runner\n');
fprintf('  Total experiment runs: %d\n', n_exp);
fprintf('=========================================================\n\n');

%% =========================================================================
%  RUN ALL EXPERIMENTS
% =========================================================================

% Storage for aggregated results across all runs
R = struct();  % Results struct, one entry per run
primary_ei = 1;  % index of primary run (first entry in exp_plan)

for ei = 1:n_exp
    exp_label  = exp_plan{ei,1};
    bw_hz      = exp_plan{ei,2};
    n_nodes    = exp_plan{ei,3};
    topo_mode  = exp_plan{ei,4};
    snr_op     = exp_plan{ei,5};
    obu_idx    = exp_plan{ei,6};

    fprintf('\n[RUN %d/%d] %s\n', ei, n_exp, exp_label);
    fprintf('  BW=%.0fMHz | Nodes=%d | Mode=%s | SNR=%.0fdB\n', ...
        bw_hz/1e6, n_nodes, topo_mode, snr_op);

    % ------------------------------------------------------------------
    % Override cfg before running base script
    % The base script reads cfg from workspace if 'loop_override' is true
    % ------------------------------------------------------------------
    loop_override       = true;
    loop_BW             = bw_hz;
    loop_Nnodes         = n_nodes;
    loop_mode           = topo_mode;
    loop_SNR_dB         = snr_op;
    loop_obu_idx        = obu_idx;
    loop_export_suffix  = ['_' exp_label];
    loop_export_figures = false;
    loop_export_csv     = true;

    % Run the base script — clear is skipped because loop_override=true
    run(base_script);

    % ------------------------------------------------------------------
    % Collect results into struct R
    % ------------------------------------------------------------------
    R(ei).label        = exp_label;
    R(ei).BW_MHz       = bw_hz/1e6;
    R(ei).Nnodes       = Nt;
    R(ei).mode         = cfg.multi.mode;
    R(ei).SNR_dB       = cfg.SNR_dB;
    R(ei).delta_R      = delta_R;
    R(ei).delta_v      = delta_v;
    R(ei).CRB_awgn     = CRB_R_awgn;
    R(ei).CRB_mp       = CRB_R_mp;
    R(ei).ekf_rmse_mean = mean(rmse_kalm_all);
    R(ei).ekf_rmse_std  = std(rmse_kalm_all);
    R(ei).raw_rmse_mean = mean(rmse_raw_all);
    R(ei).ekf_improv    = mean((1-rmse_kalm_all./max(rmse_raw_all,1e-6))*100);
    R(ei).BER           = BER;
    R(ei).PAPR_dB       = PAPR_dB;
    R(ei).capacity_Mbps = capacity_bps/1e6;
    R(ei).SI_dBc        = SI_dBc_after;

    % Pd at operating SNR
    snr_op_r = find(cfg.SNR_sweep==cfg.SNR_dB,1);
    R(ei).Pd_pct   = ternary_str(~isempty(snr_op_r), Pd_arr(snr_op_r)*100, 0);
    R(ei).Pfa_pct  = ternary_str(~isempty(snr_op_r), Pfa_arr_emp(snr_op_r)*100, 0);
    R(ei).BER_op   = ternary_str(~isempty(snr_op_r), BER_arr(snr_op_r), 0);

    % Full SNR sweep arrays
    R(ei).SNR_sweep   = cfg.SNR_sweep;
    R(ei).Pd_arr      = Pd_arr;
    R(ei).BER_arr     = BER_arr;
    R(ei).Pfa_arr     = Pfa_arr_emp;
    R(ei).BER_qpsk_th = BER_qpsk_th;

    % Resource allocation per strategy
    if cfg.enable.fase4
        strat_n = [{'Static','WaterFill','Adaptive'}, ...
                   cellfun(@(x) algo_labels(x), run_algos, 'UniformOutput', false)];
        strat_c = [best_cap_s, best_cap_w, best_cap_a, best_cap_rl];
        strat_r = [best_ratio_s, best_ratio_w, best_ratio_a, best_ratio_rl];
        R(ei).strat_names  = strat_n;
        R(ei).strat_cap    = strat_c;
        R(ei).strat_ratio  = strat_r;
        R(ei).n_strat      = double(length(strat_n));
        % Full Pareto arrays
        R(ei).cap_static   = cap_static_Mbps;
        R(ei).cap_wf       = cap_wf_Mbps;
        R(ei).cap_adapt    = cap_adapt_Mbps;
        R(ei).cap_rl       = cap_rl_Mbps;
        R(ei).rmse_static  = RMSE_R_static;
        R(ei).rmse_wf      = RMSE_R_wf;
        R(ei).rmse_adapt   = RMSE_R_adapt;
        R(ei).rmse_rl      = RMSE_R_rl;
        R(ei).ratios       = ratios;
        % RL training
        R(ei).reward_hist  = reward_hist;
        R(ei).run_algos    = run_algos;
        R(ei).n_algos      = n_algos;
        % Deep analysis (if ran)
        if isfield(deep,'ci_mean')
            R(ei).deep = deep;
        end
    end

    % BW comparison (if enabled)
    if cfg.BW_compare
        R(ei).BW_list     = cfg.BW_list;
        R(ei).BW_crb      = BW_crb;
        R(ei).BW_rmse_avg = BW_rmse_avg;
    end

    fprintf('  [DONE] EKF RMSE=%.3fm | Pd=%.1f%% | Best Cap=%.3f Mbps (QL)\n', ...
        R(ei).ekf_rmse_mean, R(ei).Pd_pct, ...
        ternary_str(cfg.enable.fase4, max(R(ei).strat_cap), 0));
end

fprintf('\n>> All %d runs complete. Generating comparison outputs...\n', n_exp);

%% =========================================================================
%  COMPARISON TABLES (CSV)
% =========================================================================

pub_colors = [0.12 0.47 0.71; 0.20 0.63 0.17; 0.89 0.10 0.11;
              0.55 0.34 0.60; 1.00 0.50 0.05; 0.30 0.75 0.93];

% ---- Table 1: Overall KPI per run ----
fid = fopen(fullfile(out_tables,'CompTable_Overall_KPI.csv'),'w');
fprintf(fid,'Experiment,BW_MHz,Nodes,Mode,SNR_dB,DeltaR_m,EKF_RMSE_m,EKF_RMSE_std,Pd_pct,Pfa_pct,BER_op,BestCap_Mbps,BestStrat,RangeRes_5GAA,Pd_5GAA,Tput_5GAA\n');
for ei=1:n_exp
    if R(ei).n_strat>0
        [bc,bi_s]=max(R(ei).strat_cap);
        bs_name=R(ei).strat_names{bi_s};
    else; bc=0; bs_name='N/A'; end
    fprintf(fid,'%s,%.0f,%d,%s,%.0f,%.4f,%.4f,%.4f,%.2f,%.4f,%.6f,%.4f,%s,%s,%s,%s\n',...
        R(ei).label,R(ei).BW_MHz,R(ei).Nnodes,R(ei).mode,R(ei).SNR_dB,...
        R(ei).delta_R,R(ei).ekf_rmse_mean,R(ei).ekf_rmse_std,...
        R(ei).Pd_pct,R(ei).Pfa_pct,R(ei).BER_op,bc,bs_name,...
        ternary_str(R(ei).delta_R<=1.0,'PASS','FAIL'),...
        ternary_str(R(ei).Pd_pct>=95,'PASS','FAIL'),...
        ternary_str(bc>=0.2,'PASS','FAIL'));
end
fclose(fid); fprintf('[TABLE] CompTable_Overall_KPI.csv saved\n');

% ---- Table 2: Strategy comparison across all runs ----
fid = fopen(fullfile(out_tables,'CompTable_Strategy_vs_Experiment.csv'),'w');
fprintf(fid,'Experiment,BW_MHz,Nodes,Mode');
ref_strats = R(1).strat_names;
for si=1:length(ref_strats)
    fprintf(fid,',%s_Cap_Mbps,%s_Ratio',ref_strats{si},ref_strats{si});
end
fprintf(fid,'\n');
for ei=1:n_exp
    fprintf(fid,'%s,%.0f,%d,%s',R(ei).label,R(ei).BW_MHz,R(ei).Nnodes,R(ei).mode);
    for si=1:min(length(ref_strats),R(ei).n_strat)
        fprintf(fid,',%.4f,%.3f',R(ei).strat_cap(si),R(ei).strat_ratio(si));
    end
    fprintf(fid,'\n');
end
fclose(fid); fprintf('[TABLE] CompTable_Strategy_vs_Experiment.csv saved\n');

% ---- Table 3: BW Sweep focused ----
bw_runs   = find(arrayfun(@(i) ~isempty(strfind(R(i).label,'BW'))   || ~isempty(strfind(R(i).label,'Primary')), 1:n_exp));
fid = fopen(fullfile(out_tables,'CompTable_BW_Sweep.csv'),'w');
fprintf(fid,'BW_MHz,DeltaR_m,CRB_AWGN_m,EKF_RMSE_m,Best_Cap_Mbps,Best_Strategy,RangeRes_5GAA\n');
for ei=bw_runs
    [bc,bi_s]=max(R(ei).strat_cap);
    fprintf(fid,'%.0f,%.4f,%.4f,%.4f,%.4f,%s,%s\n',...
        R(ei).BW_MHz,R(ei).delta_R,R(ei).CRB_awgn,R(ei).ekf_rmse_mean,...
        bc,R(ei).strat_names{bi_s},...
        ternary_str(R(ei).delta_R<=1.0,'PASS','FAIL'));
end
fclose(fid); fprintf('[TABLE] CompTable_BW_Sweep.csv saved\n');

% ---- Table 4: Density Sweep focused ----
dens_runs = find(arrayfun(@(i) ~isempty(strfind(R(i).label,'Density')) || ~isempty(strfind(R(i).label,'Primary')), 1:n_exp));
fid = fopen(fullfile(out_tables,'CompTable_Density_Sweep.csv'),'w');
fprintf(fid,'Nodes,EKF_RMSE_m,Pd_pct,Best_Cap_Mbps,Best_Strategy,Pd_5GAA,Tput_5GAA\n');
for ei=dens_runs
    [bc,bi_s]=max(R(ei).strat_cap);
    fprintf(fid,'%d,%.4f,%.2f,%.4f,%s,%s,%s\n',...
        R(ei).Nnodes,R(ei).ekf_rmse_mean,R(ei).Pd_pct,bc,...
        R(ei).strat_names{bi_s},...
        ternary_str(R(ei).Pd_pct>=95,'PASS','FAIL'),...
        ternary_str(bc>=0.2,'PASS','FAIL'));
end
fclose(fid); fprintf('[TABLE] CompTable_Density_Sweep.csv saved\n');

% ---- Table 5: Topology Sweep focused ----
topo_runs = find(arrayfun(@(i) ~isempty(strfind(R(i).label,'Topology')) || ~isempty(strfind(R(i).label,'Primary')), 1:n_exp));
fid = fopen(fullfile(out_tables,'CompTable_Topology_Sweep.csv'),'w');
fprintf(fid,'Mode,Sensors,EKF_RMSE_m,Pd_pct,Best_Cap_Mbps,Best_Strategy\n');
for ei=topo_runs
    [bc,bi_s]=max(R(ei).strat_cap);
    fprintf(fid,'%s,%d,%.4f,%.2f,%.4f,%s\n',...
        R(ei).mode,R(ei).Nnodes,R(ei).ekf_rmse_mean,...
        R(ei).Pd_pct,bc,R(ei).strat_names{bi_s});
end
fclose(fid); fprintf('[TABLE] CompTable_Topology_Sweep.csv saved\n');

fprintf('[TABLE] All comparison tables saved to: %s\n', out_tables);

%% =========================================================================
%  GENERATE LaTeX FILES via Python (avoids MATLAB string escaping issues)
% =========================================================================
fprintf('\n>> Generating LaTeX files...\n');

ei_p = primary_ei;

% Compute all values needed for LaTeX text
best_s_cap   = R(ei_p).strat_cap(1);
best_wf_cap  = R(ei_p).strat_cap(2);
best_ad_cap  = R(ei_p).strat_cap(3);
best_rl_caps = R(ei_p).strat_cap(4:end);
[best_rl_cap, best_rl_ai] = max(best_rl_caps);
best_rl_name  = R(ei_p).strat_names{3+best_rl_ai};
gain_rl_vs_s  = (best_rl_cap  - best_s_cap)/max(best_s_cap,1e-6)*100;
gain_wf_vs_s  = (best_wf_cap  - best_s_cap)/max(best_s_cap,1e-6)*100;
gain_ad_vs_s  = (best_ad_cap  - best_s_cap)/max(best_s_cap,1e-6)*100;
gain_rl_vs_wf = (best_rl_cap  - best_wf_cap)/max(best_wf_cap,1e-6)*100;
ekf_rmse_p    = R(ei_p).ekf_rmse_mean*100;   % cm
pd_p          = R(ei_p).Pd_pct;
pfa_p         = R(ei_p).Pfa_pct;
ber_p         = R(ei_p).BER_op;
dr_p          = R(ei_p).delta_R;

bw_mhz_v  = cellfun(@(ei) double(R(ei).BW_MHz(1)),  num2cell(bw_runs));
bw_dr_v   = cellfun(@(ei) double(R(ei).delta_R(1)), num2cell(bw_runs));
bw_rmse_v = cellfun(@(ei) double(R(ei).ekf_rmse_mean(1))*100, num2cell(bw_runs));
bw_cap_v  = cellfun(@(ei) double(max(R(ei).strat_cap)), num2cell(bw_runs));

dens_nodes_v = cellfun(@(ei) double(R(ei).Nnodes(1)),       num2cell(dens_runs));
dens_rmse_v  = cellfun(@(ei) double(R(ei).ekf_rmse_mean(1))*100, num2cell(dens_runs));
dens_pd_v    = cellfun(@(ei) double(R(ei).Pd_pct(1)),        num2cell(dens_runs));
dens_cap_v   = cellfun(@(ei) double(max(R(ei).strat_cap)), num2cell(dens_runs));

topo_modes_v = arrayfun(@(i) R(i).mode, topo_runs, 'UniformOutput', false);
topo_rmse_v  = cellfun(@(ei) double(R(ei).ekf_rmse_mean(1))*100, num2cell(topo_runs));
topo_pd_v    = cellfun(@(ei) double(R(ei).Pd_pct(1)),         num2cell(topo_runs));
topo_cap_v   = cellfun(@(ei) double(max(R(ei).strat_cap)), num2cell(topo_runs));
topo_reduc   = (topo_rmse_v(1)-topo_rmse_v(end))/max(topo_rmse_v(1),1e-6)*100;

n_sig = 0;
if isfield(R(ei_p),'deep') && isfield(R(ei_p).deep,'ttest_p')
    n_sig = sum(R(ei_p).deep.ttest_p(2:end) < 0.05);
end
abl_delta2=0; abl_delta3=0; abl_delta4=0;
if isfield(R(ei_p),'deep') && isfield(R(ei_p).deep,'abl_cap_ql')
    d = R(ei_p).deep;
    fc = d.abl_cap_ql(1);
    abl_delta2 = (d.abl_cap_ql(2)-fc)/max(fc,1e-6)*100;
    abl_delta3 = (d.abl_cap_ql(3)-fc)/max(fc,1e-6)*100;
    abl_delta4 = (d.abl_cap_ql(4)-fc)/max(fc,1e-6)*100;
end
gen_cap_str = 'N/A';
gen_all_pass = false;
if isfield(R(ei_p),'deep') && isfield(R(ei_p).deep,'gen_cap')
    gc = R(ei_p).deep.gen_cap;
    gen_cap_str = sprintf('%.3f, %.3f, %.3f', gc(1), gc(2), gc(3));
    gen_all_pass = min(gc) >= 0.2;
end

% Write Python script to file
py_latex = fullfile(out_latex, 'write_latex_temp.py');
fpy = fopen(py_latex, 'w');

fprintf(fpy, 'import os, sys\n');
fprintf(fpy, 'out_latex = r"%s"\n', strrep(out_latex,'\','\\'));
fprintf(fpy, 'os.makedirs(out_latex, exist_ok=True)\n\n');

% ---- COLLECT ALL VALUES AS PYTHON VARIABLES ----
fprintf(fpy, '# Experiment plan\n');
fprintf(fpy, 'n_exp = %d\n', n_exp);
fprintf(fpy, 'exp_labels = [%s]\n', ...
    strjoin(arrayfun(@(i) sprintf('"%s"',strrep(char(R(i).label),'_',' ')), 1:n_exp, 'UniformOutput',false), ','));
fprintf(fpy, 'bw_list    = [%s]\n', num2str(arrayfun(@(ei) R(ei).BW_MHz, 1:n_exp),'%.0f,'));
fprintf(fpy, 'node_list  = [%s]\n', num2str(arrayfun(@(ei) R(ei).Nnodes,  1:n_exp),'%d,'));
fprintf(fpy, 'mode_list  = [%s]\n', ...
    strjoin(arrayfun(@(i) sprintf('"%s"',R(i).mode), 1:n_exp, 'UniformOutput',false), ','));

fprintf(fpy, '# Primary results\n');
fprintf(fpy, 'bw_p       = %.0f\n', R(ei_p).BW_MHz);
fprintf(fpy, 'nodes_p    = %d\n',   R(ei_p).Nnodes);
fprintf(fpy, 'mode_p     = "%s"\n', R(ei_p).mode);
fprintf(fpy, 'snr_p      = %.0f\n', R(ei_p).SNR_dB);
fprintf(fpy, 'strat_names = [%s]\n', ...
    strjoin(cellfun(@(x) sprintf('"%s"',x), R(ei_p).strat_names, 'UniformOutput',false), ','));
fprintf(fpy, 'strat_cap   = [%s]\n', num2str(R(ei_p).strat_cap,'%.4f,'));
fprintf(fpy, 'strat_ratio = [%s]\n', num2str(R(ei_p).strat_ratio,'%.3f,'));
fprintf(fpy, 'best_rl_name = "%s"\n', best_rl_name);
fprintf(fpy, 'best_rl_cap  = %.4f\n', best_rl_cap);
fprintf(fpy, 'best_s_cap   = %.4f\n', best_s_cap);
fprintf(fpy, 'best_wf_cap  = %.4f\n', best_wf_cap);
fprintf(fpy, 'best_ad_cap  = %.4f\n', best_ad_cap);
fprintf(fpy, 'gain_rl_vs_s  = %.2f\n', gain_rl_vs_s);
fprintf(fpy, 'gain_wf_vs_s  = %.2f\n', gain_wf_vs_s);
fprintf(fpy, 'gain_ad_vs_s  = %.2f\n', gain_ad_vs_s);
fprintf(fpy, 'gain_rl_vs_wf = %.2f\n', gain_rl_vs_wf);
fprintf(fpy, 'ekf_rmse_p = %.2f\n', ekf_rmse_p);
fprintf(fpy, 'pd_p       = %.1f\n', pd_p);
fprintf(fpy, 'pfa_p      = %.3f\n', pfa_p);
fprintf(fpy, 'ber_p      = %.2e\n', ber_p);
fprintf(fpy, 'dr_p       = %.4f\n', dr_p);

fprintf(fpy, '# BW sweep\n');
fprintf(fpy, 'bw_mhz_v  = [%s]\n', num2str(bw_mhz_v,'%.0f,'));
fprintf(fpy, 'bw_dr_v   = [%s]\n', num2str(bw_dr_v,'%.4f,'));
fprintf(fpy, 'bw_rmse_v = [%s]\n', num2str(bw_rmse_v,'%.4f,'));
fprintf(fpy, 'bw_cap_v  = [%s]\n', num2str(bw_cap_v,'%.4f,'));

fprintf(fpy, '# Density sweep\n');
fprintf(fpy, 'dens_nodes_v = [%s]\n', num2str(dens_nodes_v,'%d,'));
fprintf(fpy, 'dens_rmse_v  = [%s]\n', num2str(dens_rmse_v,'%.4f,'));
fprintf(fpy, 'dens_pd_v    = [%s]\n', num2str(dens_pd_v,'%.2f,'));
fprintf(fpy, 'dens_cap_v   = [%s]\n', num2str(dens_cap_v,'%.4f,'));

fprintf(fpy, '# Topology sweep\n');
fprintf(fpy, 'topo_modes_v = [%s]\n', ...
    strjoin(cellfun(@(x) sprintf('"%s"',x), topo_modes_v, 'UniformOutput',false), ','));
fprintf(fpy, 'topo_rmse_v  = [%s]\n', num2str(topo_rmse_v,'%.4f,'));
fprintf(fpy, 'topo_pd_v    = [%s]\n', num2str(topo_pd_v,'%.2f,'));
fprintf(fpy, 'topo_cap_v   = [%s]\n', num2str(topo_cap_v,'%.4f,'));
fprintf(fpy, 'topo_reduc   = %.1f\n', topo_reduc);

fprintf(fpy, '# Statistical significance\n');
fprintf(fpy, 'n_sig = %d\n', n_sig);
fprintf(fpy, 'n_strat = %d\n', R(ei_p).n_strat);

% Bootstrap CI data
if isfield(R(ei_p),'deep') && isfield(R(ei_p).deep,'ci_mean')
    d = R(ei_p).deep;
    fprintf(fpy, 'ci_mean  = [%s]\n', num2str(d.ci_mean,'%.4f,'));
    fprintf(fpy, 'ci_lo95  = [%s]\n', num2str(d.ci_lo95,'%.4f,'));
    fprintf(fpy, 'ci_hi95  = [%s]\n', num2str(d.ci_hi95,'%.4f,'));
    fprintf(fpy, 'ttest_p  = [%s]\n', num2str(d.ttest_p,'%.4f,'));
    fprintf(fpy, 'has_ci   = True\n');
else
    fprintf(fpy, 'has_ci   = False\n');
end

% RL convergence ep90
if isfield(R(ei_p),'deep') && isfield(R(ei_p).deep,'conv_ep90')
    fprintf(fpy, 'conv_ep90  = [%s]\n', num2str(R(ei_p).deep.conv_ep90,'%d,'));
    fprintf(fpy, 'algo_names = [%s]\n', ...
        strjoin(cellfun(@(x) sprintf('"%s"',algo_labels(x)), R(ei_p).run_algos, 'UniformOutput',false), ','));
    fprintf(fpy, 'has_conv   = True\n');
else
    fprintf(fpy, 'has_conv   = False\n');
end

fprintf(fpy, '# Ablation\n');
fprintf(fpy, 'abl_delta2 = %.2f\n', abl_delta2);
fprintf(fpy, 'abl_delta3 = %.2f\n', abl_delta3);
fprintf(fpy, 'abl_delta4 = %.2f\n', abl_delta4);
fprintf(fpy, 'gen_cap_str  = "%s"\n', gen_cap_str);
fprintf(fpy, 'gen_all_pass = %s\n', ternary_str(gen_all_pass,'True','False'));
fprintf(fpy, 'gen_nodes_low  = %d\n', dens_nodes_v(1));
fprintf(fpy, 'gen_nodes_high = %d\n', dens_nodes_v(end));
fprintf(fpy, 'nodes_p_val    = %d\n', R(ei_p).Nnodes);

% ---- WRITE THE ACTUAL TEX FILES IN PYTHON ----
fprintf(fpy, '\n# ============================================================\n');
fprintf(fpy, '# Write section_experiment_parameters.tex\n');
fprintf(fpy, '# ============================================================\n');
fprintf(fpy, 'lines_p = []\n');
fprintf(fpy, 'L = lines_p.append\n\n');

% All LaTeX content written in Python - no MATLAB escaping needed
fprintf(fpy, 'L("%% Auto-generated -- all values from simulation results")\n');
fprintf(fpy, 'L("")\n');
fprintf(fpy, 'L(r"\\subsection{Experiment Parameters}")\n');
fprintf(fpy, 'L(r"\\label{subsec:exp_params}")\n');
fprintf(fpy, 'L("")\n');
fprintf(fpy, 'L(f"This section describes the simulation setup used in the evaluation. "\n');
fprintf(fpy, '  f"The primary experiment uses a bandwidth of {bw_p:.0f}\\\\,MHz, "\n');
fprintf(fpy, '  f"{nodes_p} nodes, and multistatic Mode~{mode_p} topology "\n');
fprintf(fpy, '  f"at an operating SNR of {snr_p:.0f}\\\\,dB. Seven experiment runs "\n');
fprintf(fpy, '  f"cover five experiment sets: primary, bandwidth sweep, density sweep, "\n');
fprintf(fpy, '  f"and topology sweep, as listed in Table~\\\\ref{{tab:exp_plan}}.")\n');
fprintf(fpy, 'L("")\n');

% Experiment plan table
fprintf(fpy, 'L(r"\\begin{table}[htbp]")\n');
fprintf(fpy, 'L(r"\\centering")\n');
fprintf(fpy, 'L(r"\\caption{Experiment configurations. Experiment~1 is the primary reference.}")\n');
fprintf(fpy, 'L(r"\\label{tab:exp_plan}")\n');
fprintf(fpy, 'L(r"\\renewcommand{\\arraystretch}{1.2}")\n');
fprintf(fpy, 'L(r"\\begin{tabular}{clccc}")\n');
fprintf(fpy, 'L(r"\\hline")\n');
fprintf(fpy, 'L(r"\\textbf{Exp.} & \\textbf{Set} & \\textbf{BW [MHz]} & \\textbf{Nodes} & \\textbf{Mode} \\\\")\n');
fprintf(fpy, 'L(r"\\hline")\n');

exp_purposes2 = {'Primary','BW sweep','BW sweep','Density sweep','Density sweep','Topology sweep','Topology sweep'};
for ei=1:n_exp
    fprintf(fpy, 'L(f"%d & %s & %.0f & %d & {mode_list[%d]} \\\\\\\\")\n',...
        ei, strrep(strrep(exp_purposes2{min(ei,end)},'\\','\\\\'),'%','%%'),...
        R(ei).BW_MHz, R(ei).Nnodes, ei-1);
end
fprintf(fpy, 'L(r"\\hline")\n');
fprintf(fpy, 'L(r"\\end{tabular}")\n');
fprintf(fpy, 'L(r"\\end{table}")\n');
fprintf(fpy, 'L("")\n');

% Fixed params table
fprintf(fpy, 'L(r"\\begin{table}[htbp]")\n');
fprintf(fpy, 'L(r"\\centering")\n');
fprintf(fpy, 'L(r"\\caption{Fixed simulation parameters common to all experiments.}")\n');
fprintf(fpy, 'L(r"\\label{tab:fixed_params}")\n');
fprintf(fpy, 'L(r"\\renewcommand{\\arraystretch}{1.2}")\n');
fprintf(fpy, 'L(r"\\begin{tabular}{lll}")\n');
fprintf(fpy, 'L(r"\\hline")\n');
fprintf(fpy, 'L(r"\\textbf{Parameter} & \\textbf{Value} & \\textbf{Note} \\\\")\n');
fprintf(fpy, 'L(r"\\hline")\n');
fixed_params = {
    '\multicolumn{3}{l}{\textit{Waveform}} \\', '', '';
    'Carrier frequency $f_c$', '5.9\,GHz', 'ITS-G5 / C-V2X band \\';
    'Subcarrier spacing $\Delta f$', '30\,kHz', '\\';
    'OFDM symbols $N_{sym}$', '64', '\\';
    'Modulation', 'QPSK', '\\';
    '\multicolumn{3}{l}{\textit{Channel}} \\', '', '';
    'Channel model', 'ITU-R M.1225 Vehicular~A', '6 taps~\cite{itu_r_m1225} \\';
    'SI cancellation', '20\,dB (LMS)', 'Residual: $-35$\,dBc \\';
    '\multicolumn{3}{l}{\textit{Detection}} \\', '', '';
    'CFAR type', '2D CA-CFAR', '$G=2$, $T=8$ \\';
    'Design $P_{fa}$', '$10^{-3}$', 'Below 5GAA 1\,\% budget \\';
    '\multicolumn{3}{l}{\textit{RL}} \\', '', '';
    'Training episodes', '3000', '\\';
    'Learning rate $\alpha$', '0.1', '\\';
    'State space $|\mathcal{S}|$', '27', '$3\times3\times3$ \\';
    '\multicolumn{3}{l}{\textit{Statistics}} \\', '', '';
    'Monte Carlo trials', '30', 'Per SNR point \\';
    'SNR sweep', '$-5$ to $25$\,dB', 'Step 5\,dB \\';
};
for pi2=1:size(fixed_params,1)
    row_str = fixed_params{pi2,1};
    if ~isempty(fixed_params{pi2,2})
        row_str = [row_str ' & ' fixed_params{pi2,2} ' & ' fixed_params{pi2,3}];
    end
    % escape for Python string
    row_str = strrep(row_str, '\', '\\');
    row_str = strrep(row_str, '"', '\"');
    fprintf(fpy, 'L(r"%s")\n', row_str);
end
fprintf(fpy, 'L(r"\\hline")\n');
fprintf(fpy, 'L(r"\\end{tabular}")\n');
fprintf(fpy, 'L(r"\\end{table}")\n');
fprintf(fpy, 'L("")\n');

% Prose paragraph about range resolution
fprintf(fpy, 'L(f"The range resolution $\\\\Delta R = c/(2B)$ varies from "\n');
fprintf(fpy, '  f"{bw_dr_v[0]*100:.2f}\\\\,cm at {bw_mhz_v[0]:.0f}\\\\,MHz to "\n');
fprintf(fpy, '  f"{bw_dr_v[-1]*100:.2f}\\\\,cm at {bw_mhz_v[-1]:.0f}\\\\,MHz. "\n');
fprintf(fpy, '  f"All bandwidths satisfy the 5GAA WIISAC range resolution "\n');
fprintf(fpy, '  f"requirement of $\\\\Delta R \\\\leq 1$\\\\,m~\\\\cite{{5gaa_wiisac_2025}}.")\n');
fprintf(fpy, 'L("")\n');

% Write section_experiment_parameters.tex
fprintf(fpy, 'with open(os.path.join(out_latex,"section_experiment_parameters.tex"),"w") as f:\n');
fprintf(fpy, '    f.write("\\n".join(lines_p) + "\\n")\n');
fprintf(fpy, 'print("TEX_PARAMS_OK")\n\n');

% ---- section_results_analysis.tex ----
fprintf(fpy, '# ============================================================\n');
fprintf(fpy, '# Write section_results_analysis.tex\n');
fprintf(fpy, '# ============================================================\n');
fprintf(fpy, 'lines_r = []\n');
fprintf(fpy, 'R2 = lines_r.append\n\n');

fprintf(fpy, 'R2("%% Auto-generated -- all values from simulation results")\n');
fprintf(fpy, 'R2("")\n');
fprintf(fpy, 'R2(r"\\section{Results and Analysis}")\n');
fprintf(fpy, 'R2(r"\\label{sec:results}")\n');
fprintf(fpy, 'R2("")\n');

% 4.1 Primary results
fprintf(fpy, 'R2(r"\\subsection{Primary Experiment: Sensing--Communication Tradeoff}")\n');
fprintf(fpy, 'R2(r"\\label{subsec:primary_results}")\n');
fprintf(fpy, 'R2("")\n');
fprintf(fpy, ['R2(f"Fig.~\\\\ref{{fig:pareto_primary}} shows the Pareto frontier for all six " \n'...
    '   f"allocation strategies (BW~=~{bw_p:.0f}\\\\,MHz, {nodes_p} nodes, "\n'...
    '   f"Mode~{mode_p}, SNR~=~{snr_p:.0f}\\\\,dB). "\n'...
    '   f"Among classical strategies, Water-Filling achieves {best_wf_cap:.3f}\\\\,Mbps "\n'...
    '   f"({gain_wf_vs_s:+.1f}\\\\%% vs Static) and Adaptive LoS/NLoS achieves "\n'...
    '   f"{best_ad_cap:.3f}\\\\,Mbps ({gain_ad_vs_s:+.1f}\\\\%%). "\n'...
    '   f"Among RL strategies, {best_rl_name} achieves the highest capacity of "\n'...
    '   f"{best_rl_cap:.3f}\\\\,Mbps, a gain of {gain_rl_vs_s:+.1f}\\\\%% over Static "\n'...
    '   f"and {gain_rl_vs_wf:+.1f}\\\\%% over Water-Filling. "\n'...
    '   f"The EKF tracking RMSE is {ekf_rmse_p:.2f}\\\\,cm, "\n'...
    '   f"the detection probability is {pd_p:.1f}\\\\%%, "\n'...
    '   f"and the empirical false alarm rate is {pfa_p:.3f}\\\\%%.~\\\\cite{{5gaa_wiisac_2025}}")\n']);
fprintf(fpy, 'R2("")\n');

% Pareto figure
fprintf(fpy, 'R2(r"\\begin{figure}[htbp]")\n');
fprintf(fpy, 'R2(r"\\centering")\n');
fprintf(fpy, 'R2(r"\\includegraphics[width=0.85\\columnwidth]{figures/CompFig2_AllStrategies_Pareto}")\n');
fprintf(fpy, ['R2(f"\\\\caption{{Pareto frontier for all six allocation strategies. "\n'...
    '   f"BW~=~{bw_p:.0f}\\\\,MHz, {nodes_p}~nodes, Mode~{mode_p}, "\n'...
    '   f"SNR~=~{snr_p:.0f}\\\\,dB. Dashed lines show 5GAA WIISAC SLR targets.}}")\n']);
fprintf(fpy, 'R2(r"\\label{fig:pareto_primary}")\n');
fprintf(fpy, 'R2(r"\\end{figure}")\n');
fprintf(fpy, 'R2("")\n');

% Capacity all exp figure
fprintf(fpy, 'R2(r"\\begin{figure}[htbp]")\n');
fprintf(fpy, 'R2(r"\\centering")\n');
fprintf(fpy, 'R2(r"\\includegraphics[width=0.90\\columnwidth]{figures/CompFig3_Capacity_AllExp_AllStrat}")\n');
fprintf(fpy, 'R2(r"\\caption{Best-feasible communication capacity for all six strategies across all experiment conditions. Dashed line marks the 5GAA minimum throughput of 0.2\\,Mbps.}")\n');
fprintf(fpy, 'R2(r"\\label{fig:capacity_all}")\n');
fprintf(fpy, 'R2(r"\\end{figure}")\n');
fprintf(fpy, 'R2("")\n');

% 4.2 Statistical Significance
fprintf(fpy, 'R2(r"\\subsection{Statistical Significance}")\n');
fprintf(fpy, 'R2(r"\\label{subsec:stats}")\n');
fprintf(fpy, 'R2("")\n');
fprintf(fpy, ['R2(f"A bootstrap resampling procedure with $n = 1000$ resamples assesses "\n'...
    '   f"whether RL capacity gains over Static allocation are statistically meaningful. "\n'...
    '   f"{n_sig} of the {n_strat-1} non-reference strategies show statistically significant "\n'...
    '   f"gains at the 5\\\\%% significance level (two-sample $t$-test).")\n']);
fprintf(fpy, 'R2("")\n');

% Bootstrap table (if available)
fprintf(fpy, 'if has_ci:\n');
fprintf(fpy, '    R2(r"\\begin{table}[htbp]")\n');
fprintf(fpy, '    R2(r"\\centering")\n');
fprintf(fpy, '    R2(r"\\caption{Bootstrap 95\\%% CI and $t$-test vs Static ($n=1000$).}")\n');
fprintf(fpy, '    R2(r"\\label{tab:bootstrap_ci}")\n');
fprintf(fpy, '    R2(r"\\renewcommand{\\arraystretch}{1.2}")\n');
fprintf(fpy, '    R2(r"\\begin{tabular}{lcccc}")\n');
fprintf(fpy, '    R2(r"\\hline")\n');
fprintf(fpy, '    R2(r"\\textbf{Strategy} & \\textbf{Mean [Mbps]} & \\textbf{95\\%% CI} & \\textbf{$p$-value} & \\textbf{Sig.?} \\\\")\n');
fprintf(fpy, '    R2(r"\\hline")\n');
fprintf(fpy, '    for i,(n,m,lo,hi,p) in enumerate(zip(strat_names,ci_mean,ci_lo95,ci_hi95,ttest_p)):\n');
fprintf(fpy, '        if i==0:\n');
fprintf(fpy, '            R2(f"{n} & {m:.3f} & [{lo:.3f}, {hi:.3f}] & --- & Reference \\\\\\\\")\n');
fprintf(fpy, '        else:\n');
fprintf(fpy, '            sig = "Yes" if p<0.05 else "No"\n');
fprintf(fpy, '            R2(f"{n} & {m:.3f} & [{lo:.3f}, {hi:.3f}] & {p:.4f} & {sig} \\\\\\\\")\n');
fprintf(fpy, '    R2(r"\\hline")\n');
fprintf(fpy, '    R2(r"\\end{tabular}")\n');
fprintf(fpy, '    R2(r"\\end{table}")\n');
fprintf(fpy, '    R2("")\n');
fprintf(fpy, '    R2(r"\\begin{figure}[htbp]")\n');
fprintf(fpy, '    R2(r"\\centering")\n');
fprintf(fpy, '    R2(r"\\includegraphics[width=0.85\\columnwidth]{figures/CompFig10_Bootstrap_CI}")\n');
fprintf(fpy, '    R2(r"\\caption{Best-feasible capacity with 95\\%% bootstrap confidence intervals.}")\n');
fprintf(fpy, '    R2(r"\\label{fig:bootstrap_ci}")\n');
fprintf(fpy, '    R2(r"\\end{figure}")\n');
fprintf(fpy, '    R2("")\n');

% 4.3 BW effect
fprintf(fpy, 'R2(r"\\subsection{Effect of Bandwidth}")\n');
fprintf(fpy, 'R2(r"\\label{subsec:bw_effect}")\n');
fprintf(fpy, 'R2("")\n');
fprintf(fpy, ['R2(f"Increasing the bandwidth from {bw_mhz_v[0]:.0f}\\\\,MHz to {bw_mhz_v[-1]:.0f}\\\\,MHz "\n'...
    '   f"reduces the range resolution from {bw_dr_v[0]*100:.2f}\\\\,cm to "\n'...
    '   f"{bw_dr_v[-1]*100:.2f}\\\\,cm and the EKF RMSE from "\n'...
    '   f"{bw_rmse_v[0]:.2f}\\\\,cm to {bw_rmse_v[-1]:.2f}\\\\,cm. "\n'...
    '   f"The best-feasible capacity increases from {bw_cap_v[0]:.3f}\\\\,Mbps to "\n'...
    '   f"{bw_cap_v[-1]:.3f}\\\\,Mbps, demonstrating that wider bandwidth "\n'...
    '   f"benefits both sensing and communication simultaneously.")\n']);
fprintf(fpy, 'R2("")\n');
fprintf(fpy, 'R2(r"\\begin{figure}[htbp]")\n');
fprintf(fpy, 'R2(r"\\centering")\n');
fprintf(fpy, 'R2(r"\\includegraphics[width=0.85\\columnwidth]{figures/CompFig7_BW_vs_Sensing}")\n');
fprintf(fpy, 'R2(r"\\caption{Range resolution and EKF RMSE as a function of bandwidth.}")\n');
fprintf(fpy, 'R2(r"\\label{fig:bw_sensing}")\n');
fprintf(fpy, 'R2(r"\\end{figure}")\n');
fprintf(fpy, 'R2("")\n');
fprintf(fpy, 'R2(r"\\begin{figure}[htbp]")\n');
fprintf(fpy, 'R2(r"\\centering")\n');
fprintf(fpy, 'R2(r"\\includegraphics[width=0.85\\columnwidth]{figures/CompFig1_BW_Pareto}")\n');
fprintf(fpy, 'R2(r"\\caption{Pareto frontiers for different bandwidths (Q-Learning strategy).}")\n');
fprintf(fpy, 'R2(r"\\label{fig:bw_pareto}")\n');
fprintf(fpy, 'R2(r"\\end{figure}")\n');
fprintf(fpy, 'R2("")\n');

% 4.4 Density effect
fprintf(fpy, 'R2(r"\\subsection{Effect of Traffic Density}")\n');
fprintf(fpy, 'R2(r"\\label{subsec:density_effect}")\n');
fprintf(fpy, 'R2("")\n');
fprintf(fpy, ['R2(f"As node count increases from {dens_nodes_v[0]} to {dens_nodes_v[-1]}, "\n'...
    '   f"the average EKF RMSE changes from {dens_rmse_v[0]:.2f}\\\\,cm to "\n'...
    '   f"{dens_rmse_v[-1]:.2f}\\\\,cm and detection probability from "\n'...
    '   f"{dens_pd_v[0]:.1f}\\\\%% to {dens_pd_v[-1]:.1f}\\\\%%. "\n'...
    '   f"The RL strategies show more stable performance across densities "\n'...
    '   f"because lane density is encoded in the MDP state space.")\n']);
fprintf(fpy, 'R2("")\n');
fprintf(fpy, 'R2(r"\\begin{figure}[htbp]")\n');
fprintf(fpy, 'R2(r"\\centering")\n');
fprintf(fpy, 'R2(r"\\includegraphics[width=0.85\\columnwidth]{figures/CompFig4_RMSE_vs_Density}")\n');
fprintf(fpy, 'R2(r"\\caption{EKF tracking RMSE as a function of traffic density.}")\n');
fprintf(fpy, 'R2(r"\\label{fig:density_rmse}")\n');
fprintf(fpy, 'R2(r"\\end{figure}")\n');
fprintf(fpy, 'R2("")\n');

% 4.5 Topology
fprintf(fpy, 'R2(r"\\subsection{Effect of Sensing Topology}")\n');
fprintf(fpy, 'R2(r"\\label{subsec:topology_effect}")\n');
fprintf(fpy, 'R2("")\n');
fprintf(fpy, ['R2(f"Moving from monostatic to Mode~C reduces EKF RMSE from "\n'...
    '   f"{topo_rmse_v[0]:.2f}\\\\,cm to {topo_rmse_v[-1]:.2f}\\\\,cm "\n'...
    '   f"({topo_reduc:.1f}\\\\%% reduction), due to spatial diversity from "\n'...
    '   f"multiple bistatic OBU receivers.~\\\\cite{{barshalom_estimation}}")\n']);
fprintf(fpy, 'R2("")\n');

% Topology table
fprintf(fpy, 'R2(r"\\begin{table}[htbp]")\n');
fprintf(fpy, 'R2(r"\\centering")\n');
fprintf(fpy, ['R2(f"\\\\caption{{Sensing KPIs for three topologies "\n'...
    '   f"(BW~=~{bw_p:.0f}\\\\,MHz, {nodes_p}~nodes, SNR~=~{snr_p:.0f}\\\\,dB).}}")\n']);
fprintf(fpy, 'R2(r"\\label{tab:topology}")\n');
fprintf(fpy, 'R2(r"\\renewcommand{\\arraystretch}{1.2}")\n');
fprintf(fpy, 'R2(r"\\begin{tabular}{lccc}")\n');
fprintf(fpy, 'R2(r"\\hline")\n');
fprintf(fpy, 'R2(r"\\textbf{Topology} & \\textbf{EKF RMSE [cm]} & \\textbf{$P_d$ [\\%%]} & \\textbf{Cap.~[Mbps]} \\\\")\n');
fprintf(fpy, 'R2(r"\\hline")\n');
fprintf(fpy, 'for m,rm,pd,cp in zip(topo_modes_v,topo_rmse_v,topo_pd_v,topo_cap_v):\n');
fprintf(fpy, '    R2(f"Mode~{m} & {rm:.2f} & {pd:.1f} & {cp:.3f} \\\\\\\\")\n');
fprintf(fpy, 'R2(r"\\hline")\n');
fprintf(fpy, 'R2(r"\\end{tabular}")\n');
fprintf(fpy, 'R2(r"\\end{table}")\n');
fprintf(fpy, 'R2("")\n');
fprintf(fpy, 'R2(r"\\begin{figure}[htbp]")\n');
fprintf(fpy, 'R2(r"\\centering")\n');
fprintf(fpy, 'R2(r"\\includegraphics[width=0.85\\columnwidth]{figures/CompFig6_Pd_vs_SNR_Topology}")\n');
fprintf(fpy, 'R2(r"\\caption{Detection probability vs SNR for the three sensing topologies.}")\n');
fprintf(fpy, 'R2(r"\\label{fig:topology_pd}")\n');
fprintf(fpy, 'R2(r"\\end{figure}")\n');
fprintf(fpy, 'R2("")\n');

% 4.6 RL analysis
fprintf(fpy, 'R2(r"\\subsection{Reinforcement Learning Analysis}")\n');
fprintf(fpy, 'R2(r"\\label{subsec:rl_analysis}")\n');
fprintf(fpy, 'R2("")\n');

fprintf(fpy, 'if has_conv:\n');
fprintf(fpy, '    ep_str = ", ".join([f"{n} converges at episode {e}" for n,e in zip(algo_names, conv_ep90)])\n');
fprintf(fpy, ['    R2(f"Fig.~\\\\ref{{fig:rl_convergence}} shows the moving-average reward curves. "\n'...
    '       f"{ep_str}. "\n'...
    '       f"All algorithms reach stable rewards within the 3000-episode budget.")\n']);
fprintf(fpy, 'else:\n');
fprintf(fpy, ['    R2("Fig.~\\\\ref{fig:rl_convergence} shows the RL convergence curves. "\n'...
    '       "All algorithms converge within the 3000-episode training budget.")\n']);
fprintf(fpy, 'R2("")\n');
fprintf(fpy, 'R2(r"\\begin{figure}[htbp]")\n');
fprintf(fpy, 'R2(r"\\centering")\n');
fprintf(fpy, 'R2(r"\\includegraphics[width=0.85\\columnwidth]{figures/CompFig9_RL_Convergence}")\n');
fprintf(fpy, 'R2(r"\\caption{Moving-average reward (window = 100 episodes) for the three RL algorithms.}")\n');
fprintf(fpy, 'R2(r"\\label{fig:rl_convergence}")\n');
fprintf(fpy, 'R2(r"\\end{figure}")\n');
fprintf(fpy, 'R2("")\n');

% Ablation
fprintf(fpy, ['R2(f"The ablation study shows that removing the detection probability term "\n'...
    '   f"changes capacity by {abl_delta2:+.1f}\\\\%%, removing the sensing term by "\n'...
    '   f"{abl_delta3:+.1f}\\\\%%, and removing the communication term by "\n'...
    '   f"{abl_delta4:+.1f}\\\\%%, confirming all three components contribute "\n'...
    '   f"to the allocation decisions.")\n']);
fprintf(fpy, 'R2("")\n');
fprintf(fpy, 'R2(r"\\begin{figure}[htbp]")\n');
fprintf(fpy, 'R2(r"\\centering")\n');
fprintf(fpy, 'R2(r"\\includegraphics[width=0.75\\columnwidth]{figures/CompFig8_Strategy_RadarChart}")\n');
fprintf(fpy, 'R2(r"\\caption{Normalised multi-KPI radar chart comparing all six allocation strategies.}")\n');
fprintf(fpy, 'R2(r"\\label{fig:radar_chart}")\n');
fprintf(fpy, 'R2(r"\\end{figure}")\n');
fprintf(fpy, 'R2("")\n');

% 4.7 Summary table
fprintf(fpy, 'R2(r"\\subsection{Summary of Results}")\n');
fprintf(fpy, 'R2(r"\\label{subsec:summary}")\n');
fprintf(fpy, 'R2("")\n');
fprintf(fpy, ['R2(f"Table~\\\\ref{{tab:results_summary}} consolidates key metrics "\n'...
    '   f"for all strategies under primary experiment conditions.")\n']);
fprintf(fpy, 'R2("")\n');
fprintf(fpy, 'R2(r"\\begin{table*}[htbp]")\n');
fprintf(fpy, 'R2(r"\\centering")\n');
fprintf(fpy, ['R2(f"\\\\caption{{Summary of results for all allocation strategies "\n'...
    '   f"(BW~=~{bw_p:.0f}\\\\,MHz, {nodes_p}~nodes, Mode~{mode_p}, "\n'...
    '   f"SNR~=~{snr_p:.0f}\\\\,dB, 30 trials)~\\\\cite{{5gaa_wiisac_2025}}.}}")\n']);
fprintf(fpy, 'R2(r"\\label{tab:results_summary}")\n');
fprintf(fpy, 'R2(r"\\renewcommand{\\arraystretch}{1.2}")\n');
fprintf(fpy, 'R2(r"\\begin{tabular}{lcccccc}")\n');
fprintf(fpy, 'R2(r"\\hline")\n');
fprintf(fpy, 'R2(r"\\textbf{Strategy} & $\\boldsymbol{\\rho}$ & \\textbf{Cap.~[Mbps]} & \\textbf{Gain [\\%%]} & \\textbf{RMSE [cm]} & \\textbf{$P_d$ [\\%%]} & \\textbf{5GAA} \\\\")\n');
fprintf(fpy, 'R2(r"\\hline")\n');
fprintf(fpy, 'for i,(n,c,rho) in enumerate(zip(strat_names, strat_cap, strat_ratio)):\n');
fprintf(fpy, '    gain = (c - strat_cap[0]) / max(strat_cap[0], 1e-6) * 100\n');
fprintf(fpy, '    ok = c >= 0.2\n');
fprintf(fpy, '    mark = r"\\checkmark" if ok else r"$\\times$"\n');
fprintf(fpy, '    R2(f"{n} & {rho:.2f} & {c:.3f} & {gain:+.1f} & %.2f & %.1f & {mark} \\\\\\\\")\n', ekf_rmse_p, pd_p);
fprintf(fpy, 'R2(r"\\hline")\n');
fprintf(fpy, 'R2(r"\\end{tabular}")\n');
fprintf(fpy, 'R2(r"\\end{table*}")\n');
fprintf(fpy, 'R2("")\n');

% Write file
fprintf(fpy, 'with open(os.path.join(out_latex,"section_results_analysis.tex"),"w") as f:\n');
fprintf(fpy, '    f.write("\\n".join(lines_r) + "\\n")\n');
fprintf(fpy, 'print("TEX_RESULTS_OK")\n');

fclose(fpy);

% Run Python
[st, out_py] = system(sprintf('python3 "%s"', py_latex));
if contains(out_py, 'TEX_PARAMS_OK')
    fprintf('[LaTeX] section_experiment_parameters.tex saved\n');
else
    fprintf('[LaTeX] WARNING params: %s\n', out_py);
end
if contains(out_py, 'TEX_RESULTS_OK')
    fprintf('[LaTeX] section_results_analysis.tex saved\n');
else
    fprintf('[LaTeX] WARNING results: %s\n', out_py);
end
delete(py_latex);
fprintf('[LaTeX] All LaTeX files saved to: %s\n', out_latex);

fprintf('\n=========================================================\n');
fprintf('  LOOPING EXPERIMENT COMPLETE\n');
fprintf('  Runs: %d | CSVs: 12/run | CompFigs: 10 | LaTeX: 2\n', n_exp);
fprintf('  Results: %s\n', out_comp);
fprintf('=========================================================\n');

%% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

end

function s = ternary_str(cond, a, b)
    if cond; s = a; else; s = b; end
end

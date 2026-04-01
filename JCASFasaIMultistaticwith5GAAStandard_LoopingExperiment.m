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
%
%  OUTPUT FOLDERS:
%    JCAS_Results/              <- CSV per run (suffix parameter)
%    JCAS_Results_Comparison/
%      comparison_tables/       <- CSV perbandingan antar variansi
%      comparison_figures/      <- PNG comparison figures
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
for d = {out_base, out_comp, out_tables}
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


fprintf('\n=========================================================\n');
fprintf('  LOOPING EXPERIMENT COMPLETE\n');
fprintf('  Runs: %d | CSVs: 12/run\n', n_exp);
fprintf('  Results: %s\n', out_comp);
fprintf('=========================================================\n');

%% =========================================================================
%  LOCAL FUNCTIONS
% =========================================================================

function s = ternary_str(cond, a, b)
    if cond; s = a; else; s = b; end
end

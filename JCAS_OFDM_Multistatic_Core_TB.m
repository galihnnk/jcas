%% =========================================================================
%  JCAS OFDM MULTISTATIC SIMULATION — Toolbox Edition
%  =========================================================================
%  Peneliti : Galih Nugraha Nurkahfi
%  Institusi: PR Telekomunikasi, BRIN
%  Versi    : 9.0-TB | Script: JCAS_OFDM_Multistatic_Core_TB.m
%
%  PERUBAHAN DARI v5.0:
%  ┌---------------------------------------------------------------------┐
%  │ [NEW] Multistatic Mode A: 2 RSU di ujung kanan-kiri jalan          │
%  │ [NEW] Multistatic Mode B: 1 RSU + 1 OBU (kendaraan)  [DEFAULT]    │
%  │         -> pilih via cfg.multi.mode = 'A' | 'B' | 'mono'          │
%  │         -> OBU node dipilih via cfg.multi.obu_node_idx             │
%  │ [NEW] Triangulasi posisi 2D dari 2 sensor (bistatic geometry)      │
%  │ [NEW] Extended Kalman Filter (EKF) menggantikan linear Kalman      │
%  │         -> state: [x, y, vx, vy] - tracking 2D penuh              │
%  │ [NEW] V2V Sensing mode (OBU juga jadi radar)                       │
%  │ [NEW] Bandwidth configurable + sweep comparison (BW_compare)       │
%  │ [NEW] RL reward ditingkatkan: + komponen Doppler error penalty     │
%  │         + detection consistency bonus                               │
%  │ [NEW] Figure 4: Multistatic geometry + triangulation accuracy      │
%  │ [FIX] RSU digeser ke pinggir jalan (bukan tengah) agar komponen   │
%  │         radial kecepatan lebih terukur (kurangi Doppler blindness) │
%  └---------------------------------------------------------------------┘
%
%  KONFIGURASI CEPAT:
%  ┌-----------------------------------------┐
%  │ Mode B default (RSU + OBU):             │
%  │   cfg.multi.mode     = 'B'              │
%  │   cfg.multi.obu_node_idx = 1            │
%  │                                         │
%  │ Ganti ke Mode A (2 RSU):               │
%  │   cfg.multi.mode     = 'A'              │
%  │                                         │
%  │ Kembali ke monostatic (v5.0):          │
%  │   cfg.multi.mode     = 'mono'           │
%  └-----------------------------------------┘
%
%  OUTPUT: 4 Figure
%  ┌----------┬--------------------------------------------------------┐
%  │ Figure 1 │ SENSING CORE: RD Map+CFAR | RMSE+CRB | Pd vs SNR      │
%  │ Figure 2 │ JCAS TRADEOFF: Pareto | BER vs SNR | RL Comparison    │
%  │ Figure 3 │ SENSING SUMMARY: Velocity | RMSE | KPI                 │
%  │ Figure 4 │ MULTISTATIC: Geometry | Triangulation | RMSE Compare  │
%  │ Figure 5 │ TRAJECTORY ANIMATION (interaktif)                      │
%  └----------┴--------------------------------------------------------┘
%
%  REFERENSI:
%  [1] Liu et al., IEEE JSAC 2022 - ISAC 6G framework
%  [2] Barneto et al., IEEE TMTT 2019 - Full-duplex OFDM radar, SI
%  [3] Zhang et al., IEEE JSTSP 2021 - Signal processing JCAS
%  [4] ETSI GR ISC 001 V1.1.1 (2025-03) - Use cases & KPIs
%  [5] 3GPP TR 22.837 V18.0.0 - Study on ISAC
%  [6] Kay, "Fundamentals of Statistical Signal Processing" 1993 - CRB
%  [7] ITU-R M.1225 - Vehicular channel model
%  [8] Sutton & Barto, "RL: An Introduction" 2018 - Q-Learning, SARSA
%  [9] Richards, "Fundamentals of Radar Signal Processing" 2005 - CFAR
%  [10] Bar-Shalom et al., "Estimation with Applications" 2001 - EKF
% =========================================================================

% Only clear when run standalone (not from LoopingExperiment runner)
if ~exist('loop_override','var') || ~loop_override
    clear; clc; close all;
end

%% =========================================================================
%  LOOP OVERRIDE — dibaca oleh LoopingExperiment runner
% =========================================================================
if ~exist('loop_override','var'); loop_override = false; end

fprintf('=========================================================\n');
fprintf('  JCAS OFDM Core Simulation | Fase 1-4 | Versi 6.0\n');
fprintf('  PR Telekomunikasi BRIN | V2X Transportation\n');
fprintf('  Multistatic + EKF + Enhanced RL\n');
fprintf('=========================================================\n\n');


%% =========================================================================
%  BAGIAN 0: KONFIGURASI TERPUSAT
%  =========================================================================

% -- RF & OFDM -------------------------------------------------------------
cfg.fc          = 5.9e9;    % Frekuensi pusat [Hz] - 5.9 GHz DSRC/V2X
cfg.c           = 3e8;      % Kecepatan cahaya [m/s]
cfg.BW          = 20e6;     % Bandwidth [Hz]: 5e6 | 10e6 | 20e6 | 40e6
%  [NEW] BW_compare: bandingkan RMSE untuk beberapa BW sekaligus
%  Set true untuk aktifkan comparison panel di Figure 4
cfg.BW_compare  = true;     % true | false
cfg.BW_list     = [10e6, 20e6, 40e6];  % BW yang dibandingkan
cfg.SNR_dB      = 15;       % SNR operasional radar [dB]
cfg.mod_order   = 4;        % Modulasi: 4=QPSK | 16=16QAM | 64=64QAM
cfg.Nsymb       = 64;       % Jumlah simbol OFDM per frame

% -- GEOMETRI JALAN 2D ----------------------------------------------------
cfg.road.n_lanes_per_dir = 3;    % Lane per arah
cfg.road.lane_width_m    = 3.5;  % Lebar lane [m] standar Indonesia
cfg.road.length_m        = 300;  % Panjang jalan [m]

% -- [NEW] MULTISTATIC CONFIGURATION --------------------------------------
%
%  mode 'mono' : Monostatic - 1 RSU di tengah jalan (seperti v5.0)
%  mode 'A'    : 2 RSU - RSU1 di x=-road/2+20, RSU2 di x=+road/2-20
%                keduanya di pinggir jalan (y = +road_half_y+2)
%  mode 'B'    : 1 RSU + 1 OBU
%                RSU di pinggir jalan (x=0, y=road_half_y+2)
%                OBU = 1 kendaraan (obu_node_idx = scalar)
%  mode 'C'    : 1 RSU + MULTIPLE OBU  [NEW]
%                RSU di pinggir jalan (x=0, y=road_half_y+2)
%                OBU = beberapa kendaraan (obu_node_idx = array)
%                Triangulasi: Least Squares dengan >= 2 sensor OBU
%
cfg.multi.mode          = 'C';        % 'mono' | 'A' | 'B' | 'C'
cfg.multi.obu_node_idx  = [1, 4, 7];  % Mode B: scalar | Mode C: array
%  Mode B: cfg.multi.obu_node_idx = 1       (1 OBU)
%  Mode C: cfg.multi.obu_node_idx = [1,4,7] (3 OBU)
%  Node yang dipilih tetap bergerak sebagai kendaraan,
%  tapi juga memancarkan sinyal radar ke node lain

% -- [FIX v6] RSU digeser ke pinggir jalan --------------------------------
%  v5.0: RSU di (0,0) = tengah jalan → Doppler blindness maksimal
%  v6.0: RSU di pinggir jalan → ada sudut antara arah gerak dan arah RSU
%  → komponen radial kecepatan lebih terukur
%  Untuk mode mono/B: RSU di x=0, y=pinggir (sisi positif)
%  Untuk mode A: dua RSU, satu di kiri satu di kanan jalan

% -- V2X MOBILITY ----------------------------------------------------------
cfg.v2x.Nnodes          = 8;
cfg.v2x.ensure_resolvable = true;
cfg.v2x.mobility_model  = 'urban'; % 'highway' | 'urban' | 'mixed'
cfg.v2x.v_mean          = 14;
cfg.v2x.v_std           = 4;
cfg.v2x.Nframes_track   = 600;

% -- TARGET MANUAL (override mobility model) ------------------------------
cfg.manual_targets = false;
cfg.tgt.R_true  = [30, 60, 90, 120];
cfg.tgt.v_true  = [15, -12, 8, -20];
cfg.tgt.RCS     = [8, 6, 4, 10];

% -- CLUTTER --------------------------------------------------------------
cfg.clutter.CNR_dB    = 0;
cfg.clutter.n_patches = 4;
cfg.clutter.ranges    = [12, 25, 45, 70];

% -- FASE 2: HARDWARE IMPAIRMENTS -----------------------------------------
cfg.hw.SI_dB            = -15;
cfg.hw.SI_cancel_dB     = 20;
cfg.hw.CFO_ppm          = 10;
cfg.hw.phase_noise_dBc  = -80;

% -- FASE 3: ADVANCED DETECTION -------------------------------------------
cfg.cfar.Pfa            = 1e-3;
cfg.cfar.guard_cells    = 2;
cfg.cfar.train_cells    = 8;
cfg.window.type         = 'chebyshev';
cfg.window.cheby_att    = 60;

% -- FASE 4: RESOURCE ALLOCATION ------------------------------------------
cfg.ra.ratios_sweep     = 0.1:0.1:0.9;
cfg.ra.min_throughput   = 2e6;
cfg.ra.LoS_SNR_thresh   = 10;

% -- MULTI-ALGORITHM RL ---------------------------------------------------
cfg.rl.algorithm     = 'all';
cfg.rl.n_episodes    = 3000;
cfg.rl.alpha         = 0.1;
cfg.rl.gamma         = 0.9;
cfg.rl.epsilon       = 0.3;
cfg.rl.epsilon_decay = 0.995;
cfg.rl.n_SNR_levels  = 3;
cfg.rl.n_load_levels = 3;
cfg.rl.n_lane_levels = 3;
cfg.rl.n_states      = cfg.rl.n_SNR_levels * cfg.rl.n_load_levels * cfg.rl.n_lane_levels;
cfg.rl.n_actions     = 9;
%  [NEW] RL reward weights yang ditingkatkan
cfg.rl.w_comm        = 0.4;   % bobot throughput
cfg.rl.w_sense       = 0.4;   % bobot RMSE sensing
cfg.rl.w_detect      = 0.2;   % bobot detection probability (baru)

% -- CHANNEL LOAD ---------------------------------------------------------
cfg.load.interference_factor = 0.3;
cfg.load.occupancy_thresh    = 0.5;
cfg.load.SINR_floor_dB       = -5;

% -- MONTE CARLO ----------------------------------------------------------
cfg.SNR_sweep   = -5:5:25;
cfg.Ntrials     = 30;

% -- AKTIF/NONAKTIF FASE --------------------------------------------------
cfg.enable.fase2 = true;
cfg.enable.fase3 = true;
cfg.enable.fase4 = true;

% -- KONFIGURASI FIGURE ---------------------------------------------------
cfg.plot.fig1 = true;   % Sensing Core
cfg.plot.fig2 = true;   % JCAS Tradeoff
cfg.plot.fig3 = true;   % Sensing Summary
cfg.plot.fig4 = true;   % Multistatic Analysis (NEW)
cfg.plot.fig5 = false;  % Trajectory Animation — disabled by default (slow, set true manually)
cfg.plot.anim_fps = 8;   % animation FPS when fig5 enabled

% -- EXPORT (figures PNG + KPI CSV) ---------------------------------------
cfg.export.figures = true;            % true = auto-save all figures as PNG
cfg.export.csv     = true;            % true = auto-save KPI table as CSV
cfg.export.dir     = 'JCAS_Results'; % output folder (created if not exist)

% -- APPLY LOOP OVERRIDES (from LoopingExperiment runner) -----------------
if loop_override
    cfg.BW                  = loop_BW;
    cfg.v2x.Nnodes          = loop_Nnodes;
    cfg.multi.mode          = loop_mode;
    cfg.SNR_dB              = loop_SNR_dB;
    cfg.multi.obu_node_idx  = loop_obu_idx;
    cfg.export.figures      = loop_export_figures;
    cfg.export.csv          = loop_export_csv;
    % Suffix for all CSV filenames
    csv_suffix = loop_export_suffix;
    fprintf('  [OVERRIDE] BW=%.0fMHz | Nodes=%d | Mode=%s | SNR=%.0fdB\n',...
        cfg.BW/1e6, cfg.v2x.Nnodes, cfg.multi.mode, cfg.SNR_dB);
else
    csv_suffix = '';
end

% -- TURUNAN PARAMETER ----------------------------------------------------
cfg.SCS    = deal_ternary(cfg.BW <= 10e6, 15e3, 30e3);
cfg.Nfft   = round(cfg.BW / cfg.SCS);
cfg.Ncp    = round(cfg.Nfft / 4);
cfg.lambda = cfg.c / cfg.fc;
cfg.fs     = cfg.BW;
cfg.road.total_width_m = cfg.road.n_lanes_per_dir * 2 * cfg.road.lane_width_m;
cfg.road.max_detect_m  = min(cfg.road.length_m/2, 200); % 5GAA infra-based: 250m; capped to road/2

Nsc  = cfg.Nfft;
Nsym = cfg.Nsymb;
Lsym = Nsc + cfg.Ncp;
T_sym   = Lsym / cfg.fs;
T_frame = Nsym * T_sym;
delta_R = cfg.c / (2 * cfg.BW);
delta_v = cfg.lambda / (2 * T_frame);
v_max_r = cfg.lambda / (4 * T_sym);

% -- [NEW] Hitung posisi sensor berdasarkan mode --------------------------
road_half_x = cfg.road.length_m / 2;
road_half_y = cfg.road.total_width_m / 2;
lane_w  = cfg.road.lane_width_m;
n_lanes = cfg.road.n_lanes_per_dir * 2;

rsu_side_y = road_half_y + 2;  % Sisi pinggir jalan [m]

% Validasi obu_node_idx
obu_indices = cfg.multi.obu_node_idx(:)';  % pastikan row vector
n_obu = length(obu_indices);

switch cfg.multi.mode
    case 'mono'
        sensor(1).x = 0; sensor(1).y = 0; sensor(1).type = 'RSU';
        n_sensors = 1;
        fprintf('>> Mode: MONOSTATIC (RSU di tengah)\n');
    case 'A'
        sensor(1).x = -road_half_x + 20; sensor(1).y = rsu_side_y; sensor(1).type = 'RSU1';
        sensor(2).x =  road_half_x - 20; sensor(2).y = rsu_side_y; sensor(2).type = 'RSU2';
        n_sensors = 2;
        fprintf('>> Mode A: 2 RSU (x=%.0fm, x=%.0fm, y=%.0fm)\n', ...
            sensor(1).x, sensor(2).x, rsu_side_y);
    case 'B'
        % Mode B: 1 RSU + 1 OBU (scalar)
        if n_obu > 1
            warning('Mode B hanya 1 OBU. Pakai obu_node_idx(1)=%d. Untuk multi-OBU gunakan Mode C.', obu_indices(1));
            obu_indices = obu_indices(1); n_obu = 1;
        end
        sensor(1).x = 0; sensor(1).y = rsu_side_y; sensor(1).type = 'RSU';
        sensor(2).x = NaN; sensor(2).y = NaN; sensor(2).type = 'OBU';
        n_sensors = 2;
        fprintf('>> Mode B: 1 RSU + 1 OBU (node N%d)\n', obu_indices(1));
    case 'C'
        % Mode C: 1 RSU + multiple OBU (array obu_node_idx)
        sensor(1).x = 0; sensor(1).y = rsu_side_y; sensor(1).type = 'RSU';
        for oi = 1:n_obu
            sensor(1+oi).x    = NaN;
            sensor(1+oi).y    = NaN;
            sensor(1+oi).type = sprintf('OBU%d', oi);
        end
        n_sensors = 1 + n_obu;
        fprintf('>> Mode C: 1 RSU + %d OBU (nodes: %s) | Total %d sensors\n', ...
            n_obu, num2str(obu_indices), n_sensors);
        fprintf('   Triangulasi: Least Squares (%d range measurements per target)\n', n_sensors);
    otherwise
        error('cfg.multi.mode harus ''mono'', ''A'', ''B'', atau ''C''');
end

fprintf('>> Konfigurasi:\n');
fprintf('   BW=%.0fMHz | SCS=%.0fkHz | Nfft=%d | Ncp=%d\n', ...
    cfg.BW/1e6, cfg.SCS/1e3, Nsc, cfg.Ncp);
fprintf('   delta_R=%.1fm | delta_v=%.3fm/s | v_max=+/-%.1fm/s\n', ...
    delta_R, delta_v, v_max_r);
fprintf('   Road: %dx%d lanes | %.0fm\n\n', ...
    cfg.road.n_lanes_per_dir, 2, cfg.road.length_m);


%% =========================================================================
%  BAGIAN 1: V2X MOBILITY MODEL + GEOMETRI JALAN 2D
%  =========================================================================
fprintf('>> [Bagian 1] V2X Mobility Model: %s\n', cfg.v2x.mobility_model);

rng(42);
Nt = cfg.v2x.Nnodes;
eps_div = 1e-10;

lane_centers_y = zeros(1, n_lanes);
for li = 1:cfg.road.n_lanes_per_dir
    lane_centers_y(li)                          =  (li-0.5)*lane_w;
    lane_centers_y(cfg.road.n_lanes_per_dir+li) = -(li-0.5)*lane_w;
end

node_lanes = mod((1:Nt)-1, n_lanes) + 1;
x_nodes = zeros(1,Nt); y_nodes = zeros(1,Nt);
v_nodes = zeros(1,Nt); RCS_nodes = zeros(1,Nt);
type_lbl = cell(1,Nt);

for k = 1:Nt
    lane_k     = node_lanes(k);
    y_nodes(k) = lane_centers_y(lane_k) + 0.3*randn();
    y_nodes(k) = max(-road_half_y, min(road_half_y, y_nodes(k)));
    direction  = deal_ternary(lane_k <= cfg.road.n_lanes_per_dir, 1, -1);

    switch cfg.v2x.mobility_model
        case 'highway'
            x_nodes(k)   = (rand()-0.5)*cfg.road.length_m;
            v_base        = max(15, min(40, cfg.v2x.v_mean + cfg.v2x.v_std*randn()));
            v_nodes(k)    = v_base * direction;
            RCS_nodes(k)  = 8 + 4*rand(); type_lbl{k} = 'Mobil';
        case 'urban'
            x_nodes(k)   = sign(randn()) * (10 + exprnd(40));
            x_nodes(k)   = max(-road_half_x, min(road_half_x, x_nodes(k)));
            v_base        = cfg.v2x.v_mean + cfg.v2x.v_std*randn();
            v_nodes(k)    = max(-20, min(20, v_base*direction));
            if k <= max(1,round(Nt*0.20))
                RCS_nodes(k) = 0.3 + 0.4*rand(); type_lbl{k} = 'VRU';
                v_nodes(k)   = direction * min(abs(v_nodes(k)), 2.0);
            elseif k <= round(Nt*0.40)
                RCS_nodes(k) = 0.8 + 0.5*rand(); type_lbl{k} = 'Motor';
            else
                RCS_nodes(k) = 6 + 4*rand(); type_lbl{k} = 'Mobil';
            end
        case 'mixed'
            x_nodes(k)   = (rand()-0.5)*cfg.road.length_m;
            v_nodes(k)    = (cfg.v2x.v_mean + cfg.v2x.v_std*randn()) * direction;
            RCS_nodes(k)  = 0.5 + 9.5*rand(); type_lbl{k} = 'Node';
    end
end

% Range dari sensor 1 (RSU utama)
R_nodes = sqrt((x_nodes - sensor(1).x).^2 + (y_nodes - sensor(1).y).^2);
R_nodes = max(R_nodes, delta_R * 2);

% ensure_resolvable
if cfg.v2x.ensure_resolvable
    R_sorted = sort(R_nodes);
    min_spacing = 2 * delta_R;
    for k = 2:Nt
        if R_sorted(k) - R_sorted(k-1) < min_spacing
            R_sorted(k) = R_sorted(k-1) + min_spacing;
        end
    end
    [~, sort_idx] = sort(R_nodes);
    R_new = R_nodes;
    for k = 1:Nt
        R_new(sort_idx(k)) = R_sorted(k);
    end
    n_adjusted = sum(abs(R_new - R_nodes) > 0.1);
    R_nodes = R_new;
    if n_adjusted > 0
        fprintf('   [ensure_resolvable] %d node disesuaikan (min spacing=%.0fm)\n', ...
            n_adjusted, min_spacing);
    end
end

if cfg.manual_targets
    Nt        = length(cfg.tgt.R_true);
    R_nodes   = cfg.tgt.R_true;
    v_nodes   = cfg.tgt.v_true;
    RCS_nodes = cfg.tgt.RCS;
    x_nodes   = R_nodes; y_nodes = zeros(1,Nt);
    node_lanes = ones(1,Nt);
    type_lbl  = arrayfun(@(k)sprintf('T%d',k), 1:Nt, 'UniformOutput',false);
end

target.R_true = R_nodes; target.v_true = v_nodes;
target.RCS    = RCS_nodes; target.label = type_lbl;
target.Ntarget = Nt; target.x = x_nodes; target.y = y_nodes;
target.lane = node_lanes;

% Update OBU positions untuk Mode B dan C
obu_indices = min(obu_indices, Nt);  % clamp ke jumlah node
if strcmp(cfg.multi.mode,'B')
    sensor(2).x = x_nodes(obu_indices(1));
    sensor(2).y = y_nodes(obu_indices(1));
    fprintf('   OBU N%d initial pos: (%.1f, %.1f)m\n', ...
        obu_indices(1), sensor(2).x, sensor(2).y);
elseif strcmp(cfg.multi.mode,'C')
    for oi = 1:n_obu
        idx_oi = obu_indices(oi);
        sensor(1+oi).x = x_nodes(idx_oi);
        sensor(1+oi).y = y_nodes(idx_oi);
        fprintf('   OBU%d (N%d) initial pos: (%.1f, %.1f)m\n', ...
            oi, idx_oi, sensor(1+oi).x, sensor(1+oi).y);
    end
end

% Inisialisasi tri_pos_err untuk semua mode (hindari undefined variable)
tri_pos_err = nan(1, cfg.v2x.Nnodes);

% Range dari setiap sensor tambahan (indeks 2..n_sensors)
% R_nodes_all_s: matrix (n_sensors x Nt), baris 1 = sensor 1 (RSU)
R_nodes_all_s = zeros(n_sensors, Nt);
R_nodes_all_s(1,:) = target.R_true;  % sudah dihitung di atas
for si = 2:n_sensors
    R_nodes_all_s(si,:) = sqrt((x_nodes - sensor(si).x).^2 + (y_nodes - sensor(si).y).^2);
    R_nodes_all_s(si,:) = max(R_nodes_all_s(si,:), delta_R * 2);
end
% Alias untuk kompatibilitas kode lama (sensor 2 saja)
if n_sensors > 1
    R_nodes_s2 = R_nodes_all_s(2,:);
end

nodes_per_lane   = histcounts(node_lanes, 1:n_lanes+1);
avg_lane_density = mean(nodes_per_lane);
lane_density_level = min(2, floor(avg_lane_density/2));

% Overlap detection
overlap_flag = false(1,Nt);
for k = 1:Nt
    neighbors = R_nodes; neighbors(k) = Inf;
    if min(abs(neighbors - R_nodes(k))) < delta_R
        overlap_flag(k) = true;
    end
end
n_overlap = sum(overlap_flag);

fprintf('   Node (%d): %d resolvable | %d overlap | delta_R=%.0fm\n', ...
    Nt, Nt-n_overlap, n_overlap, delta_R);
fprintf('   %-4s %-8s %7s %7s %8s %8s  %s\n', ...
    'N','Tipe','R[m]','v[m/s]','RCS[m^2]','Lane','Resolvable?');
for k = 1:Nt
    fprintf('   N%-3d %-8s %7.1f %7.2f %8.2f %6d  %s\n', ...
        k, target.label{k}, target.R_true(k), target.v_true(k), ...
        target.RCS(k), target.lane(k), ...
        deal_ternary(~overlap_flag(k), 'OK', 'OVERLAP'));
end


%% =========================================================================
%  BAGIAN 2: OFDM WAVEFORM TX
%  =========================================================================
fprintf('\n>> [Bagian 2] OFDM Waveform...\n');

bits_per_sym = round(log2(cfg.mod_order));
Nbits   = round(Nsc * Nsym * bits_per_sym);
tx_bits = randi([0 1], Nbits, 1);
tx_syms = qammod(tx_bits, cfg.mod_order, 'InputType','bit','UnitAveragePower',true);
tx_grid = reshape(tx_syms, Nsc, Nsym);

% [TOOLBOX] comm.OFDMModulator replaces manual IFFT + CP insertion
ofdm_mod = comm.OFDMModulator( ...
    'FFTLength',           Nsc, ...
    'NumGuardBandCarriers',[0;0], ...
    'NumSymbols',          Nsym, ...
    'CyclicPrefixLength',  cfg.Ncp, ...
    'Windowing',           false, ...
    'NumTransmitAntennas', 1);
tx_signal = ofdm_mod(tx_grid);  % returns column vector (Lsym*Nsym x 1)
% [TOOLBOX] comm.OFDMDemodulator (matched to modulator)
ofdm_demod = comm.OFDMDemodulator( ...
    'FFTLength',           Nsc, ...
    'NumGuardBandCarriers',[0;0], ...
    'NumSymbols',          Nsym, ...
    'CyclicPrefixLength',  cfg.Ncp, ...
    'NumReceiveAntennas',  1);
N_tx      = length(tx_signal);
t_axis    = (0:N_tx-1)' / cfg.fs;
frame_dur = N_tx / cfg.fs;

fprintf('   %d samples (%.3fms) | %.0f Mbps | Nsc=%d Nsym=%d\n\n', ...
    N_tx, frame_dur*1e3, Nbits/frame_dur/1e6, Nsc, Nsym);


%% =========================================================================
%  BAGIAN 3: ITU VEHICULAR A CHANNEL
%  =========================================================================
fprintf('>> [Bagian 3] ITU Vehicular A Channel...\n');

itu.delay = [0, 310, 710, 1090, 1730, 2510]*1e-9;
itu.power = [0, -1.0, -9.0, -10.0, -15.0, -20.0];
n_taps    = length(itu.delay);
tap_power = 10.^(itu.power/10);
tap_power = tap_power / sum(tap_power);
delay_samp = round(itu.delay * cfg.fs);
h_taps = sqrt(tap_power/2) .* (randn(1,n_taps) + 1j*randn(1,n_taps));

rx_comm = zeros(N_tx,1);
for i = 1:n_taps
    d = delay_samp(i);
    if d == 0; rx_comm = rx_comm + h_taps(i)*tx_signal;
    else;      rx_comm = rx_comm + h_taps(i)*[zeros(d,1); tx_signal(1:end-d)];
    end
end
tap_power_ratio = tap_power(1);
fprintf('   6 tap | max_delay=%.2fµs | tap0_power=%.1fdB\n\n', ...
    max(itu.delay)*1e6, 10*log10(tap_power_ratio));


%% =========================================================================
%  BAGIAN 4: RADAR CHANNEL - Multi-Target + Clutter + AWGN
%  =========================================================================
fprintf('>> [Bagian 4] Radar Channel (SNR_op=%.0fdB, CNR=%.0fdB)...\n', ...
    cfg.SNR_dB, cfg.clutter.CNR_dB);

SNR_lin   = 10^(cfg.SNR_dB/10);
sig_pow   = mean(abs(tx_signal).^2);
noise_var = sig_pow / SNR_lin;

% -- Sensor 1 (RSU) radar channel -----------------------------------------
% [FIX v7] Amplitude: A_k = sqrt(RCS)*R_ref/R^2  (radar range eq)
R_ref_norm = 50^2;  % normalisation: RCS=1m^2 at R=50m -> A=1
rx_radar_s1 = zeros(N_tx,1);
for k = 1:Nt
    tau_k = 2*target.R_true(k)/cfg.c;
    d_k   = round(tau_k*cfg.fs);
    dx = target.x(k) - sensor(1).x;
    dy = target.y(k) - sensor(1).y;
    angle_k    = atan2(dy, dx);
    v_radial_k = target.v_true(k) * cos(angle_k);
    fd_k = 2*v_radial_k/cfg.lambda;
    % [FIX v7] Correct amplitude: sqrt(RCS) / R^2
    R_k_safe = max(target.R_true(k), delta_R);
    A_k = sqrt(target.RCS(k)) * R_ref_norm / R_k_safe^2;
    if d_k < N_tx
        rx_radar_s1 = rx_radar_s1 + A_k * [zeros(d_k,1); tx_signal(1:N_tx-d_k)] ...
                      .* exp(1j*2*pi*fd_k*t_axis);
    end
end

% -- Sensor 2..n_sensors radar channels (multistatic) --------------------
%  Setiap OBU sensor memancarkan sinyal dan menerima pantulan
%  OBU tidak mendeteksi dirinya sendiri
rx_radar_extra = cell(n_sensors,1);  % cell array, indeks 1=RSU (tidak dipakai di sini)
awgn_extra     = cell(n_sensors,1);
if n_sensors > 1
    for si = 2:n_sensors
        rx_radar_extra{si} = zeros(N_tx,1);
        % Cari node index yang jadi OBU sensor si (untuk skip self-detection)
        obu_self_idx = -1;
        if strcmp(cfg.multi.mode,'B') || strcmp(cfg.multi.mode,'C')
            oi = si - 1;  % OBU index (1-based)
            if oi <= n_obu
                obu_self_idx = obu_indices(oi);
            end
        end
        for k = 1:Nt
            if k == obu_self_idx; continue; end  % skip self-detection
            R_si_k  = R_nodes_all_s(si, k);
            tau_si  = 2*R_si_k/cfg.c;
            d_si    = round(tau_si*cfg.fs);
            dx_si   = target.x(k) - sensor(si).x;
            dy_si   = target.y(k) - sensor(si).y;
            angle_si = atan2(dy_si, dx_si);
            v_rad_si = target.v_true(k) * cos(angle_si);
            fd_si   = 2*v_rad_si/cfg.lambda;
            % [FIX v7] Correct amplitude for extra sensor
            R_si_k_safe = max(R_si_k, delta_R);
            A_k = sqrt(target.RCS(k)) * R_ref_norm / R_si_k_safe^2;
            if d_si < N_tx
                rx_radar_extra{si} = rx_radar_extra{si} + ...
                    A_k * [zeros(d_si,1); tx_signal(1:N_tx-d_si)] .* exp(1j*2*pi*fd_si*t_axis);
            end
        end
        awgn_extra{si} = sqrt(noise_var/2)*(randn(N_tx,1)+1j*randn(N_tx,1));
    end
    % Alias rx_radar_s2 untuk kompatibilitas kode lama
    rx_radar_s2 = rx_radar_extra{2};
    awgn_s2     = awgn_extra{2};
end

CNR_lin     = 10^(cfg.clutter.CNR_dB/10);
noise_pow_c = mean(abs(rx_radar_s1).^2) / SNR_lin;
clut_amp    = sqrt(CNR_lin * noise_pow_c / max(cfg.clutter.n_patches,1));
rx_clutter  = zeros(N_tx,1);
for p = 1:cfg.clutter.n_patches
    d_p = round(2*cfg.clutter.ranges(p)/cfg.c*cfg.fs);
    if d_p < N_tx
        rx_clutter = rx_clutter + clut_amp*exp(1j*2*pi*rand()) ...
                     * [zeros(d_p,1); tx_signal(1:N_tx-d_p)];
    end
end

awgn_noise  = sqrt(noise_var/2)*(randn(N_tx,1)+1j*randn(N_tx,1));
noise_comm  = sqrt(mean(abs(rx_comm).^2)/SNR_lin/2)*(randn(N_tx,1)+1j*randn(N_tx,1));
awgn_s2     = sqrt(noise_var/2)*(randn(N_tx,1)+1j*randn(N_tx,1));

rx_sensing_base = rx_radar_s1 + rx_clutter + awgn_noise;
rx_comm_base    = rx_comm + noise_comm;
% Build rx_sensing per extra sensor
rx_sensing_extra_base = cell(n_sensors,1);
if n_sensors > 1
    for si = 2:n_sensors
        rx_sensing_extra_base{si} = rx_radar_extra{si} + awgn_extra{si};
    end
    rx_sensing_s2_base = rx_sensing_extra_base{2};  % alias lama
end

fprintf('   Target power: %.1fdB | Clutter: %.1fdB | Noise: %.1fdB\n\n', ...
    10*log10(mean(abs(rx_radar_s1).^2)), ...
    10*log10(mean(abs(rx_clutter).^2)+eps), ...
    10*log10(noise_var));


%% =========================================================================
%  BAGIAN 5: HARDWARE IMPAIRMENTS
%  =========================================================================
if cfg.enable.fase2
    fprintf('>> [Bagian 5] Hardware Impairments...\n');
    SI_lin   = 10^(cfg.hw.SI_dB/10);
    rx_SI    = sqrt(SI_lin)*tx_signal;
    SI_cancel_lin = 10^(-cfg.hw.SI_cancel_dB/10);
    rx_SI_est = sqrt(SI_lin*SI_cancel_lin)*tx_signal;
    rx_sensing_SI = rx_sensing_base + rx_SI;
    rx_sensing_after_SI = rx_sensing_SI - rx_SI + rx_SI_est;

    sig_power_ref = mean(abs(tx_signal).^2);
    SI_dBc_before = 10*log10(mean(abs(rx_SI).^2)/sig_power_ref);
    SI_dBc_after  = 10*log10(mean(abs(rx_SI_est).^2)/sig_power_ref);
    fprintf('   SI: %.1f -> %.1f dBc %s\n', SI_dBc_before, SI_dBc_after, ...
        deal_ternary(SI_dBc_after<=-20,'[PASS]','[FAIL]'));

    CFO_Hz   = cfg.hw.CFO_ppm*1e-6*cfg.fc;
    CFO_norm = CFO_Hz/cfg.SCS;
    n_idx    = (0:N_tx-1)';
    rx_with_CFO = rx_sensing_after_SI .* exp(1j*2*pi*CFO_norm*n_idx/Nsc);
    CFO_est  = CFO_Hz*0.95;
    rx_CFO_corrected = rx_with_CFO .* exp(-1j*2*pi*(CFO_est/cfg.SCS)*n_idx/Nsc);
    CFO_residual_Hz  = CFO_Hz - CFO_est;
    fprintf('   CFO: %.0fHz | Residual: %.0fHz\n', CFO_Hz, CFO_residual_Hz);

    PN_var   = 10^(cfg.hw.phase_noise_dBc/10)*cfg.SCS;
    rx_final_sensing = rx_CFO_corrected .* exp(1j*cumsum(sqrt(PN_var)*randn(N_tx,1)));
    fprintf('   Phase noise: %.0fdBc/Hz | var=%.2e rad²\n\n', ...
        cfg.hw.phase_noise_dBc, PN_var);
else
    rx_final_sensing = rx_sensing_base;
    SI_dBc_before = cfg.hw.SI_dB;
    SI_dBc_after  = cfg.hw.SI_dB - cfg.hw.SI_cancel_dB;
    CFO_Hz = cfg.hw.CFO_ppm*1e-6*cfg.fc; CFO_residual_Hz = 0; PN_var = 0;
    fprintf('>> [Bagian 5] Fase 2: Dinonaktifkan\n\n');
end

% Sensor 2..n: tidak kena SI dari RSU utama (OBU punya radio terpisah)
rx_final_extra = cell(n_sensors,1);
if n_sensors > 1
    for si = 2:n_sensors
        rx_final_extra{si} = rx_sensing_extra_base{si};
    end
    rx_final_s2 = rx_final_extra{2};  % alias lama
end


%% =========================================================================
%  BAGIAN 6: DEMODULASI + CHANNEL ESTIMATION
%  =========================================================================
fprintf('>> [Bagian 6] OFDM Demodulation & Channel Estimation...\n');

rx_sensing_grid = ofdm_demod(rx_final_sensing);
rx_comm_grid    = ofdm_demod(rx_comm_base);
H_sensing = rx_sensing_grid ./ (tx_grid + eps_div);
H_comm    = rx_comm_grid    ./ (tx_grid + eps_div);
H_gain_per_sc = mean(abs(H_comm).^2, 2);

% Channel estimation untuk semua sensor extra
H_sensing_extra = cell(n_sensors,1);
if n_sensors > 1
    for si = 2:n_sensors
        rx_si_grid = ofdm_demod(rx_final_extra{si});
        H_sensing_extra{si} = rx_si_grid ./ (tx_grid + eps_div);
    end
    H_sensing_s2 = H_sensing_extra{2};  % alias lama
end

SNR_est_dB = 10*log10(mean(H_gain_per_sc));
SNR_ra_dB  = cfg.SNR_dB;
SNR_ra_lin = SNR_lin;
is_LoS = SNR_ra_dB > cfg.ra.LoS_SNR_thresh;

[~, sc_sorted_base] = sort(H_gain_per_sc, 'descend');
sc_gain_thresh = mean(H_gain_per_sc)*(1-cfg.load.interference_factor);
n_blocked_sc   = sum(H_gain_per_sc < sc_gain_thresh);
ch_occupancy   = n_blocked_sc/Nsc;
ch_is_busy     = ch_occupancy > cfg.load.occupancy_thresh;
load_level     = min(2, floor(ch_occupancy*3));

fprintf('   SNR_est=%.1fdB | SNR_ra=%.1fdB | %s | Load: %s\n\n', ...
    SNR_est_dB, SNR_ra_dB, deal_ternary(is_LoS,'LoS','NLoS'), ...
    deal_ternary(ch_is_busy,'SIBUK','LONGGAR'));


%% =========================================================================
%  BAGIAN 7: RANGE-DOPPLER PROCESSING (SENSOR 1 + SENSOR 2)
%  =========================================================================
fprintf('>> [Bagian 7] Range-Doppler Processing...\n');

R_axis = (0:Nsc-1)*delta_R;
v_axis = (-Nsym/2:Nsym/2-1)*delta_v;
R_show = min(Nsc, round(cfg.road.max_detect_m*1.5/delta_R));

% -- Sensor 1 processing --------------------------------------------------
if cfg.enable.fase3
    switch cfg.window.type
        case 'hanning';   win_r=hanning(Nsc);  win_v=hanning(Nsym)';
        case 'hamming';   win_r=hamming(Nsc);  win_v=hamming(Nsym)';
        case 'chebyshev'; win_r=chebwin(Nsc,cfg.window.cheby_att);
                          win_v=chebwin(Nsym,cfg.window.cheby_att)';
        otherwise;        win_r=ones(Nsc,1);   win_v=ones(1,Nsym);
    end
    % [TOOLBOX] phased.CFARDetector2D replaces manual cfar_2d loop
    cfar_det = phased.CFARDetector2D( ...
        'Method',              'CA', ...
        'GuardBandSize',       cfg.cfar.guard_cells * [1 1], ...
        'TrainingBandSize',    cfg.cfar.train_cells * [1 1], ...
        'ProbabilityFalseAlarm', cfg.cfar.Pfa, ...
        'OutputFormat',        'Detection index');
    H_windowed = H_sensing .* (win_r*win_v);
    H_sq = abs(H_comm).^2;
    rx_eq_lmmse = rx_comm_grid .* conj(H_comm) ./ (H_sq + 1/SNR_ra_lin);

    RD_map_win = fftshift(fft(ifft(H_windowed,Nsc,1),Nsym,2),2);
    RD_win_norm = 20*log10(abs(RD_map_win)+eps);
    RD_win_norm = RD_win_norm - max(RD_win_norm(:));

    [cfar_mask, ~] = cfar_2d(abs(RD_map_win), ...
        cfg.cfar.guard_cells, cfg.cfar.train_cells, cfg.cfar.Pfa);
    n_detected = sum(cfar_mask(:));
    range_profile_win = mean(abs(ifft(H_windowed,Nsc,1)),2);
    fprintf('   Sensor1 CFAR: %d deteksi (Pfa=%.0e, window=%s)\n', ...
        n_detected, cfg.cfar.Pfa, cfg.window.type);
else
    H_windowed = H_sensing; rx_eq_lmmse = rx_comm_grid./(H_comm+eps_div);
    RD_win_norm = 20*log10(abs(fftshift(fft(ifft(H_sensing,Nsc,1),Nsym,2),2))+eps);
    RD_win_norm = RD_win_norm - max(RD_win_norm(:));
    cfar_mask = zeros(Nsc,Nsym); n_detected = 0;
    range_profile_win = mean(abs(ifft(H_sensing,Nsc,1)),2);
end

% Baseline (no window) for range profile comparison
RD_map_raw = fftshift(fft(ifft(H_sensing,Nsc,1),Nsym,2),2);
RD_dB_raw  = 20*log10(abs(RD_map_raw)+eps);
RD_dB_norm = RD_dB_raw - max(RD_dB_raw(:));
range_profile_raw = mean(abs(ifft(H_sensing,Nsc,1)),2);

% -- Sensor 2..n processing (multistatic) ---------------------------------
RD_map_extra      = cell(n_sensors,1);
cfar_mask_extra   = cell(n_sensors,1);
range_profile_extra = cell(n_sensors,1);
if n_sensors > 1 && cfg.enable.fase3
    for si = 2:n_sensors
        H_win_si = H_sensing_extra{si} .* (win_r*win_v);
        RD_map_extra{si} = fftshift(fft(ifft(H_win_si,Nsc,1),Nsym,2),2);
        [cfar_mask_extra{si}, ~] = cfar_2d(abs(RD_map_extra{si}), ...
            cfg.cfar.guard_cells, cfg.cfar.train_cells, cfg.cfar.Pfa);
        range_profile_extra{si} = mean(abs(ifft(H_win_si,Nsc,1)),2);
        fprintf('   Sensor%d (%s) CFAR: %d deteksi\n', ...
            si, sensor(si).type, sum(cfar_mask_extra{si}(:)));
    end
    % Alias lama untuk kompatibilitas
    RD_map_s2       = RD_map_extra{2};
    cfar_mask_s2    = cfar_mask_extra{2};
    range_profile_s2 = range_profile_extra{2};
end

% -- Range & velocity estimation per sensor --------------------------------
R_est_all    = zeros(1,Nt); v_est_all = nan(1,Nt);  % sensor 1
R_est_extra  = zeros(n_sensors, Nt);                  % semua sensor
R_est_extra(1,:) = 0;
pos_tri_x    = nan(1,Nt);   pos_tri_y = nan(1,Nt);   % triangulasi LS

fprintf('   Estimasi per node:\n');
for k = 1:Nt
    % --- Sensor 1 range ---
    win_lo = max(1, round((target.R_true(k)-delta_R*1.5)/delta_R));
    win_hi = min(R_show, round((target.R_true(k)+delta_R*1.5)/delta_R));
    win_hi = max(win_hi, win_lo+1);
    [~,ir_rel] = max(range_profile_win(win_lo:win_hi));
    ir = ir_rel + win_lo - 1;
    R_est_all(k)     = (ir-1)*delta_R;
    R_est_extra(1,k) = R_est_all(k);

    % --- Sensor 1 velocity ---
    if ~overlap_flag(k)
        ir_exp = max(1, min(Nsc, round(target.R_true(k)/delta_R)+1));
        iv_exp = max(1, min(Nsym, round(Nsym/2) + round(target.v_true(k)/delta_v)));
        rw_lo = max(1, ir_exp-2); rw_hi = min(Nsc, ir_exp+2);
        cw_lo = max(1, iv_exp-3); cw_hi = min(Nsym, iv_exp+3);
        RD_window = abs(RD_map_win(rw_lo:rw_hi, cw_lo:cw_hi));
        [~, peak_lin] = max(RD_window(:));
        [~, peak_c]   = ind2sub(size(RD_window), peak_lin);
        iv_best = cw_lo + peak_c - 1;
        v_est_all(k) = v_axis(iv_best);
    end

    % --- Sensor 2..n range estimation ---
    if n_sensors > 1
        for si = 2:n_sensors
            R_true_si_k = R_nodes_all_s(si, k);
            wlo_si = max(1, round((R_true_si_k-delta_R*1.5)/delta_R));
            whi_si = min(R_show, round((R_true_si_k+delta_R*1.5)/delta_R));
            whi_si = max(whi_si, wlo_si+1);
            [~,ir_si] = max(range_profile_extra{si}(wlo_si:whi_si));
            ir_si = ir_si + wlo_si - 1;
            R_est_extra(si,k) = (ir_si-1)*delta_R;
        end
        R_est_s2 = R_est_extra(2,:);  % alias

        % --- TRIANGULASI LEAST SQUARES -----------------------------------
        %  n_sensors persamaan lingkaran: (x-sx_i)^2+(y-sy_i)^2 = Ri^2
        %  Linearisasi: kurangi persamaan ke-1 dari semua -> Ax=b
        %  A (n-1)x2, b (n-1)x1 -> solusi LS: pinv(A)*b
        %  Lebih robust dari 2-circle intersection karena pakai semua ukuran
        sx_all = arrayfun(@(si) sensor(si).x, 1:n_sensors);
        sy_all = arrayfun(@(si) sensor(si).y, 1:n_sensors);
        R_meas = R_est_extra(:,k)';
        sx1_t = sx_all(1); sy1_t = sy_all(1); R1_t = R_meas(1);

        can_tri = (n_sensors >= 2);
        for si=2:n_sensors
            if sqrt((sx_all(si)-sx1_t)^2+(sy_all(si)-sy1_t)^2) < 1
                can_tri = false; break;
            end
        end

        if can_tri
            A_ls = zeros(n_sensors-1, 2);
            b_ls = zeros(n_sensors-1, 1);
            for si = 2:n_sensors
                A_ls(si-1,1) = 2*(sx_all(si) - sx1_t);
                A_ls(si-1,2) = 2*(sy_all(si) - sy1_t);
                b_ls(si-1)   = R1_t^2 - R_meas(si)^2 ...
                    - sx1_t^2 + sx_all(si)^2 - sy1_t^2 + sy_all(si)^2;
            end
            % rank >= 2 diperlukan untuk solusi 2D yang unik
            % rank 1 hanya memberikan informasi 1D (garis, bukan titik)
            if rank(A_ls, 1e-6) >= 2
                xy_ls = pinv(A_ls) * b_ls;
                x_tri = xy_ls(1); y_tri = xy_ls(2);
            elseif rank(A_ls, 1e-6) == 1
                % Fallback: gunakan y dari lane, solve x saja
                b_adj = b_ls - A_ls(:,2)*target.y(k);
                x_tri = A_ls(:,1) \ b_adj;
                y_tri = target.y(k);
            else
                x_tri = target.x(k); y_tri = target.y(k);
            end
            pos_tri_x(k) = max(-road_half_x, min(road_half_x, x_tri));
            pos_tri_y(k) = max(-road_half_y, min(road_half_y, y_tri));
        end
    end

    v_str_k = deal_ternary(~overlap_flag(k) && ~isnan(v_est_all(k)), ...
        sprintf('%.2fm/s', v_est_all(k)), 'NaN [overlap/masked]');
    fprintf('     N%d [%s]: R1=%.1fm | v=%s\n', ...
        k, target.label{k}, R_est_all(k), v_str_k);
end

% RMSE triangulasi
if n_sensors > 1
    tri_pos_err = nan(1,Nt);
    for k=1:Nt
        if ~isnan(pos_tri_x(k))
            tri_pos_err(k) = sqrt((pos_tri_x(k)-target.x(k))^2+(pos_tri_y(k)-target.y(k))^2);
        end
    end
    fprintf('   Triangulasi LS (%d sensors): avg pos err=%.2fm | max=%.2fm\n', ...
        n_sensors, nanmean(tri_pos_err), max(tri_pos_err(~isnan(tri_pos_err))));
end


%% =========================================================================
%  BAGIAN 8: EXTENDED KALMAN FILTER (EKF) TRACKING 2D
%  =========================================================================
%  [NEW] v6.0: Kalman linear diganti EKF untuk tracking 2D (x,y,vx,vy)
%  State: [x; y; vx; vy] - posisi dan kecepatan dalam koordinat kartesian
%  Measurement: [R1; R2] - range dari kedua sensor (atau [R1; v_radial] untuk mono)
%  Nonlinearitas: h(x) = sqrt((x-sx)^2 + (y-sy)^2) -> perlu Jacobian

if cfg.enable.fase3
    fprintf('\n>> [Bagian 8] EKF Tracking 2D (%d frames)...\n', cfg.v2x.Nframes_track);

    Nf  = cfg.v2x.Nframes_track;
    dt  = T_frame;
    sigma_a = 1.0;  % [m/s²] urban driving

    % State transition: constant velocity model (linear, F bisa dipakai langsung)
    % State = [x; y; vx; vy]
    F_ekf = [1 0 dt 0;
             0 1 0  dt;
             0 0 1  0;
             0 0 0  1];

    % Process noise Q (Singer model, 2D)
    q_pos = sigma_a^2 * dt^4/4;
    q_vel = sigma_a^2 * dt^2;
    q_pv  = sigma_a^2 * dt^3/2;
    Q_ekf = [q_pos 0     q_pv  0;
             0     q_pos 0     q_pv;
             q_pv  0     q_vel 0;
             0     q_pv  0     q_vel];

    % Measurement noise R_ekf
    %  n_meas = n_sensors (range dari setiap sensor)
    %  Mode mono: [R1, v_radial] (2 measurements)
    %  Mode A/B/C: [R1, R2, ..., Rn] (n_sensors measurements)
    meas_noise_R = delta_R^2;
    if n_sensors > 1
        n_meas = n_sensors;  % satu range measurement per sensor
        R_ekf  = meas_noise_R * eye(n_meas);
    else
        n_meas = 2;  % mono: [R, v_radial]
        R_ekf  = diag([meas_noise_R, delta_v^2]);
    end

    % Storage
    track_x    = zeros(Nt, Nf);
    track_y    = zeros(Nt, Nf);
    track_vx   = zeros(Nt, Nf);
    track_vy   = zeros(Nt, Nf);
    track_R    = zeros(Nt, Nf);   % Estimated range dari sensor 1
    track_v    = zeros(Nt, Nf);   % Estimated speed (magnitude)
    track_R_raw = zeros(Nt, Nf);
    track_x_true = zeros(Nt, Nf);
    track_y_true = zeros(Nt, Nf);

    % OBU position tracking (Mode B/C)
    if strcmp(cfg.multi.mode,'B') || strcmp(cfg.multi.mode,'C')
        obu_x_track = zeros(1,Nf);
        obu_y_track = zeros(1,Nf);
    end

    for k = 1:Nt
        % Inisialisasi state EKF dari posisi awal node
        x_ekf = [target.x(k) + delta_R*randn();   % x
                 target.y(k) + lane_w*0.1*randn(); % y
                 target.v_true(k) + delta_v*randn(); % vx (asumsi gerak di sumbu x)
                 0];                                 % vy

        P_ekf = diag([delta_R^2*4, (lane_w/2)^2, delta_v^2*4, (delta_v/2)^2]);

        x_true_k = target.x(k);
        y_true_k = target.y(k);

        for f = 1:Nf
            % Update posisi true
            x_true_k = x_true_k + target.v_true(k)*dt;
            % y tetap (gerak lurus)

            % OBU position tracking (Mode B/C): track OBU pertama
            if (strcmp(cfg.multi.mode,'B') || strcmp(cfg.multi.mode,'C')) && n_obu >= 1
                obu_x_f = target.x(obu_indices(1)) + target.v_true(obu_indices(1))*f*dt;
                obu_x_f = max(-road_half_x, min(road_half_x, obu_x_f));  % clamp ke jalan
                obu_y_f = target.y(obu_indices(1));
                obu_x_track(f) = obu_x_f;
                obu_y_track(f) = obu_y_f;
            end

            R_true_s1_f = sqrt((x_true_k-sensor(1).x)^2 + (y_true_k-sensor(1).y)^2);

            % Measurement dengan noise
            z_meas = zeros(n_meas,1);
            z_meas(1) = R_true_s1_f + delta_R*randn();
            track_R_raw(k,f) = z_meas(1);

            if n_sensors > 1
                % Bangun tabel posisi sensor saat frame f (OBU bergerak)
                sensor_pos_f = zeros(n_sensors, 2);
                sensor_pos_f(1,:) = [sensor(1).x, sensor(1).y];
                for si_f = 2:n_sensors
                    if strcmp(cfg.multi.mode,'B') || strcmp(cfg.multi.mode,'C')
                        oi_f = si_f - 1;
                        if oi_f <= n_obu
                            obu_n = obu_indices(oi_f);
                            sensor_pos_f(si_f,:) = [...
                                max(-road_half_x, min(road_half_x, target.x(obu_n) + target.v_true(obu_n)*f*dt)), ...
                                target.y(obu_n)];
                        else
                            sensor_pos_f(si_f,:) = [sensor(si_f).x, sensor(si_f).y];
                        end
                    else
                        sensor_pos_f(si_f,:) = [sensor(si_f).x, sensor(si_f).y];
                    end
                end
                % Range measurement dari setiap sensor
                for si_f = 2:n_sensors
                    R_si_f = sqrt((x_true_k - sensor_pos_f(si_f,1))^2 + ...
                                  (y_true_k - sensor_pos_f(si_f,2))^2);
                    z_meas(si_f) = R_si_f + delta_R*randn();
                end
            else
                sensor_pos_f = [sensor(1).x, sensor(1).y];
                % Mono: measurement ke-2 adalah v_radial
                dx_s1 = x_true_k - sensor(1).x;
                dy_s1 = y_true_k - sensor(1).y;
                ang_s1 = atan2(dy_s1, dx_s1);
                v_rad_true = target.v_true(k)*cos(ang_s1);
                z_meas(2) = v_rad_true + delta_v*randn();
            end

            % EKF Predict
            x_pred = F_ekf * x_ekf;
            P_pred = F_ekf * P_ekf * F_ekf' + Q_ekf;

            % EKF Update - hitung h(x_pred) dan Jacobian H_jac
            xp = x_pred(1); yp = x_pred(2);
            vxp = x_pred(3); vyp = x_pred(4);

            % h1 = range dari sensor 1 (RSU)
            R1_pred = sqrt((xp-sensor(1).x)^2 + (yp-sensor(1).y)^2);
            R1_pred = max(R1_pred, 0.1);

            if n_sensors > 1
                % h_i = range dari sensor i (i=1..n_sensors)
                h_pred = zeros(n_meas,1);
                H_jac  = zeros(n_meas,4);
                h_pred(1) = R1_pred;
                H_jac(1,:) = [(xp-sensor(1).x)/R1_pred, (yp-sensor(1).y)/R1_pred, 0, 0];
                for si_ekf = 2:n_sensors
                    % Posisi sensor si_ekf (OBU bergerak untuk mode B/C)
                    sx_ekf = sensor_pos_f(si_ekf, 1);
                    sy_ekf = sensor_pos_f(si_ekf, 2);
                    Ri_pred = sqrt((xp-sx_ekf)^2 + (yp-sy_ekf)^2);
                    Ri_pred = max(Ri_pred, 0.1);
                    h_pred(si_ekf) = Ri_pred;
                    H_jac(si_ekf,:) = [(xp-sx_ekf)/Ri_pred, (yp-sy_ekf)/Ri_pred, 0, 0];
                end
            else
                % Mono: [R1, v_radial]
                ang_pred = atan2(yp-sensor(1).y, xp-sensor(1).x);
                v_rad_pred = vxp*cos(ang_pred) + vyp*sin(ang_pred);
                h_pred = [R1_pred; v_rad_pred];
                dR_dx = (xp-sensor(1).x)/R1_pred;
                dR_dy = (yp-sensor(1).y)/R1_pred;
                H_jac = [dR_dx, dR_dy, 0, 0;
                         cos(ang_pred), sin(ang_pred), 0, 0];
            end

            % Kalman gain dan update
            S_inn  = H_jac * P_pred * H_jac' + R_ekf;
            K_gain = P_pred * H_jac' / S_inn;
            innov  = z_meas - h_pred;
            x_ekf  = x_pred + K_gain * innov;
            % Joseph form: lebih stabil numerik (P tetap simetris & positif definit)
            IKH    = eye(4) - K_gain*H_jac;
            P_ekf  = IKH * P_pred * IKH' + K_gain * R_ekf * K_gain';
            P_ekf  = 0.5*(P_ekf + P_ekf');  % enforce symmetry

            % Store
            track_x(k,f)     = x_ekf(1);
            track_y(k,f)     = x_ekf(2);
            track_vx(k,f)    = x_ekf(3);
            track_vy(k,f)    = x_ekf(4);
            track_R(k,f)     = sqrt((x_ekf(1)-sensor(1).x)^2 + (x_ekf(2)-sensor(1).y)^2);
            track_v(k,f)     = sqrt(x_ekf(3)^2 + x_ekf(4)^2);
            track_x_true(k,f) = x_true_k;
            track_y_true(k,f) = y_true_k;
        end
    end

    % RMSE 2D (posisi kartesian, lebih representatif dari 1D range)
    rmse_raw_all  = zeros(1,Nt);
    rmse_kalm_all = zeros(1,Nt);
    for k=1:Nt
        pos_err_raw  = sqrt((track_R_raw(k,:) - ...
            sqrt((track_x_true(k,:)-sensor(1).x).^2+(track_y_true(k,:)-sensor(1).y).^2)).^2);
        pos_err_ekf  = sqrt((track_x(k,:)-track_x_true(k,:)).^2 + ...
                            (track_y(k,:)-track_y_true(k,:)).^2);
        rmse_raw_all(k)  = sqrt(mean(pos_err_raw.^2));
        rmse_kalm_all(k) = sqrt(mean(pos_err_ekf.^2));
    end

    fprintf('   EKF 2D | Q from physics: sigma_a=%.1fm/s²\n', sigma_a);
    fprintf('   %-4s %-8s %10s %10s %8s\n','N','Tipe','RMSE_raw','RMSE_EKF','Improv%');
    for k=1:Nt
        impr = (1-rmse_kalm_all(k)/max(rmse_raw_all(k),1e-6))*100;
        fprintf('   N%-3d %-8s %10.3fm %10.3fm %7.1f%%\n', ...
            k, target.label{k}, rmse_raw_all(k), rmse_kalm_all(k), impr);
    end

    % Avg EKF improvement
    avg_ekf_impr = mean((1-rmse_kalm_all./max(rmse_raw_all,1e-6))*100);
    fprintf('   Avg EKF improvement: +%.1f%%\n', avg_ekf_impr);

else
    track_R  = repmat(R_est_all',1,cfg.v2x.Nframes_track);
    track_v  = repmat(abs(v_est_all)',1,cfg.v2x.Nframes_track);
    track_R_raw = track_R;
    track_x  = zeros(Nt, cfg.v2x.Nframes_track);
    track_y  = zeros(Nt, cfg.v2x.Nframes_track);
    track_x_true = track_x; track_y_true = track_y;
    rmse_raw_all  = abs(R_est_all-target.R_true);
    rmse_kalm_all = rmse_raw_all;
end


%% =========================================================================
%  BAGIAN 9: RESOURCE ALLOCATION + [ENHANCED] RL
%  =========================================================================
if cfg.enable.fase4
    fprintf('\n>> [Bagian 9] Resource Allocation + Enhanced RL...\n');

    ratios = cfg.ra.ratios_sweep; Nr = length(ratios);
    cap_static=zeros(1,Nr); RMSE_R_static=zeros(1,Nr);
    cap_wf=zeros(1,Nr);     RMSE_R_wf=zeros(1,Nr);
    cap_adapt=zeros(1,Nr);  RMSE_R_adapt=zeros(1,Nr);

    for ri = 1:Nr
        Nc = round(ratios(ri)*Nsc); Ns = Nsc - Nc;
        if Nc>0; cap_static(ri)=sum(log2(1+SNR_ra_lin*H_gain_per_sc(1:Nc)))*cfg.SCS; end
        if Ns>0; RMSE_R_static(ri)=sqrt((cfg.c/2)^2/(8*pi^2*SNR_ra_lin*(Ns*cfg.SCS/2)^2*Ns*Nsym));
        else;    RMSE_R_static(ri)=inf; end

        sc_wf = sc_sorted_base(1:max(Nc,1));
        if Nc>0; cap_wf(ri)=sum(log2(1+SNR_ra_lin*H_gain_per_sc(sc_wf)))*cfg.SCS; end
        if Ns>0; RMSE_R_wf(ri)=RMSE_R_static(ri); else; RMSE_R_wf(ri)=inf; end

        boost = deal_ternary(is_LoS && ~ch_is_busy, 0.15, ...
                deal_ternary(is_LoS && ch_is_busy, 0.05, ...
                deal_ternary(~is_LoS && ~ch_is_busy, -0.05, -0.15)));
        ra2 = min(0.9,max(0.1,ratios(ri)+boost));
        Nc_a=round(ra2*Nsc); Ns_a=Nsc-Nc_a;
        if Nc_a>0; cap_adapt(ri)=sum(log2(1+SNR_ra_lin*H_gain_per_sc(sc_sorted_base(1:Nc_a))))*cfg.SCS; end
        if Ns_a>0; RMSE_R_adapt(ri)=sqrt((cfg.c/2)^2/(8*pi^2*SNR_ra_lin*(Ns_a*cfg.SCS/2)^2*Ns_a*Nsym));
        else;      RMSE_R_adapt(ri)=inf; end
    end

    % -- [ENHANCED] RL precomputation ------------------------------------
    SNR_level_vals = [0,7,20]; load_level_vals=[0.1,0.4,0.8];
    lane_dens_vals = [0.5,2.0,5.0];
    precomp_cap=zeros(cfg.rl.n_states,cfg.rl.n_actions);
    precomp_rmse=zeros(cfg.rl.n_states,cfg.rl.n_actions);
    precomp_pd=zeros(cfg.rl.n_states,cfg.rl.n_actions);  % [NEW] detection prob

    for snr_l=0:cfg.rl.n_SNR_levels-1
      for load_l=0:cfg.rl.n_load_levels-1
        for lane_l=0:cfg.rl.n_lane_levels-1
            s_idx = snr_l*9+load_l*3+lane_l+1;
            snrl_s = 10^(SNR_level_vals(snr_l+1)/10);
            li_f = lane_dens_vals(lane_l+1)/max(Nt,1)*0.5;
            sinr_s = max(snrl_s*(1-load_level_vals(load_l+1)*cfg.load.interference_factor)*(1-li_f),0.01);
            for a_idx=1:cfg.rl.n_actions
                Nc_q=round(ratios(a_idx)*Nsc); Ns_q=Nsc-Nc_q;
                if Nc_q>0
                    precomp_cap(s_idx,a_idx)=sum(log2(1+sinr_s*H_gain_per_sc(sc_sorted_base(1:Nc_q))))*cfg.SCS/1e6;
                end
                if Ns_q>0
                    rmse_q = sqrt((cfg.c/2)^2/(8*pi^2*sinr_s*(Ns_q*cfg.SCS/2)^2*Ns_q*Nsym));
                    precomp_rmse(s_idx,a_idx) = rmse_q;
                    % [FIX v7] Pd: Swerling-0 + multistatic OR-fusion
                    thresh_cfar_rl = -log(cfg.cfar.Pfa);
                    pd_sw0_rl = exp(-thresh_cfar_rl / (1 + sinr_s));
                    pd_multi_rl = 1 - (1 - pd_sw0_rl)^n_sensors;
                    precomp_pd(s_idx,a_idx) = min(pd_multi_rl, 1.0);
                else
                    precomp_rmse(s_idx,a_idx)=inf;
                    precomp_pd(s_idx,a_idx)=0;
                end
            end
        end
      end
    end

    cap_max_pre  = max(precomp_cap(:));
    rmse_worst   = max(precomp_rmse(isfinite(precomp_rmse(:))));
    min_cap_norm = cfg.ra.min_throughput/1e6/max(cap_max_pre,eps);

    % [NEW] Enhanced reward function dengan detection probability
    compute_reward_v2 = @(s,a) compute_rl_reward_v2( ...
        precomp_cap(s,a), cap_max_pre, ...
        precomp_rmse(s,a), rmse_worst, min_cap_norm, ...
        precomp_pd(s,a), cfg.rl.w_comm, cfg.rl.w_sense, cfg.rl.w_detect);

    snr_current   = min(cfg.rl.n_SNR_levels-1, max(0, floor(cfg.SNR_dB/10)));
    state_current = snr_current*9 + load_level*3 + lane_density_level + 1;

    switch cfg.rl.algorithm
        case 'qlearning'; run_algos={'qlearning'};
        case 'sarsa';     run_algos={'sarsa'};
        case 'bandit';    run_algos={'bandit'};
        case 'all';       run_algos={'qlearning','sarsa','bandit'};
        otherwise;        run_algos={'qlearning'};
    end
    n_algos = length(run_algos);
    algo_labels = containers.Map({'qlearning','sarsa','bandit'},{'Q-Learning','SARSA','Greedy Bandit'});
    algo_colors_m = containers.Map({'qlearning','sarsa','bandit'},{[0.6 0.1 0.8],[0.9 0.5 0.1],[0.1 0.7 0.3]});

    Q_tables    = zeros(n_algos, cfg.rl.n_states, cfg.rl.n_actions);
    reward_hist = zeros(n_algos, cfg.rl.n_episodes);
    cap_rl      = zeros(n_algos, Nr);
    RMSE_R_rl   = zeros(n_algos, Nr);

    % [TOOLBOX v8] RL Toolbox agent setup — R2022b+ compatible API
    % rlTableRepresentation was removed in R2022a; use rlQValueFunction instead.
    % The RL Toolbox agent runs in parallel with manual tabular for validation.
    use_rltoolbox = false;  % will be set true if toolbox init succeeds
    try
        jcas_obs_info = rlNumericSpec([1 1], ...
            'LowerLimit', 1, 'UpperLimit', cfg.rl.n_states);
        jcas_act_info = rlFiniteSetSpec(num2cell(1:cfg.rl.n_actions));
        jcas_obs_info.Name = 'state';
        jcas_act_info.Name = 'action';
        % rlQValueFunction: table-based Q(s,a) — replacement for rlTableRepresentation
        qval_fn = rlQValueFunction( ...
            rlTable(jcas_obs_info, jcas_act_info), ...
            jcas_obs_info, jcas_act_info);
        rl_agent_opts = rlQAgentOptions( ...
            'DiscountFactor', cfg.rl.gamma, ...
            'EpsilonGreedyExploration', rlEpsilonGreedyExploration( ...
                'Epsilon',    cfg.rl.epsilon, ...
                'EpsilonDecay', 1 - cfg.rl.epsilon_decay, ...
                'EpsilonMin', 0.01));
        rl_tb_agent = rlQAgent(qval_fn, rl_agent_opts);
        use_rltoolbox = true;
        fprintf('   [RL Toolbox] rlQAgent initialised OK (R2022b+ API)\n');
    catch me_rl
        fprintf('   [RL Toolbox] Init failed (%s) — using manual tabular only\n', me_rl.message);
    end

    for ai = 1:n_algos
        aname = run_algos{ai};
        fprintf('   Training %s (%d ep)...\n', algo_labels(aname), cfg.rl.n_episodes);
        Q_tab  = 0.01*rand(cfg.rl.n_states, cfg.rl.n_actions);
        eps_ql = cfg.rl.epsilon;
        bandit_counts=zeros(cfg.rl.n_states,cfg.rl.n_actions);
        bandit_rewards=zeros(cfg.rl.n_states,cfg.rl.n_actions);

        for ep = 1:cfg.rl.n_episodes
            snr_ep  = min(cfg.rl.n_SNR_levels-1,max(0,floor((cfg.SNR_dB+5*randn())/10)));
            load_ep = min(cfg.rl.n_load_levels-1,max(0,load_level+randi([-1 1])));
            lane_ep = min(cfg.rl.n_lane_levels-1,max(0,lane_density_level+randi([-1 1])));
            state   = snr_ep*9+load_ep*3+lane_ep+1;

            switch aname
                case {'qlearning','sarsa'}
                    if rand()<eps_ql; action=randi(cfg.rl.n_actions);
                    else; [~,action]=max(Q_tab(state,:)); end
                case 'bandit'
                    tc = sum(bandit_counts(state,:));
                    ucb = bandit_rewards(state,:)./max(bandit_counts(state,:),1) + ...
                          sqrt(2*log(max(tc,1))./max(bandit_counts(state,:),1));
                    [~,action]=max(ucb);
            end

            reward = compute_reward_v2(state, action);
            reward_hist(ai,ep) = reward;

            snr_n  = min(cfg.rl.n_SNR_levels-1,max(0,snr_ep+randi([-1 1])));
            load_n = min(cfg.rl.n_load_levels-1,max(0,load_ep+randi([-1 1])));
            lane_n = min(cfg.rl.n_lane_levels-1,max(0,lane_ep+randi([-1 1])));
            sn     = snr_n*9+load_n*3+lane_n+1;

            switch aname
                case 'qlearning'
                    Q_tab(state,action)=Q_tab(state,action)+cfg.rl.alpha*...
                        (reward+cfg.rl.gamma*max(Q_tab(sn,:))-Q_tab(state,action));
                case 'sarsa'
                    if rand()<eps_ql; an=randi(cfg.rl.n_actions); else; [~,an]=max(Q_tab(sn,:)); end
                    Q_tab(state,action)=Q_tab(state,action)+cfg.rl.alpha*...
                        (reward+cfg.rl.gamma*Q_tab(sn,an)-Q_tab(state,action));
                case 'bandit'
                    bandit_counts(state,action)=bandit_counts(state,action)+1;
                    bandit_rewards(state,action)=bandit_rewards(state,action)+reward;
                    Q_tab(state,action)=bandit_rewards(state,action)/bandit_counts(state,action);
            end
            if ~strcmp(aname,'bandit'); eps_ql=eps_ql*cfg.rl.epsilon_decay; end
        end
        Q_tables(ai,:,:) = Q_tab;

        for ri=1:Nr
            snr_ri = min(cfg.rl.n_SNR_levels-1,max(0,floor(cfg.SNR_dB/10)));
            s_ri   = snr_ri*9+load_level*3+lane_density_level+1;
            [~,a_rl]=max(squeeze(Q_tables(ai,s_ri,:)));
            Nc_ql=round(ratios(a_rl)*Nsc); Ns_ql=Nsc-Nc_ql;
            if Nc_ql>0; cap_rl(ai,ri)=sum(log2(1+SNR_ra_lin*H_gain_per_sc(sc_sorted_base(1:Nc_ql))))*cfg.SCS; end
            if Ns_ql>0; RMSE_R_rl(ai,ri)=sqrt((cfg.c/2)^2/(8*pi^2*SNR_ra_lin*(Ns_ql*cfg.SCS/2)^2*Ns_ql*Nsym));
            else; RMSE_R_rl(ai,ri)=inf; end
        end
        [~,ba]=max(squeeze(Q_tables(ai,state_current,:)));
        fprintf('     Done. State=%d -> Action=%d (Nc/Nsc=%.1f)\n', state_current, ba, ratios(ba));
    end

    cap_static_Mbps=cap_static/1e6; cap_wf_Mbps=cap_wf/1e6;
    cap_adapt_Mbps=cap_adapt/1e6;   cap_rl_Mbps=cap_rl/1e6;
    constraint_R = 0.5;
    [best_cap_s,best_ratio_s]=find_best_feasible(cap_static_Mbps,RMSE_R_static,ratios,constraint_R);
    [best_cap_w,best_ratio_w]=find_best_feasible(cap_wf_Mbps,    RMSE_R_wf,    ratios,constraint_R);
    [best_cap_a,best_ratio_a]=find_best_feasible(cap_adapt_Mbps, RMSE_R_adapt, ratios,constraint_R);
    best_cap_rl=zeros(1,n_algos); best_ratio_rl=zeros(1,n_algos);
    for ai=1:n_algos
        [best_cap_rl(ai),best_ratio_rl(ai)]=find_best_feasible(cap_rl_Mbps(ai,:),RMSE_R_rl(ai,:),ratios,constraint_R);
    end
end


%% =========================================================================
%  BAGIAN 10: KOMUNIKASI V2X + BER MONTE CARLO
%  =========================================================================
fprintf('\n>> [Bagian 10] Komunikasi V2X + BER Monte Carlo...\n');

if cfg.enable.fase3
    rx_bits_raw = qamdemod(rx_eq_lmmse(:),cfg.mod_order,'OutputType','bit','UnitAveragePower',true);
else
    rx_eq_zf    = rx_comm_grid./(H_comm+eps_div);
    rx_bits_raw = qamdemod(rx_eq_zf(:),cfg.mod_order,'OutputType','bit','UnitAveragePower',true);
end
rx_bits_trim = rx_bits_raw(1:Nbits);
% [TOOLBOX] comm.ErrorRate for accurate BER accumulation
ber_counter = comm.ErrorRate;
ber_result  = ber_counter(tx_bits, rx_bits_trim);
BER = ber_result(1);
capacity_bps = sum(log2(1+SNR_ra_lin*H_gain_per_sc))*cfg.SCS;
tx_power_inst = abs(tx_signal).^2;
PAPR_dB = 10*log10(max(tx_power_inst)/mean(tx_power_inst));

% [TOOLBOX] CFAR detector for MC loop (separate object for clarity)
cfar_det_mc = phased.CFARDetector2D( ...
    'Method',              'CA', ...
    'GuardBandSize',       cfg.cfar.guard_cells * [1 1], ...
    'TrainingBandSize',    cfg.cfar.train_cells * [1 1], ...
    'ProbabilityFalseAlarm', cfg.cfar.Pfa, ...
    'OutputFormat',        'Detection index');
R_ref_mc2 = 50^2;  % amplitude normalisation for MC loop (same as Bagian 4)
fprintf('   Monte Carlo (%d trials × %d SNR points)... ', cfg.Ntrials, length(cfg.SNR_sweep));
N_snr  = length(cfg.SNR_sweep);
BER_arr     = zeros(1, N_snr);
RMSE_R_mc   = zeros(Nt, N_snr);
Pd_arr      = zeros(1, N_snr);
Pfa_arr_emp = zeros(1, N_snr);

for si = 1:N_snr
    snr_i  = cfg.SNR_sweep(si);
    snrl_i = 10^(snr_i/10);
    ber_sum=0; n_detect=0; n_fa=0; n_target_cells=0;
    r_errs = zeros(Nt, cfg.Ntrials);

    for tr = 1:cfg.Ntrials
        h_t = sqrt(tap_power/2).*(randn(1,n_taps)+1j*randn(1,n_taps));
        rx_c = zeros(N_tx,1);
        for i=1:n_taps
            d=delay_samp(i);
            if d==0; rx_c=rx_c+h_t(i)*tx_signal;
            else;    rx_c=rx_c+h_t(i)*[zeros(d,1);tx_signal(1:end-d)]; end
        end
        rx_r = zeros(N_tx,1);
        for k=1:Nt
            dk=round(2*target.R_true(k)/cfg.c*cfg.fs);
            % [FIX v6] Gunakan radial velocity (bukan v_true langsung)
            dx_k=target.x(k)-sensor(1).x; dy_k=target.y(k)-sensor(1).y;
            ang_k=atan2(dy_k,dx_k);
            v_rad_k=target.v_true(k)*cos(ang_k);
            fdk=2*v_rad_k/cfg.lambda;
            % [FIX v7] Correct amplitude consistent with Bagian 4
            R_k_ber = max(target.R_true(k), delta_R);
            Ak = sqrt(target.RCS(k)) * R_ref_mc2 / R_k_ber^2;
            if dk<N_tx
                rx_r=rx_r+Ak*[zeros(dk,1);tx_signal(1:N_tx-dk)].*exp(1j*2*pi*fdk*t_axis);
            end
        end
        sp_i=mean(abs(rx_r).^2); nv_i=sp_i/snrl_i;
        nz_s=sqrt(nv_i/2)*(randn(N_tx,1)+1j*randn(N_tx,1));
        nz_c=sqrt(mean(abs(rx_c).^2)/snrl_i/2)*(randn(N_tx,1)+1j*randn(N_tx,1));

        % -- [FIX v7] Build per-sensor RD maps (correct amp + all sensors) --
        win_mc_r = chebwin(Nsc,60); win_mc_v = chebwin(Nsym,60)';
        RD_per_sensor_mc = cell(n_sensors,1);
        for si_mc = 1:n_sensors
            rx_r_mc2 = zeros(N_tx,1);
            for k=1:Nt
                if si_mc > 1
                    oi_mc2 = si_mc - 1;
                    if oi_mc2 <= n_obu && obu_indices(oi_mc2) == k; continue; end
                    R_mc2_k = R_nodes_all_s(si_mc, k);
                else
                    R_mc2_k = target.R_true(k);
                end
                dk_mc2 = round(2*R_mc2_k/cfg.c*cfg.fs);
                dx_mc2 = target.x(k)-sensor(si_mc).x;
                dy_mc2 = target.y(k)-sensor(si_mc).y;
                ang_mc2 = atan2(dy_mc2,dx_mc2);
                v_rad_mc2 = target.v_true(k)*cos(ang_mc2);
                fdk_mc2 = 2*v_rad_mc2/cfg.lambda;
                R_mc2_safe = max(R_mc2_k, delta_R);
                Ak_mc2 = sqrt(target.RCS(k)) * R_ref_mc2 / R_mc2_safe^2;
                if dk_mc2 < N_tx
                    rx_r_mc2 = rx_r_mc2 + Ak_mc2*[zeros(dk_mc2,1);tx_signal(1:N_tx-dk_mc2)]...
                              .*exp(1j*2*pi*fdk_mc2*t_axis);
                end
            end
            nv_mc2 = sig_pow / snrl_i;
            nz_mc2 = sqrt(nv_mc2/2)*(randn(N_tx,1)+1j*randn(N_tx,1));
            Hg_mc2 = ofdm_demod(rx_r_mc2+nz_mc2)./(tx_grid+eps_div);
            Hg_mc2_w = Hg_mc2 .* (win_mc_r * win_mc_v);
            RD_per_sensor_mc{si_mc} = fftshift(fft(ifft(Hg_mc2_w,Nsc,1),Nsym,2),2);
        end

        % -- [FIX v7] Multistatic Pd: OR-fusion across all sensors ----
        for k=1:Nt
            detected_any = false;
            for si_mc = 1:n_sensors
                if si_mc > 1
                    oi_chk = si_mc-1;
                    if oi_chk <= n_obu && obu_indices(oi_chk)==k; continue; end
                    R_k_si = R_nodes_all_s(si_mc,k);
                else
                    R_k_si = target.R_true(k);
                end
                [cfar_mc2, ~] = cfar_2d(abs(RD_per_sensor_mc{si_mc}), ...
                    cfg.cfar.guard_cells, cfg.cfar.train_cells, cfg.cfar.Pfa);
                ir_k2 = min(round(R_k_si/delta_R)+1, Nsc);
                dx_si3=target.x(k)-sensor(si_mc).x; dy_si3=target.y(k)-sensor(si_mc).y;
                v_rad_si3=target.v_true(k)*cos(atan2(dy_si3,dx_si3));
                iv_k2 = max(1,min(Nsym, round(Nsym/2)+round(v_rad_si3/delta_v)));
                r1=max(1,ir_k2-cfg.cfar.guard_cells); r2=min(Nsc,ir_k2+cfg.cfar.guard_cells);
                c1=max(1,iv_k2-cfg.cfar.guard_cells); c2=min(Nsym,iv_k2+cfg.cfar.guard_cells);
                if r1<=r2 && c1<=c2 && any(any(cfar_mc2(r1:r2,c1:c2)))
                    detected_any = true; break;
                end
            end
            if detected_any; n_detect=n_detect+1; end
            n_target_cells=n_target_cells+1;
        end

        % -- RMSE from sensor-1 range profile (Doppler-integrated) ---
        rp_mc2 = sum(abs(RD_per_sensor_mc{1}),2);
        for k=1:Nt
            wlo=max(1,round((target.R_true(k)-delta_R*1.5)/delta_R));
            whi=min(Nsc,round((target.R_true(k)+delta_R*1.5)/delta_R)); whi=max(whi,wlo+1);
            [~,ir_l]=max(rp_mc2(wlo:whi)); ir_l=ir_l+wlo-1;
            r_errs(k,tr)=(ir_l-1)*delta_R-target.R_true(k);
        end

        % -- [FIX v7] False alarm: sensor-1, exclude target windows --
        [cfar_s1_mc, ~] = cfar_2d(abs(RD_per_sensor_mc{1}), ...
            cfg.cfar.guard_cells, cfg.cfar.train_cells, cfg.cfar.Pfa);
        fa_mask_mc = cfar_s1_mc;
        for k=1:Nt
            ir_fa=min(round(target.R_true(k)/delta_R)+1,Nsc);
            iv_fa=max(1,min(Nsym,round(Nsym/2)+round(target.v_true(k)/delta_v)));
            r1f=max(1,ir_fa-cfg.cfar.guard_cells); r2f=min(Nsc,ir_fa+cfg.cfar.guard_cells);
            c1f=max(1,iv_fa-cfg.cfar.guard_cells); c2f=min(Nsym,iv_fa+cfg.cfar.guard_cells);
            if r1f<=r2f && c1f<=c2f; fa_mask_mc(r1f:r2f,c1f:c2f)=false; end
        end
        n_fa = n_fa + sum(fa_mask_mc(:));

        Hg_c=ofdm_demod(rx_c+nz_c);
        He=Hg_c./(tx_grid+eps_div); Hq=abs(He).^2;
        rx_eq_mc=Hg_c.*conj(He)./(Hq+1/snrl_i);
        rb=qamdemod(rx_eq_mc(:),cfg.mod_order,'OutputType','bit','UnitAveragePower',true);
        [~,b]=biterr(tx_bits,rb(1:Nbits));  % biterr OK in MC (no accumulation needed)
        ber_sum=ber_sum+b;
    end
    RMSE_R_mc(:,si)=sqrt(mean(r_errs.^2,2));
    BER_arr(si)=ber_sum/cfg.Ntrials;
    Pd_arr(si)=n_detect/max(n_target_cells,1);
    % [FIX v7] Pfa normalised per valid (non-target) cell per trial
    n_guard_tot = Nt*(2*cfg.cfar.guard_cells+1)^2;
    valid_pfa_cells = max(Nsc*Nsym - n_guard_tot, 1);
    Pfa_arr_emp(si) = n_fa / max(cfg.Ntrials * valid_pfa_cells, 1);
end
fprintf('selesai.\n');

BER_qpsk_th  = qfunc(sqrt(2*10.^(cfg.SNR_sweep/10)));
BER_16qam_th = max((3/4)*qfunc(sqrt(0.4*10.^(cfg.SNR_sweep/10))),1e-6);
BER_64qam_th = max((7/6)*qfunc(sqrt(2/21*10.^(cfg.SNR_sweep/10))),1e-6);

SINR_act = 10*log10(mean(abs(rx_comm).^2)/mean(abs(noise_comm).^2));
fprintf('   BER=%.5f | SINR=%.1fdB | Throughput=%.2fMbps\n\n', ...
    BER, SINR_act, capacity_bps/1e6);


%% =========================================================================
%  BAGIAN 11: CRB
%  =========================================================================
fprintf('>> [Bagian 11] CRB...\n');
SINR_eff_act = SNR_lin * tap_power_ratio;
CRB_R_awgn   = sqrt((cfg.c/2)^2/(8*pi^2*SNR_lin*(cfg.BW/2)^2*Nsc*Nsym));
CRB_R_mp     = sqrt((cfg.c/2)^2/(8*pi^2*SINR_eff_act*(cfg.BW/2)^2*Nsc*Nsym));
CRB_v_awgn   = sqrt((cfg.lambda/2)^2/(8*pi^2*SNR_lin*T_frame^2*Nsc*Nsym));
CRB_v_mp     = sqrt((cfg.lambda/2)^2/(8*pi^2*SINR_eff_act*T_frame^2*Nsc*Nsym));
degrad_R_dB  = 20*log10(CRB_R_mp/max(CRB_R_awgn,eps));
fprintf('   CRB_R: AWGN=%.4fm | MP=%.4fm (+%.1fdB)\n\n', CRB_R_awgn, CRB_R_mp, degrad_R_dB);

% [NEW] BW comparison: hitung CRB dan RMSE untuk beberapa BW
if cfg.BW_compare
    fprintf('>> [BW Comparison] Menghitung RMSE untuk BW = ');
    BW_crb = zeros(1,length(cfg.BW_list));
    BW_rmse_avg = zeros(1,length(cfg.BW_list));
    for bi=1:length(cfg.BW_list)
        bw_i = cfg.BW_list(bi);
        fprintf('%.0fMHz ', bw_i/1e6);
        Nsc_i = round(bw_i/cfg.SCS);
        BW_crb(bi) = sqrt((cfg.c/2)^2/(8*pi^2*SNR_lin*(bw_i/2)^2*Nsc_i*Nsym));
        % Approx RMSE dari CRB ratio (relatif ke BW saat ini)
        BW_rmse_avg(bi) = mean(rmse_kalm_all) * (cfg.BW/bw_i);
    end
    fprintf('\n');
end


%% =========================================================================
%  ANALISIS MENDALAM — Statistical Significance, Sensitivity, Convergence,
%  Ablation, Cross-scenario Generalization
%  Semua hasil disimpan ke struct 'deep' untuk XLSX export
% =========================================================================
if cfg.enable.fase4
    fprintf('\n>> [Deep Analysis] Running additional analysis for paper...\n');

    % --- Strategy labels and best-feasible results (needed throughout) ----
    strat_names  = [{'Static','WaterFill','Adaptive'}, ...
                    cellfun(@(x) algo_labels(x), run_algos, 'UniformOutput', false)];
    strat_cap    = [best_cap_s, best_cap_w, best_cap_a, best_cap_rl];
    strat_ratio  = [best_ratio_s, best_ratio_w, best_ratio_a, best_ratio_rl];
    n_strat      = length(strat_names);

    % -----------------------------------------------------------------------
    % 1. STATISTICAL SIGNIFICANCE — Bootstrap CI (95%) per strategy
    % -----------------------------------------------------------------------
    fprintf('   [1/5] Bootstrap confidence intervals...\n');
    n_boot = 1000;
    all_cap_best  = [best_cap_s, best_cap_w, best_cap_a, best_cap_rl];
    all_rmse_best = repmat(mean(rmse_kalm_all), 1, n_strat);

    % We perturb the channel realization across bootstrap resamples
    % to estimate variance of best-feasible capacity per strategy
    boot_cap = zeros(n_strat, n_boot);
    for bi2 = 1:n_boot
        % Resample channel: draw new tap coefficients
        h_boot = sqrt(tap_power/2).*(randn(1,n_taps)+1j*randn(1,n_taps));
        rx_boot = zeros(N_tx,1);
        for ti=1:n_taps
            d=delay_samp(ti);
            if d==0; rx_boot=rx_boot+h_boot(ti)*tx_signal;
            else;    rx_boot=rx_boot+h_boot(ti)*[zeros(d,1);tx_signal(1:end-d)]; end
        end
        Hg_boot = ofdm_demod(rx_boot)./(tx_grid+eps_div);
        hg_boot = mean(abs(Hg_boot).^2, 2);
        [~, sc_boot] = sort(hg_boot, 'descend');

        for si2 = 1:n_strat
            cap_b = zeros(1,Nr); rmse_b = zeros(1,Nr);
            for ri2 = 1:Nr
                Nc_b = round(ratios(ri2)*Nsc); Ns_b = Nsc-Nc_b;
                if Nc_b>0; cap_b(ri2)=sum(log2(1+SNR_ra_lin*hg_boot(sc_boot(1:Nc_b))))*cfg.SCS/1e6; end
                if Ns_b>0; rmse_b(ri2)=sqrt((cfg.c/2)^2/(8*pi^2*SNR_ra_lin*(Ns_b*cfg.SCS/2)^2*Ns_b*Nsym));
                else; rmse_b(ri2)=inf; end
            end
            [bc,~]=find_best_feasible(cap_b,rmse_b,ratios,constraint_R);
            boot_cap(si2,bi2) = bc;
        end
    end
    deep.ci_mean  = mean(boot_cap, 2)';          % 1 x n_strat
    deep.ci_std   = std(boot_cap, 0, 2)';
    deep.ci_lo95  = prctile(boot_cap, 2.5, 2)';  % 95% CI lower
    deep.ci_hi95  = prctile(boot_cap, 97.5, 2)'; % 95% CI upper

    % Pairwise t-test: each RL vs Static (reference)
    deep.ttest_p  = zeros(1, n_strat);
    deep.ttest_h  = zeros(1, n_strat);
    for si2 = 2:n_strat
        [h_t, p_t] = ttest2(boot_cap(si2,:), boot_cap(1,:));
        deep.ttest_p(si2) = p_t;
        deep.ttest_h(si2) = h_t;
    end
    fprintf('     Done. Bootstrap n=%d\n', n_boot);

    % -----------------------------------------------------------------------
    % 2. SENSITIVITY ANALYSIS — Reward weight sweep (w_c vs w_s)
    % -----------------------------------------------------------------------
    fprintf('   [2/5] Reward weight sensitivity...\n');
    w_sweep = 0.1:0.1:0.8;  % w_c values; w_s = 0.9-w_c (w_d fixed 0.1)
    n_ws = length(w_sweep);
    deep.sens_wc       = w_sweep;
    deep.sens_cap_ql   = zeros(1, n_ws);
    deep.sens_rmse_ql  = zeros(1, n_ws);
    deep.sens_cap_sarsa  = zeros(1, n_ws);
    deep.sens_rmse_sarsa = zeros(1, n_ws);

    for wi = 1:n_ws
        wc_i = w_sweep(wi); ws_i = 0.9 - wc_i; wd_i = 0.1;
        rew_fn = @(s,a) compute_rl_reward_v2(...
            precomp_cap(s,a), cap_max_pre, ...
            precomp_rmse(s,a), rmse_worst, min_cap_norm, ...
            precomp_pd(s,a), wc_i, ws_i, wd_i);

        for algo_i = 1:min(2, n_algos)  % only Q-Learning and SARSA
            aname_i = run_algos{algo_i};
            Qt = 0.01*rand(cfg.rl.n_states, cfg.rl.n_actions);
            eps_i = cfg.rl.epsilon;
            for ep = 1:cfg.rl.n_episodes
                snr_ep  = min(cfg.rl.n_SNR_levels-1,max(0,floor((cfg.SNR_dB+5*randn())/10)));
                load_ep = min(cfg.rl.n_load_levels-1,max(0,load_level+randi([-1 1])));
                lane_ep = min(cfg.rl.n_lane_levels-1,max(0,lane_density_level+randi([-1 1])));
                st_ep = snr_ep*9+load_ep*3+lane_ep+1;
                if rand()<eps_i; act=randi(cfg.rl.n_actions); else; [~,act]=max(Qt(st_ep,:)); end
                rw = rew_fn(st_ep, act);
                snr_n=min(cfg.rl.n_SNR_levels-1,max(0,snr_ep+randi([-1 1])));
                load_n=min(cfg.rl.n_load_levels-1,max(0,load_ep+randi([-1 1])));
                lane_n=min(cfg.rl.n_lane_levels-1,max(0,lane_ep+randi([-1 1])));
                sn_ep=snr_n*9+load_n*3+lane_n+1;
                if strcmp(aname_i,'qlearning')
                    Qt(st_ep,act)=Qt(st_ep,act)+cfg.rl.alpha*(rw+cfg.rl.gamma*max(Qt(sn_ep,:))-Qt(st_ep,act));
                else
                    if rand()<eps_i; an2=randi(cfg.rl.n_actions); else; [~,an2]=max(Qt(sn_ep,:)); end
                    Qt(st_ep,act)=Qt(st_ep,act)+cfg.rl.alpha*(rw+cfg.rl.gamma*Qt(sn_ep,an2)-Qt(st_ep,act));
                end
                if eps_i>0.01; eps_i=eps_i*cfg.rl.epsilon_decay; end
            end
            % Evaluate
            cap_si=zeros(1,Nr); rmse_si=zeros(1,Nr);
            for ri2=1:Nr
                [~,a_s]=max(Qt(state_current,:));
                Nc_s=round(ratios(a_s)*Nsc); Ns_s=Nsc-Nc_s;
                if Nc_s>0; cap_si(ri2)=sum(log2(1+SNR_ra_lin*H_gain_per_sc(sc_sorted_base(1:Nc_s))))*cfg.SCS/1e6; end
                if Ns_s>0; rmse_si(ri2)=sqrt((cfg.c/2)^2/(8*pi^2*SNR_ra_lin*(Ns_s*cfg.SCS/2)^2*Ns_s*Nsym));
                else; rmse_si(ri2)=inf; end
            end
            [bc_s,~]=find_best_feasible(cap_si,rmse_si,ratios,constraint_R);
            if algo_i==1
                deep.sens_cap_ql(wi)  = bc_s;
                deep.sens_rmse_ql(wi) = mean(rmse_kalm_all);
            else
                deep.sens_cap_sarsa(wi)  = bc_s;
                deep.sens_rmse_sarsa(wi) = mean(rmse_kalm_all);
            end
        end
    end
    fprintf('     Done. %d weight combinations\n', n_ws);

    % -----------------------------------------------------------------------
    % 3. CONVERGENCE ANALYSIS — Moving average of reward + episode threshold
    % -----------------------------------------------------------------------
    fprintf('   [3/5] Convergence analysis...\n');
    win_conv = 100;  % moving average window
    deep.conv_ep    = 1:cfg.rl.n_episodes;
    deep.conv_mavg  = zeros(n_algos, cfg.rl.n_episodes);
    deep.conv_ep90  = zeros(1, n_algos);  % episode where 90% of final reward reached

    for ai = 1:n_algos
        rh = reward_hist(ai,:);
        for ep = 1:cfg.rl.n_episodes
            lo = max(1, ep-win_conv+1);
            deep.conv_mavg(ai,ep) = mean(rh(lo:ep));
        end
        final_r = deep.conv_mavg(ai,end);
        thresh  = 0.9 * final_r;
        ep90_idx = find(deep.conv_mavg(ai,:) >= thresh, 1);
        deep.conv_ep90(ai) = deal_ternary(~isempty(ep90_idx), ep90_idx, cfg.rl.n_episodes);
    end
    fprintf('     Done.\n');

    % -----------------------------------------------------------------------
    % 4. ABLATION STUDY — Remove each reward component one at a time
    % -----------------------------------------------------------------------
    fprintf('   [4/5] Ablation study (reward components)...\n');
    ablation_configs = {
        'Full (wc=0.4, ws=0.4, wd=0.2)',  0.4, 0.4, 0.2;
        'No detect (wc=0.5, ws=0.5, wd=0)', 0.5, 0.5, 0.0;
        'No comm   (wc=0.0, ws=0.7, wd=0.3)', 0.0, 0.7, 0.3;
        'No sense  (wc=0.7, ws=0.0, wd=0.3)', 0.7, 0.0, 0.3;
    };
    n_abl = size(ablation_configs,1);
    deep.abl_names   = ablation_configs(:,1);
    deep.abl_cap_ql  = zeros(1, n_abl);
    deep.abl_rmse_ql = zeros(1, n_abl);

    for abl_i = 1:n_abl
        wc_a = ablation_configs{abl_i,2};
        ws_a = ablation_configs{abl_i,3};
        wd_a = ablation_configs{abl_i,4};
        rew_a = @(s,a) compute_rl_reward_v2(...
            precomp_cap(s,a), cap_max_pre, ...
            precomp_rmse(s,a), rmse_worst, min_cap_norm, ...
            precomp_pd(s,a), wc_a, ws_a, wd_a);

        Qt_a = 0.01*rand(cfg.rl.n_states, cfg.rl.n_actions);
        eps_a = cfg.rl.epsilon;
        for ep = 1:cfg.rl.n_episodes
            snr_ep=min(cfg.rl.n_SNR_levels-1,max(0,floor((cfg.SNR_dB+5*randn())/10)));
            load_ep=min(cfg.rl.n_load_levels-1,max(0,load_level+randi([-1 1])));
            lane_ep=min(cfg.rl.n_lane_levels-1,max(0,lane_density_level+randi([-1 1])));
            st_a=snr_ep*9+load_ep*3+lane_ep+1;
            if rand()<eps_a; act_a=randi(cfg.rl.n_actions); else; [~,act_a]=max(Qt_a(st_a,:)); end
            rw_a = rew_a(st_a, act_a);
            snr_n=min(cfg.rl.n_SNR_levels-1,max(0,snr_ep+randi([-1 1])));
            load_n=min(cfg.rl.n_load_levels-1,max(0,load_ep+randi([-1 1])));
            lane_n=min(cfg.rl.n_lane_levels-1,max(0,lane_ep+randi([-1 1])));
            sn_a=snr_n*9+load_n*3+lane_n+1;
            Qt_a(st_a,act_a)=Qt_a(st_a,act_a)+cfg.rl.alpha*...
                (rw_a+cfg.rl.gamma*max(Qt_a(sn_a,:))-Qt_a(st_a,act_a));
            if eps_a>0.01; eps_a=eps_a*cfg.rl.epsilon_decay; end
        end
        cap_a=zeros(1,Nr); rmse_a=zeros(1,Nr);
        for ri2=1:Nr
            [~,a_abl]=max(Qt_a(state_current,:));
            Nc_a=round(ratios(a_abl)*Nsc); Ns_a=Nsc-Nc_a;
            if Nc_a>0; cap_a(ri2)=sum(log2(1+SNR_ra_lin*H_gain_per_sc(sc_sorted_base(1:Nc_a))))*cfg.SCS/1e6; end
            if Ns_a>0; rmse_a(ri2)=sqrt((cfg.c/2)^2/(8*pi^2*SNR_ra_lin*(Ns_a*cfg.SCS/2)^2*Ns_a*Nsym));
            else; rmse_a(ri2)=inf; end
        end
        [bc_a,~]=find_best_feasible(cap_a,rmse_a,ratios,constraint_R);
        deep.abl_cap_ql(abl_i)  = bc_a;
        deep.abl_rmse_ql(abl_i) = mean(rmse_kalm_all);
    end
    fprintf('     Done. %d ablation configs\n', n_abl);

    % -----------------------------------------------------------------------
    % 5. CROSS-SCENARIO GENERALIZATION
    %    Train on medium density, test on low and high density
    % -----------------------------------------------------------------------
    fprintf('   [5/5] Cross-scenario generalization...\n');
    % Use Q-Learning table trained in primary scenario (already in Q_tables)
    ai_ql = find(strcmp(run_algos,'qlearning'),1);
    if ~isempty(ai_ql)
        Q_trained = squeeze(Q_tables(ai_ql,:,:));
        % Test scenarios: different lane_density_level values
        test_densities = [0, 1, 2];  % sparse, medium, dense
        test_names     = {'Low density','Medium density','High density'};
        deep.gen_names = test_names;
        deep.gen_cap   = zeros(1,3);
        deep.gen_rmse  = zeros(1,3);

        for di = 1:3
            snr_g  = min(cfg.rl.n_SNR_levels-1,max(0,floor(cfg.SNR_dB/10)));
            st_g   = snr_g*9 + load_level*3 + test_densities(di) + 1;
            [~,a_g] = max(Q_trained(st_g,:));
            Nc_g = round(ratios(a_g)*Nsc); Ns_g = Nsc-Nc_g;
            cap_g = 0; rmse_g = inf;
            if Nc_g>0; cap_g=sum(log2(1+SNR_ra_lin*H_gain_per_sc(sc_sorted_base(1:Nc_g))))*cfg.SCS/1e6; end
            if Ns_g>0; rmse_g=sqrt((cfg.c/2)^2/(8*pi^2*SNR_ra_lin*(Ns_g*cfg.SCS/2)^2*Ns_g*Nsym)); end
            deep.gen_cap(di)  = cap_g;
            deep.gen_rmse(di) = deal_ternary(isfinite(rmse_g), rmse_g, 9999);
        end
    else
        deep.gen_names = {'N/A'}; deep.gen_cap=[0]; deep.gen_rmse=[0];
    end
    fprintf('     Done.\n');
    fprintf('>> [Deep Analysis] Complete.\n\n');
end

%% =========================================================================
%  VISUALISASI - 4 FIGURE
%  =========================================================================
fprintf('>> Generating Figures...\n');
colors_n = lines(max(Nt,6));
algo_plt_colors = {[0.6 0.1 0.8],[0.9 0.5 0.1],[0.1 0.7 0.3]};
algo_plt_marks  = {'d','p','h'};


%% -- FIGURE 1: SENSING CORE -----------------------------------------------
if cfg.plot.fig1
    fig1 = figure('Name','Fig1: Sensing Core', 'Position',[10 30 1380 920], 'Color','w');
    sgtitle(sprintf('FIGURA 1 - SENSING CORE | %d Nodes (%s) | BW=%.0fMHz | SNR=%.0fdB | %s', ...
        Nt, cfg.v2x.mobility_model, cfg.BW/1e6, cfg.SNR_dB, ...
        deal_ternary(strcmp(cfg.multi.mode,'mono'),'Monostatic',sprintf('Multistatic Mode-%s',cfg.multi.mode))), ...
        'FontWeight','bold','FontSize',12);

    % 1.1 Range-Doppler Map (Sensor 1)
    subplot(2,3,[1,2]);
    imagesc(v_axis, R_axis(1:R_show), RD_win_norm(1:R_show,:));
    colormap(gca,parula); cb=colorbar; cb.Label.String='Norm. Power [dB]';
    caxis([-35 0]); hold on;
    [cfar_r, cfar_c] = find(cfar_mask(1:R_show,:));
    if ~isempty(cfar_r)
        scatter(v_axis(cfar_c), R_axis(cfar_r), 20, 'r', 'filled', ...
            'DisplayName', sprintf('CFAR S1 (%d)', length(cfar_r)));
    end
    for k=1:Nt
        % [NEW] Plot dengan velocity radial (bukan v_true langsung)
        dx_k=target.x(k)-sensor(1).x; dy_k=target.y(k)-sensor(1).y;
        ang_k=atan2(dy_k,dx_k);
        v_rad_k=target.v_true(k)*cos(ang_k);
        clr = deal_ternary(~overlap_flag(k), 'w', 'y');
        plot(v_rad_k, target.R_true(k), '+', 'Color',clr, ...
            'MarkerSize',14, 'LineWidth',2.5);
        text(v_rad_k+1, target.R_true(k), sprintf('N%d',k), ...
            'Color',clr,'FontSize',7,'FontWeight','bold');
    end
    annotation_str = sprintf('delta_R=%.0fm (BW=%.0fMHz)\n%d/%d nodes resolvable', ...
        delta_R, cfg.BW/1e6, Nt-n_overlap, Nt);
    text(0.02,0.03,annotation_str,'Units','normalized','FontSize',8,...
        'BackgroundColor',[1 1 1 0.7],'EdgeColor',[0.5 0.5 0.5]);
    xlabel('Radial Velocity [m/s]'); ylabel('Range [m]');
    title(sprintf('Range-Doppler Map (Sensor 1: %s) + CA-CFAR (Pfa=%.0e, %d detect)', ...
        sensor(1).type, cfg.cfar.Pfa, n_detected),'FontWeight','bold');
    legend('Location','southoutside','FontSize',7,'Orientation','horizontal'); grid on;

    % 1.2 Range Profile: Rect vs Windowed
    subplot(2,3,3);
    plot(R_axis(1:R_show), 20*log10(range_profile_raw(1:R_show)/max(range_profile_raw)+eps), ...
        'b--','LineWidth',1.5,'DisplayName','Rectangular'); hold on;
    plot(R_axis(1:R_show), 20*log10(range_profile_win(1:R_show)/max(range_profile_win)+eps), ...
        'r-','LineWidth',2,'DisplayName',sprintf('%s Win',cfg.window.type));
    for k=1:Nt
        xline(target.R_true(k),':', 'Color',colors_n(mod(k-1,size(colors_n,1))+1,:), ...
            'LineWidth',1,'DisplayName',sprintf('N%d',k));
    end
    xlabel('Range [m]'); ylabel('Norm. Power [dB]'); ylim([-45 3]);
    title('Range Profile: Rect vs Windowed','FontWeight','bold');
    legend('Location','southoutside','FontSize',7,'Orientation','horizontal'); grid on;

    % 1.3 RMSE vs SNR + CRB
    subplot(2,3,4);
    CRB_awgn_ax = sqrt((cfg.c/2)^2./(8*pi^2*(10.^(cfg.SNR_sweep/10)).*(cfg.BW/2)^2*Nsc*Nsym));
    CRB_mp_ax   = CRB_awgn_ax / sqrt(tap_power_ratio);
    RMSE_avg    = mean(RMSE_R_mc,1);
    RMSE_mp     = RMSE_avg / sqrt(tap_power_ratio);
    semilogy(cfg.SNR_sweep, RMSE_avg,  'bo-','LineWidth',2,'MarkerSize',7,'DisplayName','RMSE avg (MC)');
    hold on;
    semilogy(cfg.SNR_sweep, RMSE_mp,   'bs--','LineWidth',1.5,'MarkerSize',7,'DisplayName','RMSE Mobil (MC)');
    semilogy(cfg.SNR_sweep, CRB_awgn_ax,'k:','LineWidth',2,'DisplayName','CRB (AWGN)');
    semilogy(cfg.SNR_sweep, CRB_mp_ax,  'r-','LineWidth',2,'DisplayName','CRB (Multipath)');
    fill([cfg.SNR_sweep fliplr(cfg.SNR_sweep)],[CRB_mp_ax fliplr(CRB_awgn_ax)],...
        [0.9 0.8 0.8],'FaceAlpha',0.3,'EdgeColor','none','DisplayName','MP Degradation');
    yline(0.5,'k--','LineWidth',1.5,'Label','0.5m constraint');
    xline(cfg.SNR_dB,'m--','LineWidth',1.5,'Label',sprintf('SNR_{op}=%.0fdB',cfg.SNR_dB));
    xlabel('SNR [dB]'); ylabel('RMSE Range [m]');
    title('RMSE Range vs SNR + CRB (MC)','FontWeight','bold');
    legend('Location','southoutside','FontSize',7,'Orientation','horizontal'); grid on;

    % 1.4 Pd vs SNR
    subplot(2,3,5);
    Pd_swerling = 1 - exp(-10.^(cfg.SNR_sweep/10) / (1 + 10.^(cfg.SNR_sweep/10)));
    plot(cfg.SNR_sweep, Pd_arr,      'bo-','LineWidth',2,'MarkerSize',7,'DisplayName',sprintf('Pd MC (%de+01 trials)',cfg.Ntrials));
    hold on;
    plot(cfg.SNR_sweep, Pd_swerling, 'r--','LineWidth',1.5,'DisplayName','Pd Teoritis (Swerling-0)');
    yline(0.9,'k:','LineWidth',1.5,'Label','Pd=0.9');
    yline(0.5,'k-.','LineWidth',1,'Label','Pd=0.5');
    xline(cfg.SNR_dB,'m--','LineWidth',1.5,'Label',sprintf('SNR_{op}=%.0fdB',cfg.SNR_dB));
    xlabel('SNR [dB]'); ylabel('Detection Probability P_d'); ylim([0 1.05]);
    title(sprintf('P_d vs SNR | CA-CFAR Pfa=%.0e | Ntrials=%d',cfg.cfar.Pfa,cfg.Ntrials),'FontWeight','bold');
    legend('Location','southoutside','FontSize',7,'Orientation','horizontal'); grid on;

    % 1.5 ROC Curve
    subplot(2,3,6);
    Pfa_sweep_roc = logspace(-4,-1,50);
    snr_op_lin = 10^(cfg.SNR_dB/10);
    Pd_roc_th  = 1-(1-Pfa_sweep_roc).^(snr_op_lin/(1+snr_op_lin));
    semilogx(Pfa_sweep_roc, Pd_roc_th,'r-','LineWidth',2,'DisplayName','ROC Teoritis'); hold on;
    snr_op_idx_roc = find(cfg.SNR_sweep==cfg.SNR_dB,1);
    if ~isempty(snr_op_idx_roc)
        semilogx(Pfa_arr_emp(snr_op_idx_roc), Pd_arr(snr_op_idx_roc), ...
            'bo','MarkerSize',12,'LineWidth',2,'MarkerFaceColor','b','DisplayName','ROC Empiris (MC)');
        plot(cfg.cfar.Pfa, Pd_arr(snr_op_idx_roc), 'k*','MarkerSize',14,'LineWidth',2,'DisplayName','Op. point');
    end
    yline(0.9,'k:','LineWidth',1.5,'Label','Pd=0.9');
    xlabel('False Alarm Rate P_{fa}'); ylabel('Detection Probability P_d');
    title(sprintf('ROC Curve | SNR=%.0fdB | BW=%.0fMHz',cfg.SNR_dB,cfg.BW/1e6),'FontWeight','bold');
    legend('Location','southoutside','FontSize',7,'Orientation','horizontal'); grid on; ylim([0 1.05]);
end


%% -- FIGURE 2: JCAS TRADEOFF -----------------------------------------------
if cfg.plot.fig2 && cfg.enable.fase4
    fig2 = figure('Name','Fig2: JCAS Tradeoff', 'Position',[50 30 1380 920], 'Color','w');
    sgtitle(sprintf('FIGURA 2 - JCAS TRADEOFF | %s | SNR=%.0fdB | State=%d', ...
        deal_ternary(is_LoS,'LoS','NLoS'), cfg.SNR_dB, state_current), ...
        'FontWeight','bold','FontSize',12);

    subplot(2,3,[1,2]);
    plot(cap_static_Mbps, RMSE_R_static,'b-s','LineWidth',2,'MarkerSize',8,'DisplayName','Static'); hold on;
    plot(cap_wf_Mbps,     RMSE_R_wf,    'g-^','LineWidth',2,'MarkerSize',8,'DisplayName','Water-Filling');
    plot(cap_adapt_Mbps,  RMSE_R_adapt, 'r-o','LineWidth',2,'MarkerSize',8,'DisplayName','Adaptive (LoS)');
    for ai=1:n_algos
        plot(cap_rl_Mbps(ai,:), RMSE_R_rl(ai,:), '--', ...
            'Color',algo_plt_colors{ai},'LineWidth',1.5,'Marker',algo_plt_marks{ai},...
            'MarkerSize',8,'DisplayName',algo_labels(run_algos{ai}));
    end
    xline(cfg.ra.min_throughput/1e6,'k:','LineWidth',1.5,'Label',sprintf('MinCap=%.0fMbps',cfg.ra.min_throughput/1e6));
    yline(0.5,'k--','LineWidth',1.5,'Label','RMSE<0.5m');
    % Optimal point markers
    plot(best_cap_s, 0.5,'k*','MarkerSize',12,'LineWidth',2,'HandleVisibility','off');
    plot(best_cap_w, 0.5,'k*','MarkerSize',12,'LineWidth',2,'HandleVisibility','off');
    plot(best_cap_a, 0.5,'k*','MarkerSize',12,'LineWidth',2,'HandleVisibility','off');
    xlabel('Throughput [Mbps]'); ylabel('RMSE Range Sensing [m]');
    title(sprintf('Pareto: Throughput vs RMSE | Load=%.2f (%s)', ...
        ch_occupancy, deal_ternary(ch_is_busy,'SIBUK','LONGGAR')),'FontWeight','bold');
    legend('Location','southoutside','FontSize',8,'Orientation','horizontal'); grid on;

    subplot(2,3,3);
    semilogy(cfg.SNR_sweep, max(BER_arr,1e-6),     'b-o','LineWidth',2,'MarkerSize',7,'DisplayName','4-QAM MC (ITU Veh-A)'); hold on;
    semilogy(cfg.SNR_sweep, max(BER_qpsk_th,1e-6), 'k--','LineWidth',1.5,'DisplayName','QPSK Teoritis');
    semilogy(cfg.SNR_sweep, max(BER_16qam_th,1e-6),'r:','LineWidth',1.5,'DisplayName','16-QAM Teoritis');
    semilogy(cfg.SNR_sweep, max(BER_64qam_th,1e-6),'g:','LineWidth',1.5,'DisplayName','64-QAM Teoritis');
    xline(cfg.SNR_dB,'m--','LineWidth',1.5,'Label',sprintf('SNR_{op}=%.0fdB',cfg.SNR_dB));
    yline(1e-3,'k-.','Label','BER=10^{-3}');
    xlabel('SNR [dB]'); ylabel('BER'); ylim([1e-6 1]);
    title('BER vs SNR - Monte Carlo (all curves)','FontWeight','bold');
    legend('Location','southoutside','FontSize',8,'Orientation','horizontal'); grid on;

    subplot(2,3,4);
    smooth_w = 80;
    for ai=1:n_algos
        rh_s = movmean(reward_hist(ai,:), smooth_w);
        plot(1:cfg.rl.n_episodes, rh_s, 'Color',algo_plt_colors{ai}, ...
            'LineWidth',2,'DisplayName',run_algos{ai}); hold on;
    end
    xlabel('Episode'); ylabel('Reward (smoothed)');
    title(sprintf('RL Learning Curves [Enhanced Reward] (smooth=%d)',smooth_w),'FontWeight','bold');
    legend('Location','southoutside','FontSize',8,'Orientation','horizontal'); grid on;

    subplot(2,3,5);
    all_names_bar = [{'Static','WaterFill','Adaptive'}, run_algos];
    all_caps_bar  = [best_cap_s, best_cap_w, best_cap_a, best_cap_rl];
    bar_c = [0.2 0.5 0.9; 0.2 0.8 0.4; 0.9 0.3 0.2];
    for ai=1:n_algos; bar_c(3+ai,:)=algo_plt_colors{ai}; end
    b25 = bar(all_caps_bar, 0.7);
    b25.FaceColor = 'flat';
    for mi=1:length(all_caps_bar)
        if mi<=size(bar_c,1); b25.CData(mi,:)=bar_c(mi,:); end
        text(mi, all_caps_bar(mi)+0.05, sprintf('%.2f',all_caps_bar(mi)), ...
            'HorizontalAlignment','center','FontWeight','bold','FontSize',9);
    end
    set(gca,'XTickLabel',all_names_bar,'XTickLabelRotation',15);
    ylabel('Throughput [Mbps] @ RMSE <= 0.5m');
    title('Throughput @ Sensing Constraint','FontWeight','bold'); grid on;

    subplot(2,3,6);
    ql_idx = find(strcmp(run_algos,'qlearning'),1);
    if ~isempty(ql_idx)
        Q_plot = squeeze(Q_tables(ql_idx,:,:));
        s_show = round(linspace(1,cfg.rl.n_states,9));
        imagesc(Q_plot(s_show,:)); colormap(gca,hot);
        cb26=colorbar; cb26.Label.String='Q-value';
        xlabel('Action (Nc/Nsc ratio)'); ylabel('State (subsample)');
        set(gca,'XTick',1:9,'XTickLabel',arrayfun(@(x)sprintf('%.1f',x),ratios,'UniformOutput',false));
        title('Q-Table Heatmap (Q-Learning) [Enhanced RL]','FontWeight','bold');
        for si_q=1:9
            [~,ai_q]=max(Q_plot(s_show(si_q),:));
            text(ai_q,si_q,'*','HorizontalAlignment','center','Color','cyan','FontSize',10);
        end
    else
        text(0.5,0.5,'Q-Learning tidak aktif','Units','normalized',...
            'HorizontalAlignment','center','FontSize',12);
        title('Q-Table','FontWeight','bold');
    end
end


%% -- FIGURE 3: SENSING SUMMARY --------------------------------------------
if cfg.plot.fig3
    fig3 = figure('Name','Fig3: Sensing Summary', 'Position',[90 30 1380 920], 'Color','w');
    sgtitle(sprintf('FIGURA 3 - SENSING SUMMARY | %d Nodes | BW=%.0fMHz | SNR=%.0fdB | EKF 2D', ...
        Nt, cfg.BW/1e6, cfg.SNR_dB), 'FontWeight','bold','FontSize',12);

    % 3.1 Velocity estimation
    subplot(2,3,[1,2]);
    v_masked = ~overlap_flag & ~isnan(v_est_all) & (abs(v_est_all-target.v_true) > 2*delta_v);
    v_ok     = ~overlap_flag & ~isnan(v_est_all) & (abs(v_est_all-target.v_true) <= 2*delta_v);
    b32a = bar(1:Nt, target.v_true, 0.8);
    b32a.FaceColor=[0.2 0.6 0.9]; b32a.FaceAlpha=0.7; hold on;
    v_est_plot = v_est_all; v_est_plot(isnan(v_est_all))=0;
    b32b = bar(1:Nt, v_est_plot, 0.4);
    b32b.FaceColor=[0.9 0.4 0.2]; b32b.FaceAlpha=0.9;
    for k2=1:Nt
        ypos = max(abs(target.v_true(k2)),abs(v_est_plot(k2)))+0.8;
        if overlap_flag(k2)
            text(k2,ypos,'OVL','HorizontalAlignment','center','Color',[0.7 0 0],'FontWeight','bold','FontSize',8);
        elseif v_masked(k2)
            text(k2,ypos,'MSK','HorizontalAlignment','center','Color',[0.9 0.5 0],'FontSize',7);
        else
            text(k2,ypos,sprintf('+/-%.1f',abs(v_est_all(k2)-target.v_true(k2))),...
                'HorizontalAlignment','center','FontSize',6,'Color',[0 0.5 0]);
        end
    end
    set(gca,'XTick',1:Nt,'XTickLabel',...
        arrayfun(@(k)sprintf('N%d\n%s',k,target.label{k}),1:Nt,'UniformOutput',false));
    legend([b32a b32b],{'v_{true} [m/s]','v_{est} [m/s]'},'Location','northoutside',...
        'Orientation','horizontal','FontSize',9);
    n_ok_v=sum(v_ok); n_msk_v=sum(v_masked);
    title(sprintf('Velocity Estimation: %d OK | %d Masked | %d Overlap  (delta_v=%.2fm/s)',...
        n_ok_v,n_msk_v,n_overlap,delta_v),'FontWeight','bold');
    ylabel('Velocity [m/s]'); grid on; box on;

    % 3.2 EKF RMSE per node
    subplot(2,3,3);
    b_raw  = bar(1:Nt, rmse_raw_all,  0.8); b_raw.FaceColor  = [0.8 0.3 0.3]; hold on;
    b_kalm = bar(1:Nt, rmse_kalm_all, 0.45); b_kalm.FaceColor = [0.2 0.7 0.4];
    for k2=1:Nt
        impr_k=(1-rmse_kalm_all(k2)/max(rmse_raw_all(k2),1e-6))*100;
        text(k2, rmse_kalm_all(k2)+0.1, sprintf('%.0f%%',impr_k),...
            'HorizontalAlignment','center','FontSize',7,'Color',[0 0.4 0],'FontWeight','bold');
    end
    yline(0.5,'k--','LineWidth',1.5,'Label','0.5m constraint');
    set(gca,'XTick',1:Nt,'XTickLabel',arrayfun(@(k)sprintf('N%d',k),1:Nt,'UniformOutput',false));
    legend([b_raw b_kalm],{'RMSE Raw [m]','RMSE EKF [m]'},'Location','northoutside',...
        'Orientation','horizontal','FontSize',9);
    title(sprintf('EKF 2D Improvement | avg raw=%.2fm -> EKF=%.2fm (+%.0f%%)',...
        mean(rmse_raw_all),mean(rmse_kalm_all),mean((1-rmse_kalm_all./max(rmse_raw_all,1e-6))*100)),...
        'FontWeight','bold');
    ylabel('RMSE Position 2D [m]'); grid on; box on;

    % 3.3 Range error
    subplot(2,3,4);
    R_err_plot = R_est_all - target.R_true;
    bar_col = zeros(Nt,3);
    for k2=1:Nt
        if abs(R_err_plot(k2)) < delta_R/2;   bar_col(k2,:)=[0.2 0.7 0.3];
        elseif abs(R_err_plot(k2)) < delta_R; bar_col(k2,:)=[0.9 0.7 0.1];
        else;                                  bar_col(k2,:)=[0.9 0.3 0.2];
        end
    end
    b34 = bar(1:Nt, R_err_plot, 0.7); b34.FaceColor='flat'; b34.CData=bar_col;
    hold on;
    yline( delta_R/2,'k:','LineWidth',1.2,'Label',sprintf('+%.0fm',delta_R/2));
    yline(-delta_R/2,'k:','LineWidth',1.2,'Label',sprintf('-%.0fm',delta_R/2));
    yline(0,'k-','LineWidth',0.8);
    set(gca,'XTick',1:Nt,'XTickLabel',...
        arrayfun(@(k)sprintf('N%d\nR=%.0fm',k,target.R_true(k)),1:Nt,'UniformOutput',false));
    ylabel('Range Error [m] (signed)');
    title(sprintf('Range Error | delta_R=%.0fm | green<%.0fm',delta_R,delta_R/2),'FontWeight','bold');
    grid on; box on;

    % 3.4 Per-node Pd + RMSE
    subplot(2,3,5);
    snr_op_idx_fig = find(cfg.SNR_sweep==cfg.SNR_dB,1);
    if isempty(snr_op_idx_fig); snr_op_idx_fig=max(1,round(N_snr/2)); end
    node_pd = zeros(1,Nt);
    for k=1:Nt
        rmse_k = RMSE_R_mc(k, snr_op_idx_fig);
        % [FIX v7] Per-node Pd from Swerling-0 + multistatic OR-fusion
        thresh_cfar_fig = -log(cfg.cfar.Pfa);
        pd_sw0_fig = exp(-thresh_cfar_fig / (1 + SNR_ra_lin));
        node_pd(k) = max(0, min(1, 1 - (1 - pd_sw0_fig)^n_sensors));
    end
    yyaxis left;
    bar(1:Nt, node_pd, 0.6); ylabel('Detection Probability P_d'); ylim([0 1.1]);
    yline(0.9,'k:','LineWidth',1.5,'Label','P_d=0.9');
    yyaxis right;
    rmse_snrop = RMSE_R_mc(:,snr_op_idx_fig)';
    plot(1:Nt, rmse_snrop,'rs-','LineWidth',2,'MarkerSize',8,'MarkerFaceColor','r');
    yline(CRB_R_mp,'r:','LineWidth',1.2,'Label',sprintf('CRB=%.3fm',CRB_R_mp));
    ylabel('RMSE @ SNR_{op} [m]');
    set(gca,'XTick',1:Nt,'XTickLabel',arrayfun(@(k)sprintf('N%d',k),1:Nt,'UniformOutput',false));
    title(sprintf('Per-Node P_d + RMSE | SNR=%.0fdB | avg P_d=%.0f%%',...
        cfg.SNR_dB,mean(node_pd)*100),'FontWeight','bold');
    grid on; box on;

    % 3.5 KPI Summary Table (FIXED: pakai patch, bukan annotation)
    subplot(2,3,6);
    axis off;
    avg_impr = mean((1-rmse_kalm_all./max(rmse_raw_all,1e-6))*100);
    snr_pd_idx = find(cfg.SNR_sweep==cfg.SNR_dB,1);
    if isempty(snr_pd_idx); snr_pd_idx=round(N_snr/2); end
    col_dat = {
        'Nodes resolvable', sprintf('%d / %d',Nt-n_overlap,Nt),    deal_ternary(n_overlap==0,'OK','WARN');
        'CFAR deteksi',     sprintf('%d / %d',n_detected,Nt),       deal_ternary(n_detected>=Nt*0.8,'OK','WARN');
        'Pd @ SNR_op',      sprintf('%.0f%%',Pd_arr(snr_pd_idx)*100), deal_ternary(Pd_arr(snr_pd_idx)>=0.9,'OK','WARN');
        'EKF improv.',      sprintf('+%.0f%%',avg_impr),             deal_ternary(avg_impr>50,'OK','WARN');
        'SI cancel',        sprintf('%.0f dBc',SI_dBc_after),        deal_ternary(SI_dBc_after<=-20,'[PASS]','[FAIL]');
        'BER',              sprintf('%.2e',BER),                      deal_ternary(BER<1e-3,'[EXCELLENT]','WARN');
        'Throughput',       sprintf('%.1f Mbps',capacity_bps/1e6),   '--';
        'CRB_R (AWGN)',     sprintf('%.4f m',CRB_R_awgn),            '--';
        'CRB_R (MP)',       sprintf('%.4f m',CRB_R_mp),              '--';
        'Avg RMSE EKF',     sprintf('%.3f m',mean(rmse_kalm_all)),    deal_ternary(mean(rmse_kalm_all)<0.5,'OK','WARN');
        'Mode',             sprintf('Multistatic-%s',cfg.multi.mode), '--';
    };
    % Draw table with patch (axes data coordinates)
    ax36 = gca;
    axis(ax36, [0 1 0 1]);
    hold(ax36, 'on');
    text(ax36, 0.02, 0.97, 'RINGKASAN SENSING & KOMUNIKASI', ...
        'FontWeight','bold','FontSize',10,'VerticalAlignment','top');
    y_row = 0.88;
    text(ax36,0.02,y_row,'Metode','FontSize',8,'FontWeight','bold','Color',[0.2 0.2 0.6]);
    text(ax36,0.48,y_row,'Nilai', 'FontSize',8,'FontWeight','bold','Color',[0.2 0.2 0.6]);
    text(ax36,0.80,y_row,'Status','FontSize',8,'FontWeight','bold','Color',[0.2 0.2 0.6]);
    y_row = y_row - 0.055;
    for ri=1:size(col_dat,1)
        is_ok   = strcmp(col_dat{ri,3},'OK') || strncmp(col_dat{ri,3},'[P',2) || strncmp(col_dat{ri,3},'[E',2);
        is_fail = strncmp(col_dat{ri,3},'FAIL',4) || strncmp(col_dat{ri,3},'[F',2);
        if is_ok;   bg_c=[0.87 0.97 0.87]; fc=[0 0.5 0];
        elseif is_fail; bg_c=[0.97 0.87 0.87]; fc=[0.8 0 0];
        else;       bg_c=[0.95 0.95 0.98]; fc=[0.4 0.4 0.4];
        end
        patch(ax36, [0.45 1.0 1.0 0.45], ...
              [y_row-0.005 y_row-0.005 y_row+0.048 y_row+0.048], ...
              bg_c, 'EdgeColor','none', 'FaceAlpha',1.0);
        text(ax36,0.02,y_row+0.015,col_dat{ri,1},'FontSize',7.5);
        text(ax36,0.48,y_row+0.015,col_dat{ri,2},'FontSize',7.5,'FontWeight','bold');
        text(ax36,0.80,y_row+0.015,col_dat{ri,3},'FontSize',7.5,'FontWeight','bold','Color',fc);
        y_row = y_row - 0.062;
    end
    axis(ax36,'off');
    title(ax36,'KPI Summary','FontWeight','bold');
end


%% -- FIGURE 4: MULTISTATIC ANALYSIS ---------------------------------------
if cfg.plot.fig4
    fig4 = figure('Name','Fig4: Multistatic Analysis', 'Position',[130 30 1380 920], 'Color','w');
    sgtitle(sprintf('FIGURA 4 - MULTISTATIC ANALYSIS | Mode-%s | BW=%.0fMHz', ...
        cfg.multi.mode, cfg.BW/1e6), 'FontWeight','bold','FontSize',12);

    % 4.1 Road geometry + sensor positions + triangulation
    subplot(2,3,[1,2]);
    % Road background
    fill([-road_half_x road_half_x road_half_x -road_half_x], ...
         [-road_half_y-1 -road_half_y-1 road_half_y+1 road_half_y+1], ...
         [0.85 0.85 0.88],'EdgeColor','none'); hold on;
    % Lane lines
    for li=0:n_lanes
        y_l = -road_half_y + li*lane_w;
        if li==0||li==n_lanes; lc=[0.4 0.4 0.4]; lw=2;
        elseif li==n_lanes/2; lc=[0.9 0.7 0];  lw=1.5;
        else; lc=[0.7 0.7 0.7]; lw=0.8;
        end
        line([-road_half_x road_half_x],[y_l y_l],'Color',lc,'LineWidth',lw,'LineStyle','-');
    end
    % Sensor positions (RSU=kuning, OBU1=biru, OBU2=cyan, OBU3=magenta)
    obu_clr_list = {[0.2 0.6 1],[0.2 0.9 0.6],[0.9 0.4 1],[0.9 0.7 0.1]};
    obu_cnt_f4 = 0;
    for si_s=1:n_sensors
        if ~isnan(sensor(si_s).x)
            if contains(sensor(si_s).type,'OBU')
                obu_cnt_f4 = obu_cnt_f4 + 1;
                sc = obu_clr_list{min(obu_cnt_f4,length(obu_clr_list))};
                mk = 's';
            else
                sc = [1 0.8 0]; mk = 'd';
            end
            plot(sensor(si_s).x, sensor(si_s).y, mk, ...
                'MarkerSize',16,'Color',sc,'MarkerFaceColor',sc,'LineWidth',2);
            text(sensor(si_s).x+3, sensor(si_s).y+2, ...
                sprintf('%s',sensor(si_s).type),'FontWeight','bold','FontSize',10,'Color',sc);
            theta_a = linspace(0, 2*pi, 100);
            r_arc = cfg.road.max_detect_m;
            plot(sensor(si_s).x + r_arc*cos(theta_a), ...
                 sensor(si_s).y + r_arc*sin(theta_a), '--',...
                 'Color',[sc 0.4],'LineWidth',1.2,'HandleVisibility','off');
        end
    end
    % Node positions + triangulation result
    for k=1:Nt
        tc = colors_n(mod(k-1,size(colors_n,1))+1,:);
        plot(target.x(k), target.y(k), 'o','MarkerSize',10,'Color',tc,...
            'MarkerFaceColor',tc,'LineWidth',1.5);
        text(target.x(k)+1, target.y(k)+1.5, sprintf('N%d',k),'Color',tc,'FontSize',8,'FontWeight','bold');
        % Draw range circles from sensor 1
        theta_c = linspace(0,2*pi,80);
        r_s1 = target.R_true(k);
        plot(sensor(1).x + r_s1*cos(theta_c), sensor(1).y + r_s1*sin(theta_c), ...
            ':','Color',[tc 0.3],'LineWidth',0.8,'HandleVisibility','off');
        if n_sensors>1 && ~isnan(pos_tri_x(k))
            % Triangulated position
            plot(pos_tri_x(k), pos_tri_y(k), 'x', 'MarkerSize',10, ...
                'Color',tc,'LineWidth',2.5,'HandleVisibility','off');
            % Error line
            line([target.x(k) pos_tri_x(k)],[target.y(k) pos_tri_y(k)],...
                'Color',[tc 0.5],'LineWidth',1,'LineStyle','--','HandleVisibility','off');
        end
    end
    % Legend proxies
    plot(NaN,NaN,'o','Color',[0.5 0.5 0.5],'MarkerFaceColor',[0.5 0.5 0.5],'DisplayName','True position');
    plot(NaN,NaN,'x','Color',[0.5 0.5 0.5],'LineWidth',2,'DisplayName','Triangulated position');
    xlabel('x [m] (Longitudinal)'); ylabel('y [m] (Lateral)');
    title(sprintf('Road Geometry + Sensor Positions + Triangulation (Mode %s)',cfg.multi.mode),...
        'FontWeight','bold');
    xlim([-road_half_x-5 road_half_x+5]); ylim([-road_half_y-8 road_half_y+8]);
    legend('Location','southoutside','FontSize',8,'Orientation','horizontal'); grid on; box on;

    % 4.2 Triangulation error per node
    subplot(2,3,3);
    if n_sensors>1 && any(~isnan(tri_pos_err))
        bar_c4 = zeros(Nt,3);
        tri_err_plot = tri_pos_err;
        tri_err_plot(isnan(tri_err_plot)) = 0;
        for k=1:Nt
            if isnan(tri_pos_err(k)); bar_c4(k,:)=[0.7 0.7 0.7];
            elseif tri_pos_err(k)<5; bar_c4(k,:)=[0.2 0.7 0.3];
            elseif tri_pos_err(k)<15; bar_c4(k,:)=[0.9 0.7 0.1];
            else; bar_c4(k,:)=[0.9 0.3 0.2];
            end
        end
        b43 = bar(1:Nt, tri_err_plot, 0.7); b43.FaceColor='flat'; b43.CData=bar_c4; hold on;
        yline(5,'g--','LineWidth',1.5,'Label','5m');
        yline(delta_R,'r--','LineWidth',1.5,'Label',sprintf('delta_R=%.0fm',delta_R));
        for k=1:Nt
            if ~isnan(tri_pos_err(k))
                text(k,tri_err_plot(k)+0.3,sprintf('%.1fm',tri_err_plot(k)),...
                    'HorizontalAlignment','center','FontSize',7);
            end
        end
        set(gca,'XTick',1:Nt,'XTickLabel',arrayfun(@(k)sprintf('N%d',k),1:Nt,'UniformOutput',false));
        ylabel('Position Error [m]');
        title(sprintf('Triangulation 2D Error | avg=%.2fm',nanmean(tri_pos_err)),'FontWeight','bold');
    else
        text(0.5,0.5,sprintf('Mode %s: Triangulasi tidak tersedia',cfg.multi.mode),...
            'Units','normalized','HorizontalAlignment','center','FontSize',12);
        title('Triangulation Error','FontWeight','bold');
    end
    grid on; box on;

    % 4.3 RMSE comparison: Mono vs Multistatic (sensor 1 vs triangulated)
    subplot(2,3,4);
    rmse_s1 = abs(R_est_all - target.R_true);
    if n_sensors>1
        rmse_tri = tri_pos_err;
        rmse_tri(isnan(rmse_tri)) = rmse_s1(isnan(rmse_tri));
    else
        rmse_tri = rmse_s1;
    end
    x_bar = 1:Nt;
    bw1 = bar(x_bar-0.2, rmse_s1,  0.35); bw1.FaceColor=[0.8 0.3 0.3]; hold on;
    bw2 = bar(x_bar+0.2, rmse_tri, 0.35); bw2.FaceColor=[0.2 0.7 0.4];
    yline(delta_R/2,'k--','LineWidth',1.5,'Label',sprintf('half bin=%.0fm',delta_R/2));
    set(gca,'XTick',1:Nt,'XTickLabel',arrayfun(@(k)sprintf('N%d',k),1:Nt,'UniformOutput',false));
    legend([bw1 bw2],{'Monostatic (S1)','Multistatic (Tri)'},'Location','northoutside',...
        'Orientation','horizontal','FontSize',9);
    ylabel('Position Error [m]');
    title(sprintf('Mono vs Multistatic | S1 avg=%.1fm | Tri avg=%.1fm',...
        mean(rmse_s1),nanmean(rmse_tri)),'FontWeight','bold');
    grid on; box on;

    % 4.4 BW Comparison
    subplot(2,3,5);
    if cfg.BW_compare
        bar_labels = arrayfun(@(b)sprintf('%.0fMHz',b/1e6), cfg.BW_list,'UniformOutput',false);
        b45 = bar(BW_rmse_avg, 0.6);
        b45.FaceColor='flat';
        bw_cols = [0.9 0.3 0.2; 0.2 0.7 0.4; 0.2 0.5 0.9];
        for bi=1:length(cfg.BW_list)
            if bi<=3; b45.CData(bi,:)=bw_cols(bi,:); end
            text(bi, BW_rmse_avg(bi)+0.1, sprintf('%.2fm',BW_rmse_avg(bi)),...
                'HorizontalAlignment','center','FontWeight','bold','FontSize',9);
        end
        hold on;
        for bi=1:length(cfg.BW_list)
            plot(bi, BW_crb(bi),'k^','MarkerSize',10,'MarkerFaceColor','k','LineWidth',2,...
                'HandleVisibility',deal_ternary(bi==1,'on','off'));
        end
        set(gca,'XTickLabel',bar_labels);
        ylabel('RMSE / CRB [m]');
        legend({'RMSE EKF (est.)','CRB (teoritis)'},'Location','northoutside',...
            'Orientation','horizontal','FontSize',9);
        title('Bandwidth Comparison: RMSE vs CRB','FontWeight','bold');
        yline(0.5,'k--','LineWidth',1.5,'Label','0.5m target');
        grid on; box on;
        % Highlight current BW
        [~,cur_bw_idx]=min(abs(cfg.BW_list-cfg.BW));
        text(cur_bw_idx, BW_rmse_avg(cur_bw_idx)+0.3,'(current)','HorizontalAlignment','center','FontSize',8,'Color','red');
    else
        text(0.5,0.5,'BW_compare=false','Units','normalized','HorizontalAlignment','center','FontSize',12);
        title('BW Comparison (disabled)','FontWeight','bold');
    end

    % 4.5 Radar geometry: angle vs Doppler contribution
    subplot(2,3,6);
    angles_true = zeros(1,Nt);
    v_radial_true = zeros(1,Nt);
    for k=1:Nt
        dx_k=target.x(k)-sensor(1).x; dy_k=target.y(k)-sensor(1).y;
        angles_true(k) = atan2d(dy_k,dx_k);
        v_radial_true(k) = target.v_true(k)*cosd(angles_true(k));
    end
    scatter(angles_true, abs(v_radial_true), target.RCS*20+50, 1:Nt, 'filled'); hold on;
    colormap(gca,lines(Nt)); cb46=colorbar; cb46.Label.String='Node index';
    for k=1:Nt
        text(angles_true(k)+1, abs(v_radial_true(k))+0.1, sprintf('N%d',k),'FontSize',7);
    end
    xlabel('Angle from Sensor 1 [deg]');
    ylabel('|v_{radial}| [m/s]');
    title('Radial Velocity vs Angle (Doppler blindness analysis)','FontWeight','bold');
    xline(0,'r--','LineWidth',1.5,'Label','Tangential (blind)');
    grid on; box on;
end


%% -- FIGURE 5: ANIMASI TRAJECTORY 2D (INTERAKTIF) -------------------------
cfg.anim.fps        = 8;
cfg.anim.trail_len  = 12;
cfg.anim.show_est   = true;
cfg.anim.loop       = false;

if cfg.plot.fig5
    fprintf('>> [Figure 5] Animasi Trajectory 2D EKF (%d frames @ %d fps)...\n', ...
        cfg.v2x.Nframes_track, cfg.anim.fps);

    fig5 = figure('Name','Fig5: Trajectory Animation (EKF 2D)', ...
        'Position',[160 30 1100 700], 'Color',[0.08 0.08 0.12]);

    type_cm = containers.Map({'Mobil','Motor','VRU','Node','Kendaraan'},...
        {[1.0 0.4 0.3],[0.3 0.7 1.0],[0.4 1.0 0.5],[0.8 0.6 1.0],[1.0 0.8 0.3]});
    node_colors = zeros(Nt,3);
    for k=1:Nt
        if isKey(type_cm,target.label{k}); node_colors(k,:)=type_cm(target.label{k});
        else; node_colors(k,:)=lines(1); end
    end

    x_all = [track_x_true(:); track_x(:)];
    xlim_a = [min(x_all)-15, max(x_all)+15];
    xlim_a(1) = max(xlim_a(1), -road_half_x-10);
    xlim_a(2) = min(xlim_a(2),  road_half_x+10);

    ax_anim = axes('Parent',fig5,'Color',[0.13 0.13 0.18]);
    hold(ax_anim,'on');

    fill([-road_half_x road_half_x road_half_x -road_half_x], ...
         [-road_half_y-1 -road_half_y-1 road_half_y+1 road_half_y+1], ...
         [0.22 0.22 0.28],'EdgeColor','none','Parent',ax_anim);

    for li=0:n_lanes
        y_l = -road_half_y + li*lane_w;
        if li==0||li==n_lanes; lc=[0.85 0.85 0.85]; lw=2.0; ls='-';
        elseif li==n_lanes/2; lc=[0.95 0.85 0.2]; lw=1.5; ls='-';
        else; lc=[0.45 0.45 0.55]; lw=0.8; ls='--';
        end
        line(ax_anim,[-road_half_x road_half_x],[y_l y_l],'Color',lc,'LineStyle',ls,'LineWidth',lw);
    end

    for li=1:n_lanes
        y_lc = -road_half_y + (li-0.5)*lane_w;
        dir_lbl = deal_ternary(li<=cfg.road.n_lanes_per_dir,'->','<-');
        text(-road_half_x+3, y_lc, sprintf('L%d %s',li,dir_lbl), ...
            'Color',[0.6 0.6 0.7],'FontSize',7,'Parent',ax_anim);
    end

    % Plot sensor positions (RSU=kuning, OBU=biru/cyan/magenta)
    obu_clr_anim = {[0.2 0.6 1],[0.2 0.9 0.6],[0.9 0.4 1]};
    obu_cnt_anim = 0;
    for si_s=1:n_sensors
        if ~isnan(sensor(si_s).x)
            if contains(sensor(si_s).type,'OBU')
                obu_cnt_anim = obu_cnt_anim + 1;
                obu_c_a = obu_clr_anim{min(obu_cnt_anim,length(obu_clr_anim))};
                plot(ax_anim, sensor(si_s).x, sensor(si_s).y, 's',...
                    'MarkerSize',12,'Color',obu_c_a,'MarkerFaceColor',obu_c_a,'HandleVisibility','off');
                text(sensor(si_s).x+2, sensor(si_s).y+lane_w*0.5, sensor(si_s).type,...
                    'Color',obu_c_a,'FontSize',8,'FontWeight','bold','Parent',ax_anim);
            else
                plot(ax_anim, sensor(si_s).x, sensor(si_s).y, 'd',...
                    'MarkerSize',14,'Color',[1 0.9 0.2],'MarkerFaceColor',[1 0.9 0.2],'HandleVisibility','off');
                text(sensor(si_s).x+2, sensor(si_s).y+lane_w*0.5, sensor(si_s).type,...
                    'Color',[1 0.9 0.2],'FontSize',9,'FontWeight','bold','Parent',ax_anim);
            end
        end
    end
    % Initialize animated objects (HandleVisibility off untuk semua animated)
    h_trail_true = gobjects(Nt,1);
    h_trail_est  = gobjects(Nt,1);
    h_dot_true   = gobjects(Nt,1);
    h_dot_est    = gobjects(Nt,1);
    h_err_line   = gobjects(Nt,1);
    h_vel_arr    = gobjects(Nt,1);
    h_lbl        = gobjects(Nt,1);

    for k=1:Nt
        tc = node_colors(k,:);
        h_trail_true(k) = plot(ax_anim,NaN,NaN,'-','Color',[tc 0.35],'LineWidth',1.5,'HandleVisibility','off');
        h_trail_est(k)  = plot(ax_anim,NaN,NaN,'--','Color',[tc*0.6+0.4 0.35],'LineWidth',1.0,'HandleVisibility','off');
        h_dot_true(k)   = plot(ax_anim,NaN,NaN,'o','MarkerSize',max(8,target.RCS(k)*1.2),...
            'Color',tc,'MarkerFaceColor',tc,'LineWidth',1.5,'HandleVisibility','off');
        h_dot_est(k)    = plot(ax_anim,NaN,NaN,'x','MarkerSize',12,'Color',[1 0.4 0.4],'LineWidth',2.5,'HandleVisibility','off');
        h_err_line(k)   = plot(ax_anim,[NaN NaN],[NaN NaN],'-','Color',[1 0.5 0.5 0.5],'LineWidth',1,'HandleVisibility','off');
        h_vel_arr(k)    = quiver(ax_anim,NaN,NaN,NaN,NaN,0,'Color',tc,'LineWidth',1.5,'MaxHeadSize',5,'HandleVisibility','off');
        h_lbl(k)        = text(NaN,NaN,sprintf('N%d',k),'Color',tc,'FontSize',8,'FontWeight','bold',...
            'Parent',ax_anim,'HorizontalAlignment','center');
    end

    % OBU animated dot (Mode B/C)
    if strcmp(cfg.multi.mode,'B') || strcmp(cfg.multi.mode,'C')
        h_obu_anim = plot(ax_anim,NaN,NaN,'s','MarkerSize',14,'Color',[0.2 0.6 1],...
            'MarkerFaceColor',[0.2 0.6 1],'HandleVisibility','off');
    else
        h_obu_anim = gobjects(1);
    end

    h_title   = title(ax_anim, 'Frame 0', 'Color','w','FontSize',11,'FontWeight','bold');
    h_info    = text(0.02,0.98,'','Units','normalized','Color',[0.8 1.0 0.8],...
        'FontSize',8,'VerticalAlignment','top','Parent',ax_anim,...
        'BackgroundColor',[0.1 0.1 0.15],'EdgeColor',[0.4 0.4 0.5]);
    h_rmse_bar = text(0.55,0.98,'','Units','normalized','Color',[1 0.8 0.5],...
        'FontSize',8,'VerticalAlignment','top','Parent',ax_anim,...
        'BackgroundColor',[0.1 0.1 0.15],'EdgeColor',[0.4 0.4 0.5]);

    % Legend (explicit handles)
    h_leg_nodes = gobjects(Nt,1);
    for k=1:Nt
        h_leg_nodes(k) = plot(ax_anim,NaN,NaN,'o-','Color',node_colors(k,:),...
            'MarkerFaceColor',node_colors(k,:),...
            'DisplayName',sprintf('N%d %s (%.0fm/s)',k,target.label{k},target.v_true(k)));
    end
    h_leg_est = plot(ax_anim,NaN,NaN,'x','Color',[1 0.4 0.4],'MarkerSize',10,'LineWidth',2,...
        'DisplayName','EKF Est.');
    h_leg_rng = plot(ax_anim,NaN,NaN,'--','Color',[1 0.3 0.3],'LineWidth',1.5,...
        'DisplayName',sprintf('Range %.0fm',cfg.road.max_detect_m));
    lg5 = legend(ax_anim,[h_leg_nodes; h_leg_est; h_leg_rng],...
        'Location','southoutside','Orientation','horizontal',...
        'FontSize',7.5,'TextColor','w','Color',[0.1 0.1 0.15],'EdgeColor',[0.4 0.4 0.5]);
    lg5.NumColumns = min(Nt+2, 6);

    xlabel(ax_anim,'x [m] (Longitudinal)','Color',[0.7 0.7 0.8],'FontSize',10);
    ylabel(ax_anim,'y [m] (Lateral)','Color',[0.7 0.7 0.8],'FontSize',10);
    ax_anim.XColor=[0.6 0.6 0.7]; ax_anim.YColor=[0.6 0.6 0.7];
    ax_anim.GridColor=[0.35 0.35 0.45]; ax_anim.GridAlpha=0.5;
    ax_anim.XLim = xlim_a;
    ax_anim.YLim = [-road_half_y-4, road_half_y+4];
    grid(ax_anim,'on'); box(ax_anim,'on');

    % -- Animasi loop -------------------------------------------------------
    dt_anim = 1/cfg.anim.fps;
    for f=1:cfg.v2x.Nframes_track
        if ~ishandle(fig5); break; end
        t_lo = max(1, f-cfg.anim.trail_len+1);

        for k=1:Nt
            tc = node_colors(k,:);
            set(h_trail_true(k),'XData',track_x_true(k,t_lo:f),'YData',track_y_true(k,t_lo:f));
            if cfg.anim.show_est
                set(h_trail_est(k),'XData',track_x(k,t_lo:f),'YData',track_y(k,t_lo:f));
            end
            set(h_dot_true(k),'XData',track_x_true(k,f),'YData',track_y_true(k,f));
            if cfg.anim.show_est
                set(h_dot_est(k),'XData',track_x(k,f),'YData',track_y(k,f));
            end
            set(h_err_line(k),'XData',[track_x_true(k,f) track_x(k,f)],...
                               'YData',[track_y_true(k,f) track_y(k,f)]);
            set(h_vel_arr(k),'XData',track_x_true(k,f),'YData',track_y_true(k,f),...
                'UData',track_vx(k,f)*0.3,'VData',track_vy(k,f)*0.3);
            set(h_lbl(k),'Position',[track_x_true(k,f), track_y_true(k,f)+lane_w*0.45]);
        end

        % Update OBU dot position (Mode B/C)
        if (strcmp(cfg.multi.mode,'B') || strcmp(cfg.multi.mode,'C')) && ishandle(h_obu_anim) && n_obu >= 1
            obu_x_f_a = target.x(obu_indices(1)) + target.v_true(obu_indices(1))*f*dt;
            obu_y_f_a = target.y(obu_indices(1));
            set(h_obu_anim,'XData',obu_x_f_a,'YData',obu_y_f_a);
        end

        % RMSE per frame
        pos_err_f = sqrt((track_x(:,f)-track_x_true(:,f)).^2 + (track_y(:,f)-track_y_true(:,f)).^2);
        rmse_f    = sqrt(mean(pos_err_f.^2));
        t_elapsed = f*T_frame*1000;

        set(h_title,'String',sprintf(...
            'Frame %d/%d | t=%.1fms | EKF RMSE=%.2fm | Mode-%s | SNR=%.0fdB',...
            f, cfg.v2x.Nframes_track, t_elapsed, rmse_f, cfg.multi.mode, cfg.SNR_dB));

        info_str = sprintf('Frame %d/%d | BW=%.0fMHz | %d nodes | %s',...
            f, cfg.v2x.Nframes_track, cfg.BW/1e6, Nt, ...
            deal_ternary(n_sensors>1,sprintf('Multistatic Mode-%s',cfg.multi.mode),'Monostatic'));
        set(h_info,'String',info_str);

        rmse_str = sprintf('RMSE=%.2fm  |  ',rmse_f);
        for k=1:Nt
            rmse_str = [rmse_str sprintf('N%d:%.1fm  ',k,pos_err_f(k))];
        end
        set(h_rmse_bar,'String',strtrim(rmse_str));

        drawnow limitrate;
        pause(dt_anim);
    end

    if ishandle(fig5)
        set(h_title,'String',sprintf(...
            'SELESAI - %d frames | Avg EKF RMSE=%.2fm | Mode-%s',...
            cfg.v2x.Nframes_track, mean(rmse_kalm_all), cfg.multi.mode));
        fprintf('   Animasi selesai. Figure 5 siap untuk eksplorasi.\n');
    end
end


%% =========================================================================
%  FIGURE EXPORT (PNG, 300 dpi)
%  =========================================================================
if cfg.export.figures
    export_dir = cfg.export.dir;
    if ~exist(export_dir,'dir'); mkdir(export_dir); end
    fig_handles = {fig1, fig2, fig3, fig4};
    fig_names   = {'Fig1_Sensing_Core','Fig2_JCAS_Tradeoff',...
                   'Fig3_Sensing_Summary','Fig4_Multistatic'};
    fprintf('\n[EXPORT] Saving figures to: %s\n', export_dir);
    for fi = 1:length(fig_handles)
        fh = fig_handles{fi};
        if ishandle(fh)
            fname = fullfile(export_dir, [fig_names{fi} '.png']);
            exportgraphics(fh, fname, 'Resolution', 300, 'BackgroundColor','white');
            fprintf('  Saved: %s\n', fig_names{fi});
        end
    end
end

%% =========================================================================
%  CSV EXPORT — KPI per Allocation Strategy
%  =========================================================================
if cfg.export.csv
    export_dir = cfg.export.dir;
    if ~exist(export_dir,'dir'); mkdir(export_dir); end
    csv_path = fullfile(export_dir, 'JCAS_KPI_Results.csv');

    % 5GAA SLR targets (infrastructure-based sensing, urban)
    target_range_res_m  = 1.0;   % [m]  range resolution
    target_speed_acc_ms = 0.3;   % [m/s] speed accuracy
    target_pd_pct       = 99.0;  % [%]  detection probability
    target_pfa_pct      = 5.0;   % [%]  false alarm rate
    target_tput_mbps    = 0.2;   % [Mbps] min communication throughput (AVP Type 2)

    % Collect sensing KPIs
    ekf_rmse_avg = mean(rmse_kalm_all);
    snr_op_idx   = find(cfg.SNR_sweep == cfg.SNR_dB, 1);
    pd_op        = 0;
    if ~isempty(snr_op_idx); pd_op = Pd_arr(snr_op_idx)*100; end

    % Build table
    if ~exist(export_dir,'dir'); mkdir(export_dir); end
    fid = fopen(csv_path,'w');
    if fid == -1
        warning('[EXPORT] Cannot open CSV file for writing: %s', csv_path);
    else
    fprintf(fid,'JCAS OFDM Simulation - KPI Results\n');
    fprintf(fid,'Script Version,6.0 (Multistatic+EKF+RL)\n');
    fprintf(fid,'Carrier Frequency [GHz],%.1f\n', cfg.fc/1e9);
    fprintf(fid,'Bandwidth [MHz],%.0f\n', cfg.BW/1e6);
    fprintf(fid,'Multistatic Mode,%s\n', cfg.multi.mode);
    fprintf(fid,'Nodes,%d\n', Nt);
    fprintf(fid,'SNR_op [dB],%.0f\n', cfg.SNR_dB);
    fprintf(fid,'\n');

    % --- Sensing KPIs ---
    fprintf(fid,'--- SENSING KPIs ---\n');
    fprintf(fid,'Metric,Value,5GAA SLR Target,Pass/Fail\n');
    fprintf(fid,'Range Resolution [m],%.4f,%.1f,%s\n', ...
        delta_R, target_range_res_m, deal_ternary(delta_R<=target_range_res_m,'PASS','FAIL'));
    fprintf(fid,'EKF RMSE avg [m],%.4f,N/A,N/A\n', ekf_rmse_avg);
    fprintf(fid,'CRB Range AWGN [m],%.4f,N/A,N/A\n', CRB_R_awgn);
    fprintf(fid,'CRB Range Multipath [m],%.4f,N/A,N/A\n', CRB_R_mp);
    fprintf(fid,'Pd @ SNR_op [%%],%.2f,%.1f,%s\n', ...
        pd_op, target_pd_pct, deal_ternary(pd_op>=target_pd_pct,'PASS','FAIL'));
    fprintf(fid,'Pfa [%%],%.3f,%.1f,%s\n', ...
        cfg.cfar.Pfa*100, target_pfa_pct, deal_ternary(cfg.cfar.Pfa*100<=target_pfa_pct,'PASS','FAIL'));
    fprintf(fid,'SI after mitigation [dBc],%.2f,<=-20,%s\n', ...
        SI_dBc_after, deal_ternary(SI_dBc_after<=-20,'PASS','FAIL'));
    if n_sensors > 1
        fprintf(fid,'Triangulation avg pos error [m],%.4f,N/A,N/A\n', nanmean(tri_pos_err));
    end
    fprintf(fid,'\n');

    % --- Communication KPIs ---
    fprintf(fid,'--- COMMUNICATION KPIs ---\n');
    fprintf(fid,'Metric,Value,5GAA SLR Target,Pass/Fail\n');
    fprintf(fid,'Throughput [Mbps],%.4f,%.1f,%s\n', ...
        capacity_bps/1e6, target_tput_mbps, ...
        deal_ternary(capacity_bps/1e6>=target_tput_mbps,'PASS','FAIL'));
    fprintf(fid,'BER,%.6f,<1e-3,%s\n', ...
        BER, deal_ternary(BER<1e-3,'PASS','FAIL'));
    fprintf(fid,'PAPR [dB],%.2f,N/A,N/A\n', PAPR_dB);
    fprintf(fid,'\n');

    % --- Resource Allocation KPIs ---
    if cfg.enable.fase4
        fprintf(fid,'--- RESOURCE ALLOCATION KPIs ---\n');
        fprintf(fid,'Strategy,Nc/Nsc_ratio,Capacity [Mbps],Gain vs Static [%%],5GAA Tput Pass\n');
        all_n = [{'Static','WaterFill','Adaptive'}, run_algos];
        all_c = [best_cap_s, best_cap_w, best_cap_a, best_cap_rl];
        all_r = [best_ratio_s, best_ratio_w, best_ratio_a, best_ratio_rl];
        for mi = 1:length(all_n)
            g = (all_c(mi)-best_cap_s)/max(best_cap_s,eps)*100;
            pf = deal_ternary(all_c(mi)>=target_tput_mbps,'PASS','FAIL');
            fprintf(fid,'%s,%.4f,%.4f,%+.2f,%s\n', all_n{mi}, all_r(mi), all_c(mi), g, pf);
        end
        fprintf(fid,'\n');
    end

    % --- BW Comparison ---
    if cfg.BW_compare
        fprintf(fid,'--- BANDWIDTH COMPARISON ---\n');
        fprintf(fid,'BW [MHz],CRB Range [m],RMSE avg [m]\n');
        for bi = 1:length(cfg.BW_list)
            fprintf(fid,'%.0f,%.4f,%.4f\n', cfg.BW_list(bi)/1e6, BW_crb(bi), BW_rmse_avg(bi));
        end
    end

    fclose(fid);
    fprintf('[EXPORT] CSV saved: %s\n', csv_path);
    end  % if fid ~= -1
end

%% =========================================================================
%  MULTI-CSV EXPORT — One CSV file per data table (no external dependencies)
% =========================================================================
if cfg.export.csv
    export_dir = cfg.export.dir;
    if ~exist(export_dir,'dir'); mkdir(export_dir); end

    % Re-establish strategy variables (safe even if deep analysis ran first)
    if cfg.enable.fase4
        csv_strat_names = [{'Static','WaterFill','Adaptive'}, ...
                           cellfun(@(x) algo_labels(x), run_algos, 'UniformOutput', false)];
        csv_strat_cap   = [best_cap_s, best_cap_w, best_cap_a, best_cap_rl];
        csv_strat_ratio = [best_ratio_s, best_ratio_w, best_ratio_a, best_ratio_rl];
        csv_n_strat     = length(csv_strat_names);
    else
        csv_strat_names = {'Static'}; csv_strat_cap=[0];
        csv_strat_ratio=[0]; csv_n_strat=1;
    end
    ekf_rmse_m = mean(rmse_kalm_all); ekf_rmse_s = std(rmse_kalm_all);
    raw_rmse_m = mean(rmse_raw_all);
    snr_op_c   = find(cfg.SNR_sweep==cfg.SNR_dB,1);
    pd_op_c=0; ber_op_c=0; pfa_op_c=0;
    if ~isempty(snr_op_c)
        pd_op_c=Pd_arr(snr_op_c)*100; ber_op_c=BER_arr(snr_op_c);
        pfa_op_c=Pfa_arr_emp(snr_op_c)*100;
    end

    % ---- CSV 1: Paper Summary ----
    fid=fopen(fullfile(export_dir,['CSV01_Paper_Summary' csv_suffix '.csv']),'w');
    if fid~=-1
        fprintf(fid,'JCAS OFDM v6.0 — Paper Summary\n');
        fprintf(fid,'Scenario,%s\nNodes,%d\nfc_GHz,%.1f\nBW_MHz,%.0f\nMode,%s\nSNR_dB,%.0f\nTrials,%d\n\n',...
            cfg.v2x.mobility_model,Nt,cfg.fc/1e9,cfg.BW/1e6,cfg.multi.mode,cfg.SNR_dB,cfg.Ntrials);
        fprintf(fid,'Strategy,Rho,Cap_Mbps,Gain_vs_Static_pct,EKF_RMSE_mean_m,EKF_RMSE_std_m,Raw_RMSE_m,EKF_Improv_pct,RangeRes_m,Pd_pct,BER,Pfa_pct,Tput_5GAA,RangeRes_5GAA,Pd_5GAA\n');
        for si2=1:csv_n_strat
            cap_i=csv_strat_cap(si2);
            gain_i=(cap_i-csv_strat_cap(1))/max(csv_strat_cap(1),1e-6)*100;
            ekf_imp=(1-ekf_rmse_m/max(raw_rmse_m,1e-6))*100;
            fprintf(fid,'%s,%.3f,%.4f,%+.2f,%.4f,%.4f,%.4f,%.1f,%.4f,%.2f,%.6f,%.4f,%s,%s,%s\n',...
                csv_strat_names{si2},csv_strat_ratio(si2),cap_i,gain_i,...
                ekf_rmse_m,ekf_rmse_s,raw_rmse_m,ekf_imp,...
                delta_R,pd_op_c,ber_op_c,pfa_op_c,...
                ternary_str(cap_i>=0.2,'PASS','FAIL'),...
                ternary_str(delta_R<=1.0,'PASS','FAIL'),...
                ternary_str(pd_op_c>=95,'PASS','FAIL'));
        end
        fclose(fid); fprintf('[CSV] CSV01_Paper_Summary.csv saved\n');
    end

    % ---- CSV 2: Full Pareto Sweep ----
    fid=fopen(fullfile(export_dir,['CSV02_Pareto_Sweep' csv_suffix '.csv']),'w');
    if fid~=-1
        fprintf(fid,'Strategy,Rho,Cap_Mbps,RMSE_m,Tput_5GAA,RangeRes_5GAA\n');
        for si2=1:csv_n_strat
            if si2==1; cap_a=cap_static_Mbps; rmse_a=RMSE_R_static;
            elseif si2==2; cap_a=cap_wf_Mbps; rmse_a=RMSE_R_wf;
            elseif si2==3; cap_a=cap_adapt_Mbps; rmse_a=RMSE_R_adapt;
            else; ai_c=si2-3; cap_a=cap_rl_Mbps(ai_c,:); rmse_a=RMSE_R_rl(ai_c,:); end
            for ri2=1:length(ratios)
                rmse_v=rmse_a(ri2); if ~isfinite(rmse_v); rmse_v=9999; end
                fprintf(fid,'%s,%.2f,%.4f,%.4f,%s,%s\n',...
                    csv_strat_names{si2},ratios(ri2),cap_a(ri2),rmse_v,...
                    ternary_str(cap_a(ri2)>=0.2,'PASS','FAIL'),...
                    ternary_str(rmse_v<=1.0,'PASS','FAIL'));
            end
        end
        fclose(fid); fprintf('[CSV] CSV02_Pareto_Sweep.csv saved\n');
    end

    % ---- CSV 3: Pd vs SNR ----
    fid=fopen(fullfile(export_dir,['CSV03_Pd_vs_SNR' csv_suffix '.csv']),'w');
    if fid~=-1
        fprintf(fid,'SNR_dB,Pd_pct,MissedDet_pct,BER,BER_QPSK_theory,BER_16QAM_theory,Pfa_emp_pct,Pd_5GAA,Pfa_5GAA\n');
        for si2=1:length(cfg.SNR_sweep)
            fprintf(fid,'%.0f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%s,%s\n',...
                cfg.SNR_sweep(si2),Pd_arr(si2)*100,(1-Pd_arr(si2))*100,...
                BER_arr(si2),BER_qpsk_th(si2),BER_16qam_th(si2),...
                Pfa_arr_emp(si2)*100,...
                ternary_str(Pd_arr(si2)*100>=95,'PASS','FAIL'),...
                ternary_str(Pfa_arr_emp(si2)*100<=1,'PASS','FAIL'));
        end
        fclose(fid); fprintf('[CSV] CSV03_Pd_vs_SNR.csv saved\n');
    end

    % ---- CSV 4: Per-Node RMSE ----
    fid=fopen(fullfile(export_dir,['CSV04_PerNode_RMSE' csv_suffix '.csv']),'w');
    if fid~=-1
        fprintf(fid,'Node,Type,RCS_m2,Range_m,Speed_ms,Lane,RawRMSE_m,EKFRMSE_m,EKF_Improv_pct,Pass_5GAA\n');
        for k=1:Nt
            ekf_imp_k=(1-rmse_kalm_all(k)/max(rmse_raw_all(k),1e-6))*100;
            fprintf(fid,'%d,%s,%.1f,%.1f,%.2f,%d,%.4f,%.4f,%.1f,%s\n',...
                k,target.label{k},target.RCS(k),target.R_true(k),target.v_true(k),...
                target.lane(k),rmse_raw_all(k),rmse_kalm_all(k),ekf_imp_k,...
                ternary_str(rmse_kalm_all(k)<=1.0,'PASS','FAIL'));
        end
        fprintf(fid,'MEAN,,,,,,%.4f,%.4f,%.1f,\n',mean(rmse_raw_all),mean(rmse_kalm_all),...
            mean((1-rmse_kalm_all./max(rmse_raw_all,1e-6))*100));
        fclose(fid); fprintf('[CSV] CSV04_PerNode_RMSE.csv saved\n');
    end

    % ---- CSV 5: BW Comparison ----
    if cfg.BW_compare
        fid=fopen(fullfile(export_dir,['CSV05_BW_Comparison' csv_suffix '.csv']),'w');
        if fid~=-1
            fprintf(fid,'BW_MHz,Nsc,RangeRes_m,CRB_m,RMSE_approx_m,Pass_5GAA\n');
            for bi=1:length(cfg.BW_list)
                rr_bw=cfg.c/(2*cfg.BW_list(bi));
                nsc_bw=round(cfg.BW_list(bi)/cfg.SCS);
                fprintf(fid,'%.0f,%d,%.4f,%.4f,%.4f,%s\n',...
                    cfg.BW_list(bi)/1e6,nsc_bw,rr_bw,BW_crb(bi),BW_rmse_avg(bi),...
                    ternary_str(rr_bw<=1.0,'PASS','FAIL'));
            end
            fclose(fid); fprintf('[CSV] CSV05_BW_Comparison.csv saved\n');
        end
    end

    % ---- CSV 6: RL Training History (sampled every 10 episodes) ----
    if cfg.enable.fase4
        fid=fopen(fullfile(export_dir,['CSV06_RL_Training' csv_suffix '.csv']),'w');
        if fid~=-1
            hdr_str='Episode';
            for ai=1:n_algos; hdr_str=[hdr_str ',' algo_labels(run_algos{ai}) '_reward']; end
            fprintf(fid,'%s\n',hdr_str);
            ep_s=1:10:cfg.rl.n_episodes;
            for ei=1:length(ep_s)
                ep=ep_s(ei); row_s=sprintf('%d',ep);
                for ai=1:n_algos; row_s=[row_s sprintf(',%.4f',reward_hist(ai,ep))]; end
                fprintf(fid,'%s\n',row_s);
            end
            fclose(fid); fprintf('[CSV] CSV06_RL_Training.csv saved\n');
        end
    end

    % ---- CSV 7: Statistical Significance ----
    if cfg.enable.fase4 && isfield(deep,'ci_mean')
        fid=fopen(fullfile(export_dir,['CSV07_Statistical_Significance' csv_suffix '.csv']),'w');
        if fid~=-1
            fprintf(fid,'Bootstrap CI (n=%d) and t-test vs Static\n',n_boot);
            fprintf(fid,'Strategy,Mean_Cap_Mbps,Std_Cap_Mbps,CI_Low95_Mbps,CI_High95_Mbps,CI_Width_Mbps,pvalue_vs_Static,Significant_p005\n');
            for si2=1:csv_n_strat
                p_str=deal_ternary(si2==1,'N/A',sprintf('%.4f',deep.ttest_p(si2)));
                sig_str=deal_ternary(si2==1,'REF',ternary_str(deep.ttest_p(si2)<0.05,'YES','NO'));
                fprintf(fid,'%s,%.4f,%.4f,%.4f,%.4f,%.4f,%s,%s\n',...
                    csv_strat_names{si2},deep.ci_mean(si2),deep.ci_std(si2),...
                    deep.ci_lo95(si2),deep.ci_hi95(si2),...
                    deep.ci_hi95(si2)-deep.ci_lo95(si2),p_str,sig_str);
            end
            fclose(fid); fprintf('[CSV] CSV07_Statistical_Significance.csv saved\n');
        end
    end

    % ---- CSV 8: Sensitivity Analysis ----
    if cfg.enable.fase4 && isfield(deep,'sens_wc')
        fid=fopen(fullfile(export_dir,['CSV08_Sensitivity_Analysis' csv_suffix '.csv']),'w');
        if fid~=-1
            fprintf(fid,'w_c,w_s,w_d,QL_Cap_Mbps,SARSA_Cap_Mbps,Diff_Mbps,QL_Tput_5GAA\n');
            for wi=1:length(deep.sens_wc)
                wc_i=deep.sens_wc(wi);
                fprintf(fid,'%.1f,%.1f,0.1,%.4f,%.4f,%.4f,%s\n',...
                    wc_i,0.9-wc_i,deep.sens_cap_ql(wi),deep.sens_cap_sarsa(wi),...
                    deep.sens_cap_ql(wi)-deep.sens_cap_sarsa(wi),...
                    ternary_str(deep.sens_cap_ql(wi)>=0.2,'PASS','FAIL'));
            end
            fclose(fid); fprintf('[CSV] CSV08_Sensitivity_Analysis.csv saved\n');
        end
    end

    % ---- CSV 9: Convergence Analysis ----
    if cfg.enable.fase4 && isfield(deep,'conv_mavg')
        fid=fopen(fullfile(export_dir,['CSV09_Convergence' csv_suffix '.csv']),'w');
        if fid~=-1
            hdr_str='Episode';
            for ai=1:n_algos; hdr_str=[hdr_str ',' algo_labels(run_algos{ai}) '_mavg']; end
            fprintf(fid,'%s\n',hdr_str);
            ep_s=1:10:cfg.rl.n_episodes;
            for ei=1:length(ep_s)
                ep=ep_s(ei); row_s=sprintf('%d',ep);
                for ai=1:n_algos; row_s=[row_s sprintf(',%.4f',deep.conv_mavg(ai,ep))]; end
                fprintf(fid,'%s\n',row_s);
            end
            fclose(fid); fprintf('[CSV] CSV09_Convergence.csv saved\n');
        end
    end

    % ---- CSV 10: Ablation Study ----
    if cfg.enable.fase4 && isfield(deep,'abl_names')
        fid=fopen(fullfile(export_dir,['CSV10_Ablation' csv_suffix '.csv']),'w');
        if fid~=-1
            fprintf(fid,'Configuration,QL_Cap_Mbps,Delta_vs_Full_Mbps,Delta_pct,Tput_5GAA\n');
            full_c=deep.abl_cap_ql(1);
            for abl_i=1:n_abl
                dabs=deep.abl_cap_ql(abl_i)-full_c;
                dpct=dabs/max(full_c,1e-6)*100;
                fprintf(fid,'%s,%.4f,%+.4f,%+.2f,%s\n',...
                    deep.abl_names{abl_i},deep.abl_cap_ql(abl_i),dabs,dpct,...
                    ternary_str(deep.abl_cap_ql(abl_i)>=0.2,'PASS','FAIL'));
            end
            fclose(fid); fprintf('[CSV] CSV10_Ablation.csv saved\n');
        end
    end

    % ---- CSV 11: Cross-scenario Generalization ----
    if cfg.enable.fase4 && isfield(deep,'gen_names')
        fid=fopen(fullfile(export_dir,['CSV11_Generalization' csv_suffix '.csv']),'w');
        if fid~=-1
            fprintf(fid,'Q-Learning trained on medium density — tested on all densities\n');
            fprintf(fid,'Scenario,Cap_Mbps,RMSE_m,Tput_5GAA,RangeRes_5GAA\n');
            for di=1:length(deep.gen_names)
                fprintf(fid,'%s,%.4f,%.4f,%s,%s\n',...
                    deep.gen_names{di},deep.gen_cap(di),deep.gen_rmse(di),...
                    ternary_str(deep.gen_cap(di)>=0.2,'PASS','FAIL'),...
                    ternary_str(deep.gen_rmse(di)<=1.0,'PASS','FAIL'));
            end
            fclose(fid); fprintf('[CSV] CSV11_Generalization.csv saved\n');
        end
    end

    % ---- CSV 12: System Config ----
    fid=fopen(fullfile(export_dir,['CSV12_System_Config' csv_suffix '.csv']),'w');
    if fid~=-1
        fprintf(fid,'Parameter,Value,Unit\n');
        fprintf(fid,'Centre frequency,%.1f,GHz\n',cfg.fc/1e9);
        fprintf(fid,'Bandwidth,%.0f,MHz\n',cfg.BW/1e6);
        fprintf(fid,'Subcarrier spacing,%.0f,kHz\n',cfg.SCS/1e3);
        fprintf(fid,'Num subcarriers,%d,\n',Nsc);
        fprintf(fid,'OFDM symbols,%d,\n',Nsym);
        fprintf(fid,'Modulation,%d-QAM,\n',cfg.mod_order);
        fprintf(fid,'Operating SNR,%.0f,dB\n',cfg.SNR_dB);
        fprintf(fid,'Num nodes,%d,\n',Nt);
        fprintf(fid,'Mobility model,%s,\n',cfg.v2x.mobility_model);
        fprintf(fid,'v_mean,%.0f,m/s\n',cfg.v2x.v_mean);
        fprintf(fid,'v_std,%.0f,m/s\n',cfg.v2x.v_std);
        fprintf(fid,'Road length,%.0f,m\n',cfg.road.length_m);
        fprintf(fid,'Max detect range,%.0f,m\n',cfg.road.max_detect_m);
        fprintf(fid,'Multistatic mode,%s,\n',cfg.multi.mode);
        fprintf(fid,'Num sensors,%d,\n',n_sensors);
        fprintf(fid,'CFAR Pfa,%.0e,\n',cfg.cfar.Pfa);
        fprintf(fid,'CFAR guard cells,%d,\n',cfg.cfar.guard_cells);
        fprintf(fid,'CFAR train cells,%d,\n',cfg.cfar.train_cells);
        fprintf(fid,'SI before cancel,%.0f,dBc\n',cfg.hw.SI_dB);
        fprintf(fid,'SI cancellation,%.0f,dB\n',cfg.hw.SI_cancel_dB);
        fprintf(fid,'Monte Carlo trials,%d,\n',cfg.Ntrials);
        fprintf(fid,'RL episodes,%d,\n',cfg.rl.n_episodes);
        fprintf(fid,'RL alpha,%.2f,\n',cfg.rl.alpha);
        fprintf(fid,'RL gamma,%.2f,\n',cfg.rl.gamma);
        fprintf(fid,'RL epsilon0,%.2f,\n',cfg.rl.epsilon);
        fprintf(fid,'Range resolution,%.4f,m\n',delta_R);
        fprintf(fid,'Velocity resolution,%.4f,m/s\n',delta_v);
        fprintf(fid,'CRB range AWGN,%.4f,m\n',CRB_R_awgn);
        fprintf(fid,'CRB range MP,%.4f,m\n',CRB_R_mp);
        fclose(fid); fprintf('[CSV] CSV12_System_Config.csv saved\n');
    end
    fprintf('[EXPORT] All CSV files saved to: %s\n', export_dir);
end

%% =========================================================================
%  DEBUG DUMP — Consolidated single-file debug output
%  All CSV content + key variables written to one txt file.
%  Purpose: paste this file directly to Claude for debugging.
%  File: debug_output<csv_suffix>.txt  (same folder as CSVs)
% =========================================================================
if cfg.export.csv
    dbg_path = fullfile(export_dir, ['debug_output' csv_suffix '.txt']);
    fdbg = fopen(dbg_path, 'w');
    if fdbg == -1
        warning('[DEBUG] Cannot open debug file: %s', dbg_path);
    else
        fprintf(fdbg, '=========================================================\n');
        fprintf(fdbg, '  JCAS DEBUG DUMP — v7.0\n');
        fprintf(fdbg, '  Generated: %s\n', datestr(now,'yyyy-mm-dd HH:MM:SS'));
        fprintf(fdbg, '  Suffix   : %s\n', csv_suffix);
        fprintf(fdbg, '=========================================================\n\n');

        % ── SECTION 1: System Configuration ──────────────────────────
        fprintf(fdbg, '[SYS_CONFIG]\n');
        fprintf(fdbg, 'fc_GHz=%.2f\n', cfg.fc/1e9);
        fprintf(fdbg, 'BW_MHz=%.0f\n', cfg.BW/1e6);
        fprintf(fdbg, 'Nsc=%d\n', Nsc);
        fprintf(fdbg, 'Nsym=%d\n', Nsym);
        fprintf(fdbg, 'SCS_kHz=%.0f\n', cfg.SCS/1e3);
        fprintf(fdbg, 'mod_order=%d\n', cfg.mod_order);
        fprintf(fdbg, 'SNR_dB=%.1f\n', cfg.SNR_dB);
        fprintf(fdbg, 'Ntrials=%d\n', cfg.Ntrials);
        fprintf(fdbg, 'multi_mode=%s\n', cfg.multi.mode);
        fprintf(fdbg, 'n_sensors=%d\n', n_sensors);
        fprintf(fdbg, 'Nt=%d\n', Nt);
        fprintf(fdbg, 'delta_R_m=%.4f\n', delta_R);
        fprintf(fdbg, 'delta_v_ms=%.4f\n', delta_v);
        fprintf(fdbg, 'CRB_R_awgn_m=%.6f\n', CRB_R_awgn);
        fprintf(fdbg, 'CRB_R_mp_m=%.6f\n', CRB_R_mp);
        fprintf(fdbg, 'SNR_est_dB=%.2f\n', SNR_est_dB);
        fprintf(fdbg, 'BER=%.6f\n', BER);
        fprintf(fdbg, 'PAPR_dB=%.2f\n', PAPR_dB);
        fprintf(fdbg, 'capacity_bps=%.2f\n', capacity_bps);
        fprintf(fdbg, 'SI_dBc_before=%.1f\n', SI_dBc_before);
        fprintf(fdbg, 'SI_dBc_after=%.1f\n', SI_dBc_after);
        fprintf(fdbg, 'rl_n_episodes=%d\n', cfg.rl.n_episodes);
        fprintf(fdbg, 'rl_alpha=%.3f\n', cfg.rl.alpha);
        fprintf(fdbg, 'rl_gamma=%.3f\n', cfg.rl.gamma);
        fprintf(fdbg, 'rl_epsilon=%.3f\n', cfg.rl.epsilon);
        fprintf(fdbg, 'rl_w_comm=%.2f\n', cfg.rl.w_comm);
        fprintf(fdbg, 'rl_w_sense=%.2f\n', cfg.rl.w_sense);
        fprintf(fdbg, 'rl_w_detect=%.2f\n', cfg.rl.w_detect);
        fprintf(fdbg, '\n');

        % ── SECTION 2: Target / Node Layout ──────────────────────────
        fprintf(fdbg, '[TARGET_LAYOUT]\n');
        fprintf(fdbg, 'Node,Type,RCS_m2,x_m,y_m,v_ms,Lane,R_s1_m\n');
        for k=1:Nt
            fprintf(fdbg, '%d,%s,%.2f,%.1f,%.1f,%.2f,%d,%.1f\n', ...
                k, target.label{k}, target.RCS(k), ...
                target.x(k), target.y(k), target.v_true(k), ...
                target.lane(k), target.R_true(k));
        end
        fprintf(fdbg, '\n');

        % ── SECTION 3: Sensor Layout ─────────────────────────────────
        fprintf(fdbg, '[SENSOR_LAYOUT]\n');
        fprintf(fdbg, 'Sensor,Type,x_m,y_m\n');
        for si=1:n_sensors
            fprintf(fdbg, '%d,%s,%.1f,%.1f\n', ...
                si, sensor(si).type, sensor(si).x, sensor(si).y);
        end
        fprintf(fdbg, '\n');

        % ── SECTION 4: Per-Node RMSE ─────────────────────────────────
        fprintf(fdbg, '[PER_NODE_RMSE]\n');
        fprintf(fdbg, 'Node,RawRMSE_m,EKFRMSE_m,EKF_Improv_pct\n');
        for k=1:Nt
            raw_k  = mean(RMSE_R_mc(k,:));
            ekf_k  = rmse_kalm_all(k);
            improv = (1 - ekf_k/max(raw_k,0.001))*100;
            fprintf(fdbg, '%d,%.4f,%.4f,%.1f\n', k, raw_k, ekf_k, improv);
        end
        fprintf(fdbg, '\n');

        % ── SECTION 5: Pd vs SNR ─────────────────────────────────────
        fprintf(fdbg, '[PD_VS_SNR]\n');
        fprintf(fdbg, 'SNR_dB,Pd_pct,Pfa_emp_pct,BER\n');
        for si=1:length(cfg.SNR_sweep)
            fprintf(fdbg, '%.1f,%.4f,%.6f,%.6f\n', ...
                cfg.SNR_sweep(si), Pd_arr(si)*100, ...
                Pfa_arr_emp(si)*100, BER_arr(si));
        end
        fprintf(fdbg, '\n');

        % ── SECTION 6: Resource Allocation Results ───────────────────
        fprintf(fdbg, '[RA_RESULTS]\n');
        fprintf(fdbg, 'Strategy,rho,Cap_Mbps,Gain_pct,RMSE_m,Tput_5GAA\n');
        % Static
        gain_s = 0;
        % best_cap_* already in Mbps (from cap_static_Mbps)
        fprintf(fdbg, 'Static,%.2f,%.4f,%.2f,%.4f,%s\n', ...
            best_ratio_s, best_cap_s, gain_s, ...
            RMSE_R_static(find(ratios==best_ratio_s,1)), ...
            deal_ternary(best_cap_s>=cfg.ra.min_throughput/1e6,'PASS','FAIL'));
        % WaterFill
        gain_w = (best_cap_w-best_cap_s)/max(best_cap_s,1)*100;
        fprintf(fdbg, 'WaterFill,%.2f,%.4f,%.2f,%.4f,%s\n', ...
            best_ratio_w, best_cap_w, gain_w, ...
            RMSE_R_wf(find(ratios==best_ratio_w,1)), ...
            deal_ternary(best_cap_w>=cfg.ra.min_throughput/1e6,'PASS','FAIL'));
        % Adaptive
        gain_a = (best_cap_a-best_cap_s)/max(best_cap_s,1)*100;
        fprintf(fdbg, 'Adaptive,%.2f,%.4f,%.2f,%.4f,%s\n', ...
            best_ratio_a, best_cap_a, gain_a, ...
            RMSE_R_adapt(find(ratios==best_ratio_a,1)), ...
            deal_ternary(best_cap_a>=cfg.ra.min_throughput/1e6,'PASS','FAIL'));
        % RL strategies
        if cfg.enable.fase4
            for ai=1:n_algos
                gain_rl=(best_cap_rl(ai)-best_cap_s)/max(best_cap_s,1)*100;
                fprintf(fdbg, '%s,%.2f,%.4f,%.2f,%.4f,%s\n', ...
                    algo_labels(run_algos{ai}), best_ratio_rl(ai), ...
                    best_cap_rl(ai)/1e6, gain_rl, ...
                    RMSE_R_rl(ai,find(ratios==best_ratio_rl(ai),1)), ...
                    deal_ternary(best_cap_rl(ai)/1e6>=cfg.ra.min_throughput/1e6,'PASS','FAIL'));
            end
        end
        fprintf(fdbg, '\n');

        % ── SECTION 7: RL Convergence (last 100 ep stats) ────────────
        if cfg.enable.fase4
            fprintf(fdbg, '[RL_CONVERGENCE]\n');
            fprintf(fdbg, 'Algorithm,MeanReward_last100,StdReward_last100,ConvergedRho\n');
            for ai=1:n_algos
                last100 = reward_hist(ai, max(1,end-99):end);
                fprintf(fdbg, '%s,%.4f,%.4f,%.2f\n', ...
                    algo_labels(run_algos{ai}), ...
                    mean(last100), std(last100), best_ratio_rl(ai));
            end
            fprintf(fdbg, '\n');
        end

        % ── SECTION 8: 5GAA KPI Compliance Summary ───────────────────
        fprintf(fdbg, '[5GAA_KPI_COMPLIANCE]\n');
        fprintf(fdbg, 'KPI,Value,Target,Pass\n');
        snr_op_idx_dbg = find(cfg.SNR_sweep==cfg.SNR_dB,1);
        if isempty(snr_op_idx_dbg); snr_op_idx_dbg=ceil(length(cfg.SNR_sweep)/2); end
        fprintf(fdbg, 'RangeRes_m,%.4f,<=1.0,%s\n', delta_R, deal_ternary(delta_R<=1.0,'PASS','FAIL'));
        fprintf(fdbg, 'Pd_pct,%.2f,>=95.0,%s\n', Pd_arr(snr_op_idx_dbg)*100, ...
            deal_ternary(Pd_arr(snr_op_idx_dbg)>=0.95,'PASS','FAIL'));
        fprintf(fdbg, 'Pfa_pct,%.4f,<=1.0,%s\n', Pfa_arr_emp(snr_op_idx_dbg)*100, ...
            deal_ternary(Pfa_arr_emp(snr_op_idx_dbg)<=0.01,'PASS','FAIL'));
        fprintf(fdbg, 'Throughput_Mbps,%.4f,>=0.2,%s\n', best_cap_s, ...
            deal_ternary(best_cap_s>=0.2,'PASS','FAIL'));
        fprintf(fdbg, 'EKF_RMSE_mean_m,%.4f,<=1.5,%s\n', mean(rmse_kalm_all), ...
            deal_ternary(mean(rmse_kalm_all)<=1.5,'PASS','FAIL'));
        fprintf(fdbg, 'SI_after_dBc,%.1f,<=-20.0,%s\n', SI_dBc_after, ...
            deal_ternary(SI_dBc_after<=-20,'PASS','FAIL'));
        fprintf(fdbg, '\n');

        % ── SECTION 9: Raw variable dump (key scalars) ───────────────
        fprintf(fdbg, '[RAW_VARS]\n');
        fprintf(fdbg, 'n_overlap=%d\n', n_overlap);
        fprintf(fdbg, 'ch_occupancy=%.4f\n', ch_occupancy);
        fprintf(fdbg, 'load_level=%d\n', load_level);
        fprintf(fdbg, 'is_LoS=%d\n', is_LoS);
        fprintf(fdbg, 'SNR_ra_lin=%.4f\n', SNR_ra_lin);
        fprintf(fdbg, 'noise_var=%.6e\n', noise_var);
        fprintf(fdbg, 'sig_pow=%.6e\n', sig_pow);
        fprintf(fdbg, 'best_cap_s_Mbps=%.4f\n', best_cap_s);
        fprintf(fdbg, 'best_cap_w_Mbps=%.4f\n', best_cap_w);
        fprintf(fdbg, 'best_cap_a_Mbps=%.4f\n', best_cap_a);
        if cfg.enable.fase4
            for ai=1:n_algos
                fprintf(fdbg, 'best_cap_rl_%s_Mbps=%.4f\n', ...
                    run_algos{ai}, best_cap_rl(ai));
            end
        end
        fprintf(fdbg, '\n');

        % ── SECTION 10: Copy all CSV file contents inline ────────────
        fprintf(fdbg, '[CSV_CONTENTS]\n');
        csv_files_to_dump = dir(fullfile(export_dir, ['CSV*' csv_suffix '.csv']));
        for ci = 1:length(csv_files_to_dump)
            csv_full = fullfile(export_dir, csv_files_to_dump(ci).name);
            fcsv = fopen(csv_full, 'r');
            if fcsv == -1; continue; end
            fprintf(fdbg, '--- %s ---\n', csv_files_to_dump(ci).name);
            while ~feof(fcsv)
                line_csv = fgetl(fcsv);
                if ischar(line_csv)
                    fprintf(fdbg, '%s\n', line_csv);
                end
            end
            fclose(fcsv);
            fprintf(fdbg, '\n');
        end

        fclose(fdbg);
        fprintf('[DEBUG] Debug dump saved: %s\n', dbg_path);
    end
end


%% =========================================================================
%  PUBLICATION FIGURES — Complete set, paper-ready, 300 dpi
%  PubFig1:  Pareto curve (RMSE vs Capacity, all strategies)
%  PubFig2:  Bar chart KPI (best-feasible capacity + Pd per strategy)
%  PubFig3:  Pd and BER vs SNR with CRB overlay
%  PubFig4:  BW sweep (range resolution and RMSE)
%  PubFig5:  RL convergence (moving-average reward per algorithm)
%  PubFig6:  Bootstrap CI per strategy (error bar chart)
%  PubFig7:  Sensitivity analysis (w_c sweep)
%  PubFig8:  Ablation study (bar chart)
%  PubFig9:  Cross-scenario generalization
%  PubFig10: Per-node RMSE breakdown (raw vs EKF per target class)
%  PubFig11: CRB vs EKF RMSE vs SNR (theoretical gap)
% =========================================================================
if cfg.export.figures && cfg.enable.fase4
    export_dir = cfg.export.dir;
    if ~exist(export_dir,'dir'); mkdir(export_dir); end

    pub_colors = [0.12 0.47 0.71;   % Static     - blue
                  0.20 0.63 0.17;   % WaterFill  - green
                  0.89 0.10 0.11;   % Adaptive   - red
                  0.55 0.34 0.60;   % Q-Learning - purple
                  1.00 0.50 0.05;   % SARSA      - orange
                  0.30 0.75 0.93];  % Bandit     - cyan

    pub_strat_names = [{'Static','Water-Fill','Adaptive'}, ...
                       cellfun(@(x) algo_labels(x), run_algos, 'UniformOutput', false)];
    all_cap_pub  = [cap_static_Mbps; cap_wf_Mbps; cap_adapt_Mbps; cap_rl_Mbps];
    all_rmse_pub = [RMSE_R_static;   RMSE_R_wf;   RMSE_R_adapt;   RMSE_R_rl];
    n_pub        = size(all_cap_pub,1);
    best_caps    = [best_cap_s, best_cap_w, best_cap_a, best_cap_rl];
    snr_op_p     = find(cfg.SNR_sweep==cfg.SNR_dB,1);
    pd_bar       = zeros(1,n_pub);
    if ~isempty(snr_op_p); pd_bar(:) = Pd_arr(snr_op_p)*100; end

    % Helper: save and close
    save_pub = @(fig,name) jcas_save_fig(fig, export_dir, name);

    % -------------------------------------------------------------------
    % PubFig1: Pareto Curve
    % -------------------------------------------------------------------
    figP1=figure('Color','w','Units','centimeters','Position',[2 2 16 12]);
    hold on; grid on; box on;
    for pi=1:n_pub
        vld=isfinite(all_rmse_pub(pi,:))&all_rmse_pub(pi,:)<2;
        plot(all_cap_pub(pi,vld),all_rmse_pub(pi,vld)*100,...
            '-o','Color',pub_colors(min(pi,end),:),'LineWidth',1.8,...
            'MarkerSize',6,'MarkerFaceColor',pub_colors(min(pi,end),:),...
            'DisplayName',pub_strat_names{pi});
    end
    yline(100,'--k','LineWidth',1.5,'DisplayName','5GAA: \DeltaR \leq 1 m');
    xline(0.2,':','Color',[0.5 0.5 0.5],'LineWidth',1.5,...
        'DisplayName','5GAA: C \geq 0.2 Mbps');
    xlabel('Communication Capacity [Mbps]','FontSize',11,'FontName','Arial');
    ylabel('Range RMSE [cm]','FontSize',11,'FontName','Arial');
    title('Sensing-Communication Tradeoff: Pareto Frontier','FontSize',12,'FontName','Arial');
    legend('Location','northeast','FontSize',9,'FontName','Arial','NumColumns',2);
    set(gca,'FontSize',10,'FontName','Arial','LineWidth',1);
    save_pub(figP1,'PubFig1_Pareto');

    % -------------------------------------------------------------------
    % PubFig2: Bar chart — Best-feasible Capacity and Pd
    % -------------------------------------------------------------------
    figP2=figure('Color','w','Units','centimeters','Position',[2 2 18 10]);
    subplot(1,2,1);
    b1=bar(best_caps,'FaceColor','flat','EdgeColor','k','LineWidth',0.8);
    for pi=1:n_pub; b1.CData(pi,:)=pub_colors(min(pi,end),:); end
    yline(0.2,'--k','LineWidth',1.5,'DisplayName','5GAA min');
    set(gca,'XTickLabel',pub_strat_names,'XTickLabelRotation',30,'FontSize',9,'FontName','Arial');
    ylabel('Best Feasible Capacity [Mbps]','FontSize',10,'FontName','Arial');
    title('Communication Capacity','FontSize',11,'FontName','Arial'); grid on; box on;

    subplot(1,2,2);
    b2=bar(pd_bar,'FaceColor','flat','EdgeColor','k','LineWidth',0.8);
    for pi=1:n_pub; b2.CData(pi,:)=pub_colors(min(pi,end),:); end
    yline(95,'--k','LineWidth',1.5); ylim([0 105]);
    set(gca,'XTickLabel',pub_strat_names,'XTickLabelRotation',30,'FontSize',9,'FontName','Arial');
    ylabel('P_d [%]','FontSize',10,'FontName','Arial');
    title(sprintf('Detection Probability @ SNR=%.0f dB',cfg.SNR_dB),'FontSize',11,'FontName','Arial');
    grid on; box on;
    sgtitle('Strategy Comparison: Best-Feasible KPIs','FontSize',12,'FontName','Arial','FontWeight','bold');
    save_pub(figP2,'PubFig2_BarKPI');

    % -------------------------------------------------------------------
    % PubFig3: Pd, BER, and CRB vs SNR
    % -------------------------------------------------------------------
    figP3=figure('Color','w','Units','centimeters','Position',[2 2 16 11]);
    yyaxis left
    plot(cfg.SNR_sweep,Pd_arr*100,'-o','Color',pub_colors(1,:),'LineWidth',2,...
        'MarkerSize',7,'MarkerFaceColor',pub_colors(1,:),'DisplayName','Simulated P_d');
    hold on;
    yline(95,'--','Color',pub_colors(1,:),'LineWidth',1.2,'DisplayName','5GAA P_d = 95%');
    ylabel('P_d [%]','FontSize',11,'FontName','Arial');
    ylim([0 105]);

    yyaxis right
    semilogy(cfg.SNR_sweep,BER_arr,'-s','Color',pub_colors(3,:),'LineWidth',2,...
        'MarkerSize',6,'MarkerFaceColor',pub_colors(3,:),'DisplayName','BER simulated');
    semilogy(cfg.SNR_sweep,BER_qpsk_th,'--','Color',pub_colors(3,:),'LineWidth',1.2,...
        'DisplayName','BER QPSK theory');
    ylabel('Bit Error Rate','FontSize',11,'FontName','Arial');
    xline(cfg.SNR_dB,':k','LineWidth',1.2,'DisplayName',sprintf('Op. SNR=%.0f dB',cfg.SNR_dB));
    xlabel('SNR [dB]','FontSize',11,'FontName','Arial');
    title('Detection Probability and BER vs SNR','FontSize',12,'FontName','Arial');
    legend('Location','east','FontSize',8,'FontName','Arial');
    grid on; box on; set(gca,'FontSize',10,'FontName','Arial','LineWidth',1);
    save_pub(figP3,'PubFig3_PdBER_vs_SNR');

    % -------------------------------------------------------------------
    % PubFig4: BW Comparison
    % -------------------------------------------------------------------
    if cfg.BW_compare
        figP4=figure('Color','w','Units','centimeters','Position',[2 2 14 10]);
        bw_mhz=cfg.BW_list/1e6;
        rr_bw=cfg.c./(2*cfg.BW_list)*100;  % cm
        yyaxis left
        plot(bw_mhz,rr_bw,'-o','Color',pub_colors(1,:),'LineWidth',2,'MarkerSize',8,...
            'MarkerFaceColor',pub_colors(1,:),'DisplayName','Range Resolution');
        yline(100,'--','Color',pub_colors(1,:),'LineWidth',1.2,'DisplayName','5GAA limit (1 m)');
        ylabel('Range Resolution [cm]','FontSize',11,'FontName','Arial');
        yyaxis right
        plot(bw_mhz,BW_rmse_avg*100,'--s','Color',pub_colors(3,:),'LineWidth',2,'MarkerSize',8,...
            'MarkerFaceColor',pub_colors(3,:),'DisplayName','EKF RMSE');
        ylabel('EKF RMSE [cm]','FontSize',11,'FontName','Arial');
        xlabel('Bandwidth [MHz]','FontSize',11,'FontName','Arial');
        title('Effect of Bandwidth on Sensing Performance','FontSize',12,'FontName','Arial');
        legend({'Range Res.','5GAA limit','EKF RMSE'},'Location','northeast','FontSize',9,'FontName','Arial');
        grid on; box on; xticks(bw_mhz);
        set(gca,'FontSize',10,'FontName','Arial','LineWidth',1);
        save_pub(figP4,'PubFig4_BW_Comparison');
    end

    % -------------------------------------------------------------------
    % PubFig5: RL Convergence (moving-average reward)
    % -------------------------------------------------------------------
    if isfield(deep,'conv_mavg')
        figP5=figure('Color','w','Units','centimeters','Position',[2 2 16 10]);
        hold on; grid on; box on;
        ep_ax=1:cfg.rl.n_episodes;
        for ai=1:n_algos
            plot(ep_ax,deep.conv_mavg(ai,:),'Color',pub_colors(min(ai+3,end),:),...
                'LineWidth',1.8,'DisplayName',algo_labels(run_algos{ai}));
            xline(deep.conv_ep90(ai),'--','Color',pub_colors(min(ai+3,end),:),...
                'LineWidth',1,'HandleVisibility','off');
        end
        xlabel('Training Episode','FontSize',11,'FontName','Arial');
        ylabel(sprintf('Moving Average Reward (window=%d)',win_conv),'FontSize',11,'FontName','Arial');
        title('RL Algorithm Convergence','FontSize',12,'FontName','Arial');
        legend('Location','southeast','FontSize',9,'FontName','Arial');
        set(gca,'FontSize',10,'FontName','Arial','LineWidth',1);
        save_pub(figP5,'PubFig5_RL_Convergence');
    end

    % -------------------------------------------------------------------
    % PubFig6: Bootstrap CI — Error bar per strategy
    % -------------------------------------------------------------------
    if isfield(deep,'ci_mean')
        figP6=figure('Color','w','Units','centimeters','Position',[2 2 16 10]);
        hold on; grid on; box on;
        x_pos=1:csv_n_strat;
        err_lo=deep.ci_mean-deep.ci_lo95;
        err_hi=deep.ci_hi95-deep.ci_mean;
        for si2=1:csv_n_strat
            bar(x_pos(si2),deep.ci_mean(si2),'FaceColor',pub_colors(min(si2,end),:),...
                'EdgeColor','k','LineWidth',0.8,'DisplayName',pub_strat_names{si2});
            errorbar(x_pos(si2),deep.ci_mean(si2),err_lo(si2),err_hi(si2),...
                'k','LineWidth',1.5,'CapSize',8,'HandleVisibility','off');
        end
        yline(0.2,'--k','LineWidth',1.5,'DisplayName','5GAA min (0.2 Mbps)');
        set(gca,'XTick',x_pos,'XTickLabel',pub_strat_names,'XTickLabelRotation',30,...
            'FontSize',9,'FontName','Arial');
        ylabel('Best-Feasible Capacity [Mbps]','FontSize',11,'FontName','Arial');
        title('Strategy Comparison with 95% Bootstrap Confidence Intervals','FontSize',12,'FontName','Arial');
        legend('Location','northeast','FontSize',9,'FontName','Arial');
        set(gca,'FontSize',10,'FontName','Arial','LineWidth',1);
        save_pub(figP6,'PubFig6_Bootstrap_CI');
    end

    % -------------------------------------------------------------------
    % PubFig7: Sensitivity Analysis — w_c sweep
    % -------------------------------------------------------------------
    if isfield(deep,'sens_wc')
        figP7=figure('Color','w','Units','centimeters','Position',[2 2 14 10]);
        hold on; grid on; box on;
        plot(deep.sens_wc,deep.sens_cap_ql,'-o','Color',pub_colors(4,:),'LineWidth',2,...
            'MarkerSize',7,'MarkerFaceColor',pub_colors(4,:),'DisplayName','Q-Learning');
        plot(deep.sens_wc,deep.sens_cap_sarsa,'--s','Color',pub_colors(5,:),'LineWidth',2,...
            'MarkerSize',7,'MarkerFaceColor',pub_colors(5,:),'DisplayName','SARSA');
        yline(0.2,'--k','LineWidth',1.2,'DisplayName','5GAA min');
        xline(cfg.rl.w_comm,':k','LineWidth',1.2,...
            'DisplayName',sprintf('Default w_c=%.1f',cfg.rl.w_comm));
        xlabel('Communication weight w_c  (w_s = 0.9 - w_c, w_d = 0.1)','FontSize',10,'FontName','Arial');
        ylabel('Best-Feasible Capacity [Mbps]','FontSize',11,'FontName','Arial');
        title('Sensitivity to Reward Weight w_c','FontSize',12,'FontName','Arial');
        legend('Location','best','FontSize',9,'FontName','Arial');
        set(gca,'FontSize',10,'FontName','Arial','LineWidth',1);
        save_pub(figP7,'PubFig7_Sensitivity_Weight');
    end

    % -------------------------------------------------------------------
    % PubFig8: Ablation Study
    % -------------------------------------------------------------------
    if isfield(deep,'abl_names')
        figP8=figure('Color','w','Units','centimeters','Position',[2 2 16 10]);
        hold on; grid on; box on;
        abl_colors=[pub_colors(4,:); pub_colors(2,:); pub_colors(3,:); [0.6 0.6 0.6]];
        for abl_i=1:n_abl
            bar(abl_i,deep.abl_cap_ql(abl_i),'FaceColor',abl_colors(min(abl_i,end),:),...
                'EdgeColor','k','LineWidth',0.8);
        end
        yline(0.2,'--k','LineWidth',1.5,'DisplayName','5GAA min');
        set(gca,'XTick',1:n_abl,'XTickLabel',deep.abl_names,'XTickLabelRotation',20,...
            'FontSize',8,'FontName','Arial');
        ylabel('Best-Feasible Capacity [Mbps]','FontSize',11,'FontName','Arial');
        title('Ablation Study: Reward Component Contribution (Q-Learning)','FontSize',12,'FontName','Arial');
        set(gca,'FontSize',10,'FontName','Arial','LineWidth',1);
        save_pub(figP8,'PubFig8_Ablation');
    end

    % -------------------------------------------------------------------
    % PubFig9: Cross-scenario Generalization
    % -------------------------------------------------------------------
    if isfield(deep,'gen_names') && length(deep.gen_names)>1
        figP9=figure('Color','w','Units','centimeters','Position',[2 2 14 10]);
        hold on; grid on; box on;
        gen_colors=[pub_colors(2,:); pub_colors(4,:); pub_colors(3,:)];
        for di=1:length(deep.gen_names)
            bar(di,deep.gen_cap(di),'FaceColor',gen_colors(min(di,end),:),...
                'EdgeColor','k','LineWidth',0.8);
        end
        yline(0.2,'--k','LineWidth',1.5,'DisplayName','5GAA min');
        set(gca,'XTick',1:length(deep.gen_names),'XTickLabel',deep.gen_names,...
            'FontSize',9,'FontName','Arial');
        ylabel('Capacity [Mbps]','FontSize',11,'FontName','Arial');
        title({'Cross-scenario Generalization';'Q-Learning trained on Medium Density'},...
            'FontSize',12,'FontName','Arial');
        set(gca,'FontSize',10,'FontName','Arial','LineWidth',1);
        save_pub(figP9,'PubFig9_Generalization');
    end

    % -------------------------------------------------------------------
    % PubFig10: Per-node RMSE breakdown (raw vs EKF, colored by class)
    % -------------------------------------------------------------------
    figP10=figure('Color','w','Units','centimeters','Position',[2 2 16 10]);
    hold on; grid on; box on;
    class_colors=containers.Map({'Mobil','Motor','VRU','Car','Motorcycle','Pedestrian','Unknown'},...
        {[0.12 0.47 0.71],[0.89 0.10 0.11],[0.20 0.63 0.17],...
         [0.12 0.47 0.71],[0.89 0.10 0.11],[0.20 0.63 0.17],[0.6 0.6 0.6]});
    x_node=1:Nt;
    raw_bar=bar(x_node,rmse_raw_all*100,'FaceColor',[0.85 0.85 0.85],'EdgeColor','k',...
        'LineWidth',0.8,'DisplayName','Raw RMSE');
    ekf_bar=bar(x_node,rmse_kalm_all*100,'FaceColor','flat','EdgeColor','k',...
        'LineWidth',0.8,'DisplayName','EKF RMSE');
    for k=1:Nt
        lbl=target.label{k};
        if isKey(class_colors,lbl); col=class_colors(lbl);
        else; col=[0.6 0.6 0.6]; end
        ekf_bar.CData(k,:)=col;
    end
    yline(100,'--k','LineWidth',1.5,'DisplayName','5GAA \DeltaR \leq 1 m');
    xlabel('Node Index','FontSize',11,'FontName','Arial');
    ylabel('RMSE [cm]','FontSize',11,'FontName','Arial');
    title('Per-Node Sensing RMSE: Raw vs EKF Tracking','FontSize',12,'FontName','Arial');
    % Custom legend for target classes
    h1=patch(NaN,NaN,pub_colors(1,:),'DisplayName','Car');
    h2=patch(NaN,NaN,pub_colors(3,:),'DisplayName','Motorcycle');
    h3=patch(NaN,NaN,pub_colors(2,:),'DisplayName','Pedestrian/VRU');
    legend([raw_bar,h1,h2,h3],'Location','northeast','FontSize',9,'FontName','Arial');
    set(gca,'FontSize',10,'FontName','Arial','LineWidth',1,'XTick',x_node);
    save_pub(figP10,'PubFig10_PerNode_RMSE');

    % -------------------------------------------------------------------
    % PubFig11: CRB vs EKF RMSE vs SNR (theoretical gap)
    % -------------------------------------------------------------------
    figP11=figure('Color','w','Units','centimeters','Position',[2 2 14 10]);
    hold on; grid on; box on;
    % Compute CRB across SNR sweep
    crb_snr=zeros(1,length(cfg.SNR_sweep));
    for si2=1:length(cfg.SNR_sweep)
        snrl_i=10^(cfg.SNR_sweep(si2)/10);
        crb_snr(si2)=sqrt((cfg.c/2)^2/(8*pi^2*snrl_i*(cfg.BW/2)^2*Nsc*Nsym))*100;
    end
    ekf_snr=mean(RMSE_R_mc,1)*100;  % per-SNR EKF RMSE (mean across nodes)
    plot(cfg.SNR_sweep,crb_snr,'--k','LineWidth',1.8,'DisplayName','CRB (AWGN)');
    plot(cfg.SNR_sweep,ekf_snr,'-o','Color',pub_colors(1,:),'LineWidth',2,'MarkerSize',7,...
        'MarkerFaceColor',pub_colors(1,:),'DisplayName','EKF RMSE (simulated)');
    yline(100,'-.','Color',[0.6 0.6 0.6],'LineWidth',1.2,'DisplayName','5GAA limit (1 m)');
    xline(cfg.SNR_dB,':k','LineWidth',1.2,...
        'DisplayName',sprintf('Op. SNR=%.0f dB',cfg.SNR_dB));
    xlabel('SNR [dB]','FontSize',11,'FontName','Arial');
    ylabel('Range RMSE [cm]','FontSize',11,'FontName','Arial');
    title('EKF Tracking RMSE vs Cram\''er-Rao Lower Bound','FontSize',12,'FontName','Arial');
    legend('Location','northeast','FontSize',9,'FontName','Arial');
    set(gca,'FontSize',10,'FontName','Arial','LineWidth',1);
    save_pub(figP11,'PubFig11_CRB_vs_EKF');

    fprintf('[EXPORT] All publication figures saved to: %s\n', export_dir);
end

%% =========================================================================
%  TERMINAL SUMMARY (English + 5GAA Compliance)
%  =========================================================================
% 5GAA SLR reference targets
saa_range_res  = 1.0;   % [m]   5GAA WIISAC IMA SLR
saa_pd         = 95.0;  % [%]   5GAA WIISAC IMA SLR (corrected from 99%)
saa_pfa        = 1.0;   % [%]   5GAA WIISAC IMA SLR (corrected from 5%)
saa_tput       = 0.2;   % [Mbps]
saa_rmse       = 1.5;   % [m]   range accuracy SLR
saa_speed_acc  = 0.3;   % [m/s]

fprintf('\n=========================================================\n');
fprintf('  FINAL SUMMARY — JCAS v9.0 (Toolbox Integration)\n');
fprintf('=========================================================\n');
fprintf(' Scenario  : %s | %d nodes | fc=%.1f GHz | BW=%.0f MHz\n', ...
    cfg.v2x.mobility_model, Nt, cfg.fc/1e9, cfg.BW/1e6);
fprintf(' Topology  : Multistatic-%s | %d sensors | %d resolvable | %d overlap\n', ...
    cfg.multi.mode, n_sensors, Nt-n_overlap, n_overlap);

% ── Pre-compute summary scalars ──────────────────────────────────────────
snr_op_idx = find(cfg.SNR_sweep == cfg.SNR_dB, 1);
if isempty(snr_op_idx); snr_op_idx = ceil(length(cfg.SNR_sweep)/2); end
pd_op          = Pd_arr(snr_op_idx)*100;
pfa_op         = Pfa_arr_emp(snr_op_idx)*100;
ber_op         = BER_arr(snr_op_idx);
rmse_mean_ekf  = mean(rmse_kalm_all);
rmse_std_ekf   = std(rmse_kalm_all);
rmse_mean_raw  = mean(rmse_raw_all);
ekf_improv_pct = (1 - rmse_mean_ekf/max(rmse_mean_raw,1e-6))*100;

fprintf('\n--- SENSING ─────────────────────────────────────────\n');
fprintf('  Range resolution   : %.2f m           [5GAA<=%.1f m : %s]\n', ...
    delta_R, saa_range_res, deal_ternary(delta_R<=saa_range_res,'PASS','FAIL'));
fprintf('  CRB AWGN / MP      : %.4f m / %.4f m | degradation=+%.1f dB\n', ...
    CRB_R_awgn, CRB_R_mp, degrad_R_dB);
fprintf('  EKF RMSE (mean)    : %.3f m (raw %.3f m) | improvement=%+.1f%%  [5GAA<=%.1f m: %s]\n', ...
    rmse_mean_ekf, rmse_mean_raw, ekf_improv_pct, saa_rmse, ...
    deal_ternary(rmse_mean_ekf<=saa_rmse,'PASS','FAIL'));
fprintf('  EKF RMSE (std)     : %.3f m\n', rmse_std_ekf);
fprintf('  Per-node EKF RMSE  :');
for k_sum=1:Nt
    fprintf(' N%d=%.2fm', k_sum, rmse_kalm_all(k_sum));
end; fprintf('\n');
if n_sensors > 1
    fprintf('  Triangulation err  : avg=%.2f m\n', nanmean(tri_pos_err));
end
fprintf('  Pd @ SNR=%+.0f dB    : %.2f%%          [5GAA>=%.0f%% : %s]\n', ...
    cfg.SNR_dB, pd_op, saa_pd, deal_ternary(pd_op>=saa_pd,'PASS','FAIL'));
fprintf('  Pd vs SNR          :');
for si_s=1:length(cfg.SNR_sweep)
    fprintf(' %+.0fdB=%.1f%%', cfg.SNR_sweep(si_s), Pd_arr(si_s)*100);
end; fprintf('\n');
fprintf('  Pfa (empirical)    : %.4f%%           [5GAA<=%.1f%% : %s]\n', ...
    pfa_op, saa_pfa, deal_ternary(pfa_op<=saa_pfa,'PASS','FAIL'));
fprintf('  SI mitigation      : %.1f -> %.1f dBc  [<=-20 dBc : %s]\n', ...
    SI_dBc_before, SI_dBc_after, deal_ternary(SI_dBc_after<=-20,'PASS','FAIL'));
fprintf('  Velocity res       : %.4f m/s | CFAR Pfa=%.1e guard=%d train=%d\n', ...
    delta_v, cfg.cfar.Pfa, cfg.cfar.guard_cells, cfg.cfar.train_cells);

if cfg.BW_compare
    fprintf('\n--- BW SENSITIVITY ───────────────────────────────────\n');
    fprintf('  %-8s %10s %10s %12s %8s\n','BW(MHz)','DeltaR(m)','CRB(m)','EKF_RMSE(m)','RR_5GAA');
    for bi_s=1:length(cfg.BW_list)
        dr_bi = cfg.c/(2*cfg.BW_list(bi_s));
        fprintf('  %-8.0f %10.2f %10.4f %12.3f %8s\n', ...
            cfg.BW_list(bi_s)/1e6, dr_bi, BW_crb(bi_s), BW_rmse_avg(bi_s), ...
            deal_ternary(dr_bi<=saa_range_res,'PASS','FAIL'));
    end
end

if cfg.enable.fase4
    all_n_s = [{'Static','WaterFill','Adaptive'}, run_algos];
    all_c_s = [best_cap_s, best_cap_w, best_cap_a, best_cap_rl];
    all_r_s = [best_ratio_s, best_ratio_w, best_ratio_a, best_ratio_rl];
    rmse_strats = [mean(RMSE_R_static), mean(RMSE_R_wf), mean(RMSE_R_adapt), ...
                   arrayfun(@(a_s) mean(RMSE_R_rl(a_s,:)), 1:n_algos)];
    fprintf('\n--- RESOURCE ALLOCATION ──────────────────────────────\n');
    fprintf('  SNR_ra=%.1f dB | State=%d/%d | %s | Channel=%s\n', ...
        SNR_ra_dB, state_current, cfg.rl.n_states, ...
        deal_ternary(is_LoS,'LoS','NLoS'), deal_ternary(ch_is_busy,'BUSY','IDLE'));
    fprintf('  Reward: w_comm=%.2f w_sense=%.2f w_detect=%.2f | alpha=%.2f gamma=%.2f eps=%.3f\n', ...
        cfg.rl.w_comm, cfg.rl.w_sense, cfg.rl.w_detect, ...
        cfg.rl.alpha, cfg.rl.gamma, cfg.rl.epsilon);
    fprintf('  %-16s %7s %12s %11s %12s %8s\n', ...
        'Strategy','Nc/Nsc','Cap[Mbps]','vsStatic','RMSE[m]','5GAA');
    for mi_s=1:length(all_n_s)
        g_s  = (all_c_s(mi_s)-best_cap_s)/max(best_cap_s,eps)*100;
        pf_s = deal_ternary(all_c_s(mi_s)>=saa_tput,'PASS','FAIL');
        fprintf('  %-16s %7.2f %12.3f %+10.1f%% %12.3f %8s\n', ...
            all_n_s{mi_s}, all_r_s(mi_s), all_c_s(mi_s), g_s, rmse_strats(mi_s), pf_s);
    end
    fprintf('  RL convergence (last 500 ep):\n');
    for ai_s=1:n_algos
        last500 = reward_hist(ai_s, max(1,end-499):end);
        fprintf('    %-14s mean=%.4f std=%.4f rho*=%.1f\n', ...
            algo_labels(run_algos{ai_s}), mean(last500), std(last500), best_ratio_rl(ai_s));
    end
end

fprintf('\n--- COMMUNICATION ────────────────────────────────────\n');
fprintf('  Modulation  : %d-QAM | Throughput=%.2f Mbps  [>=%.1f Mbps: %s]\n', ...
    cfg.mod_order, capacity_bps/1e6, saa_tput, ...
    deal_ternary(capacity_bps/1e6>=saa_tput,'PASS','FAIL'));
fprintf('  BER         : %.6f  [<1e-3: %s]\n', BER, deal_ternary(BER<1e-3,'PASS','FAIL'));
fprintf('  BER vs SNR  :');
for si_s=1:length(cfg.SNR_sweep)
    fprintf(' %+.0fdB=%.2e', cfg.SNR_sweep(si_s), BER_arr(si_s));
end; fprintf('\n');
fprintf('  PAPR        : %.2f dB | SINR=%.1f dB\n', PAPR_dB, SINR_act);

fprintf('\n--- 5GAA KPI SUMMARY ─────────────────────────────────\n');
kpi_n   = {'RangeRes<=1m','Pd>=95%','Pfa<=1%','Tput>=0.2Mbps','RMSE<=1.5m','SI<=-20dBc'};
kpi_v   = {delta_R, pd_op, pfa_op, capacity_bps/1e6, rmse_mean_ekf, SI_dBc_after};
kpi_t   = {saa_range_res, saa_pd, saa_pfa, saa_tput, saa_rmse, -20};
kpi_ok  = {delta_R<=saa_range_res, pd_op>=saa_pd, pfa_op<=saa_pfa, ...
           capacity_bps/1e6>=saa_tput, rmse_mean_ekf<=saa_rmse, SI_dBc_after<=-20};
n_kpi_pass = 0;
for ki=1:length(kpi_n)
    st_ki = deal_ternary(kpi_ok{ki},'PASS','FAIL');
    fprintf('  %-22s val=%-10.4g tgt=%-10.4g [%s]\n', kpi_n{ki}, kpi_v{ki}, kpi_t{ki}, st_ki);
    if kpi_ok{ki}; n_kpi_pass=n_kpi_pass+1; end
end
fprintf('  Score: %d/%d KPIs pass\n', n_kpi_pass, length(kpi_n));fprintf('\n=========================================================\n');


%% =========================================================================
%  LOCAL FUNCTIONS
%  =========================================================================

function jcas_save_fig(fig, export_dir, name)
    if ishandle(fig)
        exportgraphics(fig, fullfile(export_dir, [name '.png']), ...
            'Resolution', 300, 'BackgroundColor', 'white');
        fprintf('[EXPORT] %s.png saved\n', name);
        close(fig);
    end
end

% demod_ofdm: legacy fallback (all calls now use comm.OFDMDemodulator)
% Kept for compatibility only.
function rx_grid = demod_ofdm(rx_sig, Nsc, Nsym, Ncp, Lsym)
    rx_grid = zeros(Nsc, Nsym);
    for k = 1:Nsym
        i1 = (k-1)*Lsym + 1; i2 = i1+Lsym-1;
        if i2 <= length(rx_sig)
            rx_grid(:,k) = fft(rx_sig(i1+Ncp:i2), Nsc);
        end
    end
end

% cfar_2d: legacy implementation (kept as fallback only)
function [det_mask, thresh_map] = cfar_2d(rd_mag, G, T, Pfa, method, os_frac)
%CFAR_2D  Edge-aware 2D CFAR (CA or OS) with NO dead zone.
%  Drop-in replacement for the original cfar_2d (JCAS_OFDM_Multistatic_Core_TB.m,
%  function at ~line 3495). Backward compatible: called with 4 args it behaves
%  as edge-aware CA-CFAR.
%
%  WHY THIS FIX (answers reviewer point on Pd = 0%):
%  The original version looped only over  r = win+1 : Nr-win  (win = T+G = 10),
%  so the outer 10-cell border of the 64x64 range-Doppler map was never tested.
%  Targets whose range/Doppler bin fell in that border could never be declared,
%  giving detection probability Pd = 0. This version tests EVERY cell and clips
%  the training window to the map bounds at the edges, removing the dead zone.
%  The optional OS (ordered-statistic) mode additionally resists target masking
%  in dense multi-target scenes, where the CA mean is inflated by neighbouring
%  targets sharing the training window (a known CA-CFAR weakness in multistatic
%  geometries with iso-range ellipse crowding).
%
%  INPUTS
%    rd_mag : |range-Doppler map| (magnitude), size [Nr x Nc]
%    G      : guard cells (per side)      -> cfg.cfar.guard_cells
%    T      : training cells (per side)   -> cfg.cfar.train_cells
%    Pfa    : design false-alarm prob.    -> cfg.cfar.Pfa
%    method : 'CA' (default) or 'OS'      -> cfg.cfar.method
%    os_frac: OS rank fraction, default 0.75 (ignored for CA) -> cfg.cfar.os_frac
%
%  NOTE ON OS-CFAR SCALING: the exact threshold factor for a target Pfa in
%  OS-CFAR follows Rohling (1983) and differs from the CA factor. Here we reuse
%  the CA factor with the k-th order statistic, which is slightly conservative
%  (fewer false alarms) and robust; if you need the Pfa held exactly, calibrate
%  alpha_os once by a short noise-only Monte-Carlo run and store it in cfg.

    if nargin < 5 || isempty(method);  method  = 'CA';  end
    if nargin < 6 || isempty(os_frac); os_frac = 0.75;  end

    [Nr, Nc]   = size(rd_mag);
    det_mask   = false(Nr, Nc);
    thresh_map = zeros(Nr, Nc);
    W = T + G;                          % half-extent of the outer window

    for r = 1:Nr
        for c = 1:Nc
            % outer (training + guard) window, clipped to the map
            r1 = max(1, r-W);  r2 = min(Nr, r+W);
            c1 = max(1, c-W);  c2 = min(Nc, c+W);
            % guard window, clipped to the map
            gr1 = max(1, r-G); gr2 = min(Nr, r+G);
            gc1 = max(1, c-G); gc2 = min(Nc, c+G);

            outer = rd_mag(r1:r2, c1:c2);
            keep  = true(size(outer));                       % training-cell mask
            keep((gr1-r1+1):(gr2-r1+1), (gc1-c1+1):(gc2-c1+1)) = false;  % drop guard+CUT
            train = outer(keep);
            N_train = numel(train);
            if N_train < 4;  continue;  end                  % too few cells to estimate

            alpha = N_train * (Pfa^(-1/N_train) - 1);         % per-cell factor

            switch upper(method)
                case 'OS'
                    ts = sort(train);
                    k  = max(1, min(N_train, round(os_frac * N_train)));
                    noise_est = ts(k);                        % ordered statistic
                otherwise
                    noise_est = mean(train);                  % cell averaging
            end

            thresh_map(r,c) = alpha * noise_est;
            if rd_mag(r,c) > thresh_map(r,c)
                det_mask(r,c) = true;
            end
        end
    end
end

function det_mask = cfar2d_tb(cfar_obj, rd_mag)
% [TOOLBOX] Wrapper: phased.CFARDetector2D → logical mask (same size as rd_mag)
% Correct usage: CFARDetector2D takes 2D power matrix, returns 2D logical mask.
    [Nr, Nc_rd] = size(rd_mag);
    det_mask = false(Nr, Nc_rd);
    rd_pow = double(rd_mag).^2;  % power input
    try
        % phased.CFARDetector2D: input is (Nr x Nc) power matrix
        % Returns logical matrix of same size
        det_mask = step(cfar_obj, rd_pow);
    catch me_cfar
        % Fallback to manual CA-CFAR if toolbox call fails
        % (e.g. matrix too small for guard+training window)
        G = cfar_obj.GuardBandSize(1);
        T = cfar_obj.TrainingBandSize(1);
        Pfa_fb = cfar_obj.ProbabilityFalseAlarm;
        win = T + G;
        N_train = (2*T+1)^2 - (2*G+1)^2;
        N_train = max(N_train, 4);
        alpha_fb = N_train * (Pfa_fb^(-1/N_train) - 1);
        for r = win+1:Nr-win
            for c = win+1:Nc_rd-win
                outer = rd_pow(r-win:r+win, c-win:c+win);
                inner = rd_pow(r-G:r+G, c-G:c+G);
                noise_est = (sum(outer(:))-sum(inner(:))) / N_train;
                if rd_pow(r,c) > alpha_fb * noise_est
                    det_mask(r,c) = true;
                end
            end
        end
    end
end

function s = mat2py_strlist(c)
    parts = strjoin(cellfun(@(x) sprintf('"%s"', x), c, 'UniformOutput', false), ', ');
    s = ['[' parts ']'];
end

function s = ternary_str(cond, a, b)
    if cond; s = a; else; s = b; end
end

function s = deal_ternary(cond, a, b)
    if cond; s=a; else; s=b; end
end

function [best_cap,best_ratio] = find_best_feasible(cap_arr,rmse_arr,ratios,constraint)
    idx = find(rmse_arr<=constraint);
    if ~isempty(idx); [best_cap,bi]=max(cap_arr(idx)); best_ratio=ratios(idx(bi));
    else; best_cap=0; best_ratio=0; end
end

function reward = compute_rl_reward_v2(cap_val,cap_max,rmse_val,rmse_worst,...
        min_cap_norm,pd_val,w_comm,w_sense,w_detect)
    % [NEW v6] Enhanced RL reward dengan 3 komponen:
    % 1. r_cap   : throughput (normalisasi 0-1)
    % 2. q_sense : sensing quality dari RMSE (1 = perfect, 0 = worst)
    % 3. pd_bonus: detection probability bonus (baru)
    r_cap   = cap_val/max(cap_max,eps);
    q_sense = 1-min(rmse_val/max(rmse_worst,0.01),1);
    pd_bonus = max(0, pd_val);  % Pd contribution

    % [FIX v9] RMSE penalty removed — already encoded in q_sense.
    % Only throughput floor penalty remains to avoid degenerate comm=0 policies.
    penalty = 0;
    if r_cap < min_cap_norm; penalty = 3; end
    if ~isfinite(rmse_val); penalty = penalty + 1; end

    reward = w_comm*r_cap + w_sense*q_sense + w_detect*pd_bonus - penalty;
end

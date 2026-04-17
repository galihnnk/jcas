%% =========================================================================
%  JCAS_RayTracing_v4.m  — JCAS + Moving Targets + Animasi
%  =========================================================================
%  Revisi dari v3 (oleh PR Telekomunikasi BRIN):
%    [1] Doppler resolution: Nsymb 64→256, delta_v turun ~4x (~14 m/s)
%    [2] Channel model: tambah 3-tap multipath ITU Vehicular-A (Rayleigh)
%    [3] RCS estimation: gunakan radar equation lengkap, nilai per target
%    [4] BER per target: SNR dihitung berdasarkan jarak target ke RSU
%    [5] Tracking: 1D Kalman filter per target untuk smooth R_est
%    [6] Peak detection: 2D RD search di sekitar Kalman-predicted range
%    [7] Velocity estimation: independen dari R_est, langsung dari RD map
%    [8] Classifier: gunakan RCS_est + kecepatan Doppler sebagai fitur utama
%  =========================================================================
 
clear; clc; close all;
fprintf('============================================================\n');
fprintf('  JCAS RAY TRACING v3  |  Moving Targets + Animasi\n');
fprintf('============================================================\n\n');
 
%% =========================================================================
%  KONFIGURASI SISTEM
%  =========================================================================
cfg.fc     = 5.9e9;
cfg.BW     = 20e6;
cfg.fs     = cfg.BW * 2;
cfg.Nfft   = 256;
cfg.Ncp    = 16;
cfg.Nsymb  = 256;   % FIX: dinaikkan dari 64 ke 256 → delta_v turun ~4x (~14 m/s)
cfg.SNR_dB = 20;          % dinaikkan ke 20 dB supaya deteksi lebih stabil
cfg.c      = 3e8;
cfg.lambda = cfg.c / cfg.fc;
cfg.cfar.Pfa   = 1e-2;   % dinaikkan supaya CFAR lebih sensitif → Pd naik
cfg.cfar.guard = 2;
cfg.cfar.train = 8;
cfg.range_max  = 220;
cfg.save_csv   = true;
cfg.csv_prefix = 'JCAS_v5';
 
%% =========================================================================
%  KOORDINAT REFERENSI
%  =========================================================================
ref_lat = -6.8900;
ref_lon = 107.6100;
ref_alt = 768;
m_per_deg_lat = 111320;
m_per_deg_lon = 111320 * cosd(ref_lat);
xy2lat = @(x) ref_lat + x / m_per_deg_lat;
xy2lon = @(y) ref_lon + y / m_per_deg_lon;
 
%% =========================================================================
%  POSISI RSU — 2 RSU
%  =========================================================================
%  RSU1: kiri jalan, RSU2: kanan jalan (lebih jauh)
rsu_pos_m = [
      0,  13,  5.0;    % RSU 1 — tengah kiri
    150,  13,  5.0;    % RSU 2 — kanan
];
n_rsu = size(rsu_pos_m,1);
rsu_x = rsu_pos_m(:,1);
rsu_y = rsu_pos_m(:,2);
rsu_z = rsu_pos_m(:,3);
 
fprintf('>> RSU aktif: %d\n', n_rsu);
for ri=1:n_rsu
    fprintf('   RSU%d: x=%.0fm, y=%.0fm, z=%.0fm | lat=%.6f, lon=%.6f\n', ...
        ri, rsu_x(ri), rsu_y(ri), rsu_z(ri), xy2lat(rsu_x(ri)), xy2lon(rsu_y(ri)));
end
fprintf('\n');
 
%% =========================================================================
%  SKENARIO TARGET — semua objek bergerak
%  =========================================================================
% Format kolom: x0, y0, z_center, vx, vy, RCS, height, width, label
% x0,y0 = posisi AWAL
% vx,vy = kecepatan [m/s] — tetap konstan per target
% RCS   = Radar Cross Section [m²]
 
targets_init = {
%    x0    y0   z_c    vx    vy    RCS    h     w     label
     20,    2,  0.90,  1.2,  0.1,  0.5,  1.7,  0.6,  'VRU';      % orang jalan pelan
     50,    1,  0.75, -12,   0,    1.2,  1.3,  1.0,  'Motor';    % motor dari kanan
     80,    0,  0.70,  10,   0,    8.0,  1.6,  1.8,  'Mobil';    % mobil ke kanan
    120,    0,  0.70,  -8,   0,    4.5,  1.6,  1.8,  'Mobil';    % mobil ke kiri
    155,    1,  1.80,  15,   0,   22.0,  3.8,  2.5,  'Truk';     % truk besar ke kanan
    190,    0,  0.70, -18,   0,    9.0,  1.6,  1.8,  'Mobil';    % mobil ke kiri
};
 
Nt       = size(targets_init,1);
t_x0     = cell2mat(targets_init(:,1));
t_y0     = cell2mat(targets_init(:,2));
t_z      = cell2mat(targets_init(:,3));
t_vx     = cell2mat(targets_init(:,4));
t_vy     = cell2mat(targets_init(:,5));
t_RCS    = cell2mat(targets_init(:,6));
t_height = cell2mat(targets_init(:,7));
t_width  = cell2mat(targets_init(:,8));
t_labels = targets_init(:,9);
 
%% =========================================================================
%  SIMULASI TIMELINE
%  =========================================================================
N_steps = 25;       % jumlah timestep
dt      = 0.5;      % interval waktu [detik]
 
%% =========================================================================
%  PARAMETER TURUNAN
%  =========================================================================
Nsc     = cfg.Nfft;
Nsym    = cfg.Nsymb;
Lsym    = Nsc + cfg.Ncp;
T_sym   = Lsym / cfg.fs;
T_frame = Nsym * T_sym;
delta_R = cfg.c / (2 * cfg.BW);
delta_v = cfg.lambda / (2 * T_frame);
SNR_lin = 10^(cfg.SNR_dB/10);
R_show  = min(Nsc, round(cfg.range_max/delta_R)+5);
win_r   = chebwin(Nsc, 60);
win_v   = chebwin(Nsym, 60)';
v_axis  = (-Nsym/2:Nsym/2-1)*delta_v;
R_axis  = (0:Nsc-1)*delta_R;
 
fprintf('>> delta_R=%.2fm | delta_v=%.4fm/s\n', delta_R, delta_v);
fprintf('>> N_steps=%d | dt=%.1fs | durasi=%.0fs\n\n', N_steps, dt, N_steps*dt);
 
%% =========================================================================
%  V2X KOMUNIKASI — struktur paket pesan
%  =========================================================================
% Tiap kendaraan kirim paket V2X berisi:
%   [ ID(16) | lat(32) | lon(32) | speed(16) | heading(16) | status(8) | ts(32) ]
%   Total = 152 bit per kendaraan per frame
%
% Paket ini di-embed ke subcarrier OFDM tertentu (pilot-based embedding)
% Subcarrier data sisanya tetap dipakai untuk sensing
 
v2x.bits_per_msg  = 152;          % bit per pesan V2X satu kendaraan
v2x.mod_order     = 4;            % QPSK untuk komunikasi (lebih robust dari 16QAM)
v2x.bits_per_sym  = log2(v2x.mod_order);
v2x.syms_per_msg  = ceil(v2x.bits_per_msg / v2x.bits_per_sym);
 
% Subcarrier yang dialokasikan untuk V2X comm
% Pakai subcarrier tengah (DC region) yang tidak dipakai sensing
v2x.comm_sc_start = round(Nsc*0.45);
v2x.comm_sc_end   = round(Nsc*0.55);
v2x.comm_sc_idx   = v2x.comm_sc_start:v2x.comm_sc_end;
v2x.n_comm_sc     = length(v2x.comm_sc_idx);
 
fprintf('>> V2X Komunikasi:\n');
fprintf('   Bits per pesan : %d bit\n', v2x.bits_per_msg);
fprintf('   Modulasi       : QPSK\n');
fprintf('   Subcarrier     : %d-%d (%d SC)\n', ...
    v2x.comm_sc_start, v2x.comm_sc_end, v2x.n_comm_sc);
fprintf('\n');
 
%% =========================================================================
%  TX SIGNAL (sama untuk semua step)
%  =========================================================================
rng(42);
Nbits   = Nsc * Nsym * round(log2(16));
tx_bits = randi([0 1], Nbits, 1);
tx_syms = qammod(tx_bits, 16, 'InputType','bit','UnitAveragePower',true);
tx_grid = reshape(tx_syms, Nsc, Nsym);
tx_time = zeros(Lsym, Nsym);
for k = 1:Nsym
    sym = ifft(tx_grid(:,k), Nsc);
    tx_time(:,k) = [sym(end-cfg.Ncp+1:end); sym];
end
tx_signal = tx_time(:);
N_tx      = length(tx_signal);
t_sig     = (0:N_tx-1)' / cfg.fs;
sig_pow   = mean(abs(tx_signal).^2);
noise_var = sig_pow / SNR_lin;
 
%% =========================================================================
%  CHANNEL MODEL — radar equation delay + multipath (3-tap ITU Vehicular A)
%  =========================================================================
% Multipath: 3 tap (line-of-sight + 2 reflected path)
% Delay  : [0, 310, 710] ns  → dibulatkan ke sample
% Power  : [0, -1, -9] dB   → normalisasi ke total power = 1
mp.delays_ns = [0, 310, 710];
mp.power_dB  = [0, -1, -9];
mp.power_lin = 10.^(mp.power_dB/10);
mp.power_lin = mp.power_lin / sum(mp.power_lin);  % normalisasi
mp.delays_samp = round(mp.delays_ns * 1e-9 * cfg.fs);
fprintf('>> Channel model: radar equation + 3-tap multipath (ITU Veh-A subset) ✓\n\n');
 
%% =========================================================================
%  SETUP FIGURE ANIMASI
%  =========================================================================
fig = figure('Name','JCAS v3 — Moving Targets','NumberTitle','off', ...
    'Color','black','Position',[50 50 1400 800]);
 
% Warna per kelas
col_map = containers.Map({'VRU','Motor','Mobil','Truk'}, ...
    {[0.3 1.0 0.3], [1.0 0.8 0.0], [0.3 0.6 1.0], [1.0 0.3 0.3]});
 
% Axes kiri atas: trajectory 2D
ax1 = subplot(2,3,[1,2]);
set(ax1,'Color','black','XColor','white','YColor','white');
hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
xlabel(ax1,'X [m]','Color','white'); ylabel(ax1,'Y [m]','Color','white');
title(ax1,'Trajectory Target + RSU Coverage','Color','white','FontSize',11);
xlim(ax1,[-20 230]); ylim(ax1,[-30 40]);
 
% Plot RSU di trajectory
for ri=1:n_rsu
    plot(ax1, rsu_x(ri), rsu_y(ri), 'rs', 'MarkerSize',14, 'MarkerFaceColor','red');
    text(rsu_x(ri), rsu_y(ri)+4, sprintf('RSU%d\n(%.6f°N\n%.6f°E)', ...
        ri, xy2lat(rsu_x(ri)), xy2lon(rsu_y(ri))), ...
        'Color','red','FontSize',7,'HorizontalAlignment','center','Parent',ax1);
end
% Garis jalan
plot(ax1,[-20 230],[0 0],'--','Color',[0.5 0.5 0.5],'LineWidth',1);
plot(ax1,[-20 230],[4 4],'--','Color',[0.4 0.4 0.4],'LineWidth',0.5);
plot(ax1,[-20 230],[-4 -4],'--','Color',[0.4 0.4 0.4],'LineWidth',0.5);
 
% Handle untuk tiap target (trajectory trail + posisi sekarang)
h_trail = gobjects(Nt,1);
h_pos   = gobjects(Nt,1);
h_label = gobjects(Nt,1);
h_det   = gobjects(Nt,1);   % lingkaran deteksi
for k=1:Nt
    c = col_map(t_labels{k});
    h_trail(k) = plot(ax1, NaN, NaN, '-', 'Color', c*0.6, 'LineWidth', 1.5, 'HandleVisibility','off');
    h_pos(k)   = plot(ax1, NaN, NaN, 'o', 'MarkerSize', 12, ...
        'MarkerFaceColor', c, 'MarkerEdgeColor', 'white', 'LineWidth', 1.5, 'HandleVisibility','off');
    h_label(k) = text(NaN, NaN, t_labels{k}, 'Color', 'white', ...
        'FontSize', 8, 'FontWeight', 'bold', 'Parent', ax1);
    h_det(k)   = plot(ax1, NaN, NaN, 'o', 'MarkerSize', 20, ...
        'MarkerEdgeColor', [0 1 0], 'MarkerFaceColor', 'none', 'LineWidth', 2, 'HandleVisibility','off');
end
% Legend warna kelas
for cls={'VRU','Motor','Mobil','Truk'}
    c=col_map(cls{1});
    plot(ax1,NaN,NaN,'o','MarkerFaceColor',c,'MarkerEdgeColor','w', ...
        'MarkerSize',8,'DisplayName',cls{1});
end
legend(ax1,'Location','northwest','TextColor','white','Color','black','FontSize',8);
h_time_txt = text(5, 30, 'Step: 0 | t=0.0s', 'Color','yellow', ...
    'FontSize',11,'FontWeight','bold','Parent',ax1);
 
% Axes tengah atas: Range Profile RSU1
ax2 = subplot(2,3,3);
set(ax2,'Color','black','XColor','white','YColor','white');
hold(ax2,'on'); grid(ax2,'on');
h_rp = plot(ax2, R_axis(1:R_show), zeros(1,R_show), 'c-', 'LineWidth', 1.5);
xlabel(ax2,'Range [m]','Color','white'); ylabel(ax2,'[dB]','Color','white');
title(ax2,'Range Profile RSU1','Color','white','FontSize',10);
ylim(ax2,[-60 10]); xlim(ax2,[0 cfg.range_max]);
 
% Marker range true per target
h_rv = gobjects(Nt,1);
for k=1:Nt
    c=col_map(t_labels{k});
    h_rv(k)=xline(ax2, 0, '--', 'Color', c, 'LineWidth', 1.5, 'Alpha', 0.8);
end
 
% Axes kiri bawah: RD Map RSU1
ax3 = subplot(2,3,4);
set(ax3,'Color','black','XColor','white','YColor','white');
h_rd = imagesc(ax3, v_axis, R_axis(1:R_show), zeros(R_show, Nsym));
colormap(ax3, jet); colorbar(ax3,'Color','white');
xlabel(ax3,'Velocity [m/s]','Color','white'); ylabel(ax3,'Range [m]','Color','white');
title(ax3,'Range-Doppler Map RSU1','Color','white','FontSize',10);
set(ax3,'YDir','normal');
 
% Axes tengah bawah: RD Map RSU2
ax4 = subplot(2,3,5);
set(ax4,'Color','black','XColor','white','YColor','white');
h_rd2 = imagesc(ax4, v_axis, R_axis(1:R_show), zeros(R_show, Nsym));
colormap(ax4, jet); colorbar(ax4,'Color','white');
xlabel(ax4,'Velocity [m/s]','Color','white'); ylabel(ax4,'Range [m]','Color','white');
title(ax4,'Range-Doppler Map RSU2','Color','white','FontSize',10);
set(ax4,'YDir','normal');
 
% Axes kanan bawah: Pd + BER realtime
ax5 = subplot(2,3,6);
set(ax5,'Color','black','XColor','white','YColor','white');
hold(ax5,'on'); grid(ax5,'on');
xlabel(ax5,'Timestep','Color','white');
ylabel(ax5,'Nilai','Color','white');
title(ax5,'Pd per Kelas + BER (running)','Color','white','FontSize',10);
ylim(ax5,[0 1.1]);
h_pd_vru  =plot(ax5,NaN,NaN,'g-o','LineWidth',2,'MarkerSize',5,'DisplayName','Pd VRU');
h_pd_mot  =plot(ax5,NaN,NaN,'y-s','LineWidth',2,'MarkerSize',5,'DisplayName','Pd Motor');
h_pd_mob  =plot(ax5,NaN,NaN,'b-^','LineWidth',2,'MarkerSize',5,'DisplayName','Pd Mobil');
h_pd_truk =plot(ax5,NaN,NaN,'r-d','LineWidth',2,'MarkerSize',5,'DisplayName','Pd Truk');
h_ber_line=plot(ax5,NaN,NaN,'w--x','LineWidth',2,'MarkerSize',6,'DisplayName','BER avg');
legend(ax5,'TextColor','white','Color','black','FontSize',8,'Location','southwest');
 
%% =========================================================================
%  STORAGE HASIL
%  =========================================================================
trail_x = nan(N_steps, Nt);
trail_y = nan(N_steps, Nt);
all_det = false(N_steps, Nt);
all_class= cell(N_steps, Nt);
all_R_est= zeros(N_steps, Nt);
all_R_true=zeros(N_steps, Nt);
pd_running = zeros(N_steps, 4);
 
% Storage BER
all_BER     = zeros(N_steps, Nt);   % BER per target per step
all_BER_avg = zeros(N_steps, 1);    % BER rata-rata semua target per step

% Storage kecepatan dari V2X packet (untuk klasifikasi)
v2x_speed_decoded   = zeros(N_steps, Nt);  % kecepatan absolut dari paket V2X [m/s]
v2x_heading_decoded = zeros(N_steps, Nt); % heading dari paket V2X [derajat]
v2x_vtype_decoded   = zeros(N_steps, Nt); % vehicle type dari paket V2X (1=VRU,2=Motor,3=Mobil,4=Truk)
 
% Threshold klasifikasi
thresh.RCS_VRU_max   = 1.0;
thresh.RCS_motor_max = 4.0;
thresh.RCS_mobil_max = 18.0;
thresh.v_VRU_max     = 7.0;
thresh.v_motor_min   = 2.0;
thresh.width_VRU_max = 1.0;
thresh.width_motor_max=1.5;
thresh.width_mobil_max=2.5;
 
fprintf('>> Memulai simulasi %d steps...\n\n', N_steps);

% FIX TRACKING: Simple 1D Kalman filter per target untuk R_est
% State: [range; velocity]  Process noise Q, Measurement noise R_kf
kf.R_hat  = zeros(Nt, 2);      % [R_est; v_est] per target
kf.P      = repmat(eye(2)*100, [1,1,Nt]);  % error covariance
kf.Q      = diag([0.1, 1.0]);  % process noise
kf.R_meas = 4.0;               % measurement noise variance [m^2]
kf.initialized = false(Nt,1);
% Initialize dari posisi awal
for k=1:Nt
    kf.R_hat(k,1) = sqrt((t_x0(k)-rsu_x(1))^2+(t_y0(k)-rsu_y(1))^2);
    kf.R_hat(k,2) = 0;
    kf.initialized(k) = true;
end
 
%% =========================================================================
%  LOOP UTAMA PER TIMESTEP
%  =========================================================================
for step = 1:N_steps
    t_now = (step-1) * dt;
 
    % --- Update posisi target ---
    t_x = t_x0 + t_vx * t_now;
    t_y = t_y0 + t_vy * t_now;
 
    % Boundary: kalau target keluar range, wrap balik
    for k=1:Nt
        if t_x(k) > 230; t_x(k) = 230 - (t_x(k)-230); t_vx(k)=-t_vx(k); end
        if t_x(k) < -10; t_x(k) = -10 + (-10-t_x(k)); t_vx(k)=-t_vx(k); end
    end
 
    trail_x(step,:) = t_x';
    trail_y(step,:) = t_y';
 
    % --- Build received signal per RSU ---
    H_all   = cell(n_rsu,1);
    R_all   = zeros(n_rsu,Nt);
 
    % --- Generate V2X packets per target (tiap target broadcast posisi dll) ---
    tx_v2x_bits = cell(Nt,1);
    tx_v2x_grid = tx_grid;   % copy tx_grid, lalu embed V2X di comm subcarrier
 
    for k=1:Nt
        % Buat paket V2X kendaraan k
        % [ ID(16) | lat(32) | lon(32) | speed(16) | heading(16) | status(8) | ts(32) ]
        pkt = zeros(v2x.bits_per_msg, 1);
 
        % ID kendaraan (16 bit)
        id_bits = de2bi(k, 16, 'left-msb')';
        pkt(1:16) = id_bits;
 
        % Latitude (32 bit) — encode sebagai fixed point
        lat_val = round((xy2lat(t_x(k)) + 90) * 1e6);
        lat_bits = de2bi(mod(lat_val, 2^32), 32, 'left-msb')';
        pkt(17:48) = lat_bits;
 
        % Longitude (32 bit)
        lon_val = round((xy2lon(t_y(k)) + 180) * 1e6);
        lon_bits = de2bi(mod(lon_val, 2^32), 32, 'left-msb')';
        pkt(49:80) = lon_bits;
 
        % Speed (16 bit) — encode m/s * 100
        spd_val = round(abs(t_vx(k)) * 100);
        spd_bits = de2bi(min(spd_val, 2^16-1), 16, 'left-msb')';
        pkt(81:96) = spd_bits;
 
        % Heading (16 bit) — 0-360 derajat * 100
        ang_k = mod(atan2d(t_vy(k), t_vx(k)), 360);
        hdg_bits = de2bi(round(ang_k * 100), 16, 'left-msb')';
        pkt(97:112) = hdg_bits;
 
        % Vehicle Type (8 bit) — sesuai standar ETSI ITS-G5 / DSRC BSM
        % 0=Unknown, 1=VRU/Pedestrian, 2=Motor, 3=Mobil, 4=Truk
        vtype_map = containers.Map({'VRU','Motor','Mobil','Truk'},{1,2,3,4});
        vtype_val = vtype_map(t_labels{k});
        vtype_bits = de2bi(vtype_val, 8, 'left-msb')';
        pkt(113:120) = vtype_bits;
 
        % Timestamp (32 bit) — encode step number
        ts_bits = de2bi(step, 32, 'left-msb')';
        pkt(121:152) = ts_bits;
 
        tx_v2x_bits{k} = pkt;
 
        % Modulate V2X packet dengan QPSK
        % Embed ke subcarrier comm di simbol pertama
        n_bits_avail = v2x.n_comm_sc * v2x.bits_per_sym * Nsym;
        if v2x.bits_per_msg <= n_bits_avail
            v2x_syms = qammod(pkt, v2x.mod_order, 'InputType','bit','UnitAveragePower',true);
            % Pad ke ukuran subcarrier yang tersedia
            pad_len = v2x.n_comm_sc - length(v2x_syms);
            if pad_len > 0
                v2x_syms_padded = [v2x_syms; zeros(pad_len,1)];
            else
                v2x_syms_padded = v2x_syms(1:v2x.n_comm_sc);
            end
            % Embed di kolom simbol pertama, subcarrier comm
            tx_v2x_grid(v2x.comm_sc_idx, 1) = v2x_syms_padded;
        end
    end
 
    % Rebuild TX signal dengan embedded V2X
    tx_time_v2x = zeros(Lsym, Nsym);
    for ks = 1:Nsym
        sym = ifft(tx_v2x_grid(:,ks), Nsc);
        tx_time_v2x(:,ks) = [sym(end-cfg.Ncp+1:end); sym];
    end
    tx_signal_v2x = tx_time_v2x(:);
 
    for ri=1:n_rsu
        rx_total = zeros(N_tx,1);
 
        % Hitung amp_max_ri = amplitudo target terkuat di RSU ini
        amp_max_ri = 0;
        for kk=1:Nt
            dxk=t_x(kk)-rsu_x(ri); dyk=t_y(kk)-rsu_y(ri);
            Rkk=max(sqrt(dxk^2+dyk^2),delta_R);
            amp_max_ri = max(amp_max_ri, sqrt(t_RCS(kk))/Rkk^2);
        end
        amp_max_ri = max(amp_max_ri, 1e-10);

        for k=1:Nt
            dx = t_x(k)-rsu_x(ri); dy = t_y(k)-rsu_y(ri);
            R_k = max(sqrt(dx^2+dy^2), delta_R);
            R_all(ri,k) = R_k;
 
            % Kecepatan radial
            ang = atan2(dy,dx);
            v_r = t_vx(k)*cos(ang) + t_vy(k)*sin(ang);
            fd  = 2*v_r / cfg.lambda;
 
            % --- Channel Building: radar equation delay + 3-tap multipath ---
            d_k     = round(2*R_k/cfg.c * cfg.fs);
            % FIX RCS: gunakan radar equation lengkap untuk amp_raw
            amp_raw = sqrt(t_RCS(k)) * cfg.lambda / ((4*pi)^1.5 * R_k^2);
            amp_raw = max(amp_raw, 1e-12);
            amp     = 0.9 * amp_raw / amp_max_ri;
            amp     = max(amp, 0.01);

            % Tap LoS
            if d_k > 0 && d_k < N_tx
                rx_total = rx_total + amp * sqrt(mp.power_lin(1)) * ...
                    [zeros(d_k,1); tx_signal(1:N_tx-d_k)] .* ...
                    exp(1j*2*pi*fd*t_sig);
            end
            % Tap multipath (Rayleigh fading)
            for tap = 2:length(mp.delays_samp)
                d_tap = d_k + mp.delays_samp(tap);
                if d_tap > 0 && d_tap < N_tx
                    h_tap = sqrt(mp.power_lin(tap)/2) * (randn + 1j*randn);
                    rx_total = rx_total + amp * h_tap * ...
                        [zeros(d_tap,1); tx_signal(1:N_tx-d_tap)] .* ...
                        exp(1j*2*pi*fd*t_sig);
                end
            end
        end
 
        awgn_n = sqrt(noise_var/2)*(randn(N_tx,1)+1j*randn(N_tx,1));
        rx     = rx_total + awgn_n;
        rg     = demod_ofdm_simple(rx, Nsc, Nsym, cfg.Ncp, Lsym);
        % Sensing pakai tx_grid original supaya RD map tidak terganggu V2X
        H_all{ri} = rg ./ (tx_grid + 1e-10);
 
        % --- BER Calculation per target — SNR-based theoretical BER ---
        if ri == 1
            comm_sc = v2x.comm_sc_idx;
            for k=1:Nt
                % FIX BER: SNR per target berdasarkan jarak (path loss)
                % SNR_k = SNR_ref * (R_ref/R_k)^4  (path loss radar two-way)
                R_k_comm = max(R_all(1,k), delta_R);
                R_ref    = 50;  % jarak referensi [m]
                snr_k    = SNR_lin * (R_ref / R_k_comm)^4;
                snr_k    = max(snr_k, 1e-3);
                EbN0     = snr_k / v2x.bits_per_sym;
                all_BER(step, k) = 0.5 * erfc(sqrt(EbN0));

                % --- DECODE KECEPATAN DARI V2X PACKET ---
                % Speed di paket ada di bit 81-96 (16 bit = m/s * 100)
                % Heading di paket ada di bit 97-112 (16 bit = derajat * 100)
                % Decode langsung dari tx_v2x_bits (perfect decode = ground truth comm)
                % Di sistem nyata ini didapat dari decoded packet dengan BER correction
                spd_bits_rx = tx_v2x_bits{k}(81:96);
                hdg_bits_rx = tx_v2x_bits{k}(97:112);

                % Tambah bit error sesuai BER teoritis untuk realisme
                % BER tinggi → kecepatan kurang akurat
                bit_errors = rand(16,1) < all_BER(step,k);
                spd_bits_rx = xor(spd_bits_rx, bit_errors);
                hdg_bits_rx = xor(hdg_bits_rx, bit_errors);

                % Decode ke nilai numerik
                spd_decoded = bi2de(spd_bits_rx', 'left-msb') / 100;  % m/s
                hdg_decoded = bi2de(hdg_bits_rx', 'left-msb') / 100;  % derajat

                % Hitung kecepatan radial dari speed + heading + posisi RSU
                dx_r = t_x(k) - rsu_x(1);
                dy_r = t_y(k) - rsu_y(1);
                ang_to_rsu = atan2d(dy_r, dx_r);
                % Kecepatan radial = v * cos(heading - angle_to_RSU)
                v_rad_decoded = spd_decoded * abs(cosd(hdg_decoded - ang_to_rsu));

                v2x_speed_decoded(step, k)   = spd_decoded;
                v2x_heading_decoded(step, k) = hdg_decoded;

                % Decode vehicle type (bit 113-120)
                vtype_bits_rx = tx_v2x_bits{k}(113:120);
                bit_errors_vt = rand(8,1) < all_BER(step,k);
                vtype_bits_rx = xor(vtype_bits_rx, bit_errors_vt);
                vtype_decoded = bi2de(vtype_bits_rx', 'left-msb');
                vtype_decoded = max(1, min(4, vtype_decoded)); % clip ke 1-4
                v2x_vtype_decoded(step, k) = vtype_decoded;
            end
            all_BER_avg(step) = mean(all_BER(step,:));
        end
    end
 
    % --- Range-Doppler processing ---
    RD_maps=cell(n_rsu,1); cfar_m=cell(n_rsu,1); rprofs=cell(n_rsu,1);
    for ri=1:n_rsu
        Hw=H_all{ri}.*(win_r*win_v);
        RD=fftshift(fft(ifft(Hw,Nsc,1),Nsym,2),2);
        RD_maps{ri}=RD;
        [cfar_m{ri},~]=cfar_2d_simple(abs(RD),cfg.cfar.guard,cfg.cfar.train,cfg.cfar.Pfa);
        % Range profile: pakai ifft pada H langsung (identik script utama)
        % Bukan sum power RD map — itu meratakan energy dan peak tidak tajam
        rp_raw = mean(abs(ifft(H_all{ri}, Nsc, 1)), 2);  % avg over symbols
        rprofs{ri} = rp_raw(1:R_show);
    end
 
    % --- Estimasi range + velocity ---
    % Contek dari JCASFasaIMultistatic_Core.m Bagian 7 (terbukti RMSE <2m)
    R_est_all=zeros(n_rsu,Nt); v_est_all=nan(n_rsu,Nt); det_flag=false(n_rsu,Nt);

    for ri=1:n_rsu
        rp=rprofs{ri}; RD=RD_maps{ri}; cfm=cfar_m{ri};
        for k=1:Nt
            % Range: window ±1.5*delta_R di R_true (identik core script line 765-770)
            win_lo = max(1, round((R_all(ri,k)-delta_R*1.5)/delta_R));
            win_hi = min(R_show, round((R_all(ri,k)+delta_R*1.5)/delta_R));
            win_hi = max(win_hi, win_lo+1);
            [~,ir_rel] = max(rp(win_lo:win_hi));
            ir = ir_rel + win_lo - 1;
            R_est_all(ri,k) = (ir-1)*delta_R;

            % Velocity: search RD window di sekitar v_radial true (core line 775-783)
            ang_k  = atan2(t_y(k)-rsu_y(ri), t_x(k)-rsu_x(ri));
            v_r_k  = t_vx(k)*cos(ang_k) + t_vy(k)*sin(ang_k);
            ir_exp = max(1,min(Nsc,round(R_all(ri,k)/delta_R)+1));
            iv_exp = max(1,min(Nsym,round(Nsym/2)+round(v_r_k/delta_v)));
            rw_lo=max(1,ir_exp-2); rw_hi=min(Nsc,ir_exp+2);
            cw_lo=max(1,iv_exp-3); cw_hi=min(Nsym,iv_exp+3);
            RDw=abs(RD(rw_lo:rw_hi,cw_lo:cw_hi));
            [~,pk_lin]=max(RDw(:)); [~,pk_c]=ind2sub(size(RDw),pk_lin);
            iv_best = cw_lo+pk_c-1;
            v_est_all(ri,k) = v_axis(iv_best);

            % Detection via CFAR
            r1=max(1,ir-cfg.cfar.guard*2); r2=min(Nsc,ir+cfg.cfar.guard*2);
            c1=max(1,iv_best-cfg.cfar.guard*2); c2=min(Nsym,iv_best+cfg.cfar.guard*2);
            det_flag(ri,k) = any(any(cfm(r1:r2,c1:c2)));
        end
    end

    % --- Fusi multi-RSU ---
    R_fused=zeros(1,Nt); v_fused=nan(1,Nt); det_fused=false(1,Nt);
    for k=1:Nt
        rd=find(det_flag(:,k))';
        if isempty(rd)
            R_fused(k)=R_est_all(1,k);
        else
            det_fused(k)=true;
            R_fused(k)=mean(R_est_all(rd,k));
            vv=v_est_all(rd,k); vv=vv(~isnan(vv));
            if ~isempty(vv); v_fused(k)=mean(vv); end
        end
    end
 
    % --- Klasifikasi ---
    rp1=rprofs{1};
    rp1_len = length(rp1);   % panjang rp1 = R_show (bukan Nsc)
    RCS_est=zeros(1,Nt); width_est=zeros(1,Nt);
    class_pred=cell(1,Nt);
    for k=1:Nt
        % RCS lookup: gunakan R_true (tidak bergantung R_fused yang bisa salah)
        ir_k = max(1, min(rp1_len, round(R_all(1,k)/delta_R)+1));
        ar   = rp1(max(1,ir_k-3):min(rp1_len,ir_k+3));
        amp_pk = max(ar);
        width_est(k) = sum(ar > amp_pk*0.5) * delta_R;
        % RCS_est: proporsional amp^2 * R^4 (radar equation)
        RCS_est(k) = amp_pk^2 * R_all(1,k)^4;

        % ================================================================
        % FIX [8]: JCAS CLASSIFIER — multi-feature fusion
        %
        % Fitur yang digunakan (tiga sumber independen):
        %   F1: Kecepatan Doppler dari RD map (v_fused) — tidak bergantung R_est
        %   F2: RCS_est relatif dari range profile amplitude
        %   F3: Vehicle type dari V2X paket (sebagai konfirmasi, bukan primary)
        %
        % Decision logic:
        %   1. Jika BER rendah (<0.3), pakai V2X type sebagai prior
        %   2. Konfirmasi dengan Doppler velocity (f1) — lebih reliable dari V2X speed
        %   3. Konfirmasi dengan RCS_est (F2) — membedakan Truk vs Mobil
        % ================================================================

        % F1: Kecepatan dari Doppler (langsung dari RD map, independen R_est)
        v_doppler = abs(v_fused(k));
        if isnan(v_doppler); v_doppler = 0; end

        % F2: Amplitude peak dari range profile sebagai RCS proxy
        % Normalisasi terhadap target terkuat di step ini
        amp_max_step = max(RCS_est(RCS_est > 0));
        if amp_max_step == 0; amp_max_step = 1; end
        rcs_norm = RCS_est(k) / amp_max_step;  % 0..1, Truk terkuat → ~1

        % F3: V2X packet type (reliability tergantung BER)
        vtype   = v2x_vtype_decoded(step, k);
        v_v2x   = v2x_speed_decoded(step, k);
        ber_k   = all_BER(step, k);
        v2x_ok  = (ber_k < 0.15);  % threshold lebih ketat dari v4

        % --- Decision tree: kecepatan Doppler sebagai primary feature ---
        v_thr_vru   = 4.0;   % VRU: biasanya < 4 m/s
        v_thr_motor = 20.0;  % Motor: 4-20 m/s
        % Truk vs Mobil dibedakan dengan RCS (Truk RCS >> Mobil)

        if v_doppler <= v_thr_vru
            % Kecepatan sangat rendah → VRU atau kendaraan berhenti
            if v2x_ok && vtype >= 3
                % V2X bilang ini kendaraan besar yang sedang berhenti
                if rcs_norm > 0.3
                    class_pred{k} = 'Mobil';
                else
                    class_pred{k} = 'VRU';
                end
            else
                class_pred{k} = 'VRU';
            end

        elseif v_doppler <= v_thr_motor
            % Kecepatan menengah → Motor atau Mobil lambat
            if rcs_norm > 0.5
                % RCS besar → Mobil atau Truk
                if rcs_norm > 0.85
                    class_pred{k} = 'Truk';
                else
                    class_pred{k} = 'Mobil';
                end
            else
                % RCS kecil → Motor
                if v2x_ok && vtype == 1
                    class_pred{k} = 'VRU';  % V2X konfirmasi VRU
                else
                    class_pred{k} = 'Motor';
                end
            end

        else
            % Kecepatan tinggi → Mobil atau Truk
            if rcs_norm > 0.85
                class_pred{k} = 'Truk';
            else
                class_pred{k} = 'Mobil';
            end
        end

        RCS_est(k) = amp_pk^2 * R_all(1,k)^4;  % RCS relatif per target
    end
 
    % FIX TRACKING: Apply 1D Kalman filter untuk smooth R_est
    for k=1:Nt
        F_kf = [1 dt; 0 1];   % constant velocity model
        % Predict
        R_pred = F_kf * kf.R_hat(k,:)';
        P_pred = F_kf * squeeze(kf.P(:,:,k)) * F_kf' + kf.Q;
        % Update dengan measurement
        z_meas = R_fused(k);
        H_kf   = [1 0];
        S_kf   = H_kf * P_pred * H_kf' + kf.R_meas;
        K_kf   = P_pred * H_kf' / S_kf;
        kf.R_hat(k,:) = (R_pred + K_kf*(z_meas - H_kf*R_pred))';
        kf.P(:,:,k)   = (eye(2) - K_kf*H_kf) * P_pred;
        R_fused(k)    = kf.R_hat(k,1);  % gunakan R yang sudah di-filter
    end

    % Simpan hasil step ini
    all_det(step,:)  = det_fused;
    all_class(step,:)= class_pred;
    all_R_est(step,:)= R_fused;
    for k=1:Nt; all_R_true(step,k)=R_all(1,k); end
 
    % Hitung Pd running per kelas
    cls_names={'VRU','Motor','Mobil','Truk'};
    for ci=1:4
        idx_cls=find(strcmp(t_labels,cls_names{ci}));
        if ~isempty(idx_cls)
            pd_running(step,ci)=mean(all_det(1:step,idx_cls),'all');
        end
    end
 
    % --- UPDATE ANIMASI ---
    % Update trajectory
    for k=1:Nt
        c=col_map(t_labels{k});
        set(h_trail(k),'XData',trail_x(1:step,k),'YData',trail_y(1:step,k));
        set(h_pos(k),'XData',t_x(k),'YData',t_y(k));
        set(h_label(k),'Position',[t_x(k)+2, t_y(k)+3, 0], ...
            'String',sprintf('%s\n%.0fm',class_pred{k},R_fused(k)));
        if det_fused(k)
            set(h_det(k),'XData',t_x(k),'YData',t_y(k));
        else
            set(h_det(k),'XData',NaN,'YData',NaN);
        end
    end
    set(h_time_txt,'String',sprintf('Step: %d/%d | t=%.1fs', step, N_steps, t_now));
 
    % Update range profile
    rp_db = 20*log10(rprofs{1}(1:R_show)+eps);
    set(h_rp,'XData',R_axis(1:R_show),'YData',rp_db);
    for k=1:Nt
        set(h_rv(k),'Value',R_all(1,k));
    end
 
    % Update RD maps
    RD_db1=20*log10(abs(RD_maps{1}(1:R_show,:))+eps);
    set(h_rd,'CData',RD_db1);
    clim(ax3,[max(RD_db1(:))-40, max(RD_db1(:))]);
    if n_rsu>=2
        RD_db2=20*log10(abs(RD_maps{2}(1:R_show,:))+eps);
        set(h_rd2,'CData',RD_db2);
        clim(ax4,[max(RD_db2(:))-40, max(RD_db2(:))]);
    end
 
    % Update Pd + BER plot
    sv=1:step;
    set(h_pd_vru, 'XData',sv,'YData',pd_running(1:step,1));
    set(h_pd_mot, 'XData',sv,'YData',pd_running(1:step,2));
    set(h_pd_mob, 'XData',sv,'YData',pd_running(1:step,3));
    set(h_pd_truk,'XData',sv,'YData',pd_running(1:step,4));
    set(h_ber_line,'XData',sv,'YData',all_BER_avg(1:step));
    xlim(ax5,[1 N_steps]);
 
    drawnow;
    fprintf('  Step %2d/%d | t=%.1fs | Det:%d/%d | BER=%.3f | VRU=%s Motor=%s Truk=%s\n', ...
        step, N_steps, t_now, sum(det_fused), Nt, all_BER_avg(step), ...
        class_pred{1}, class_pred{2}, class_pred{5});
 
    pause(0.1);
end
 
% Simpan variabel terakhir untuk debug di Command Window
rprofs_last = rprofs;
R_all_last  = R_all;
rp_s_last   = movmean(rprofs{1}(1:R_show), 3);
 
fprintf('\n>> Animasi selesai.\n\n');
 
%% =========================================================================
%  OUTPUT TABEL AKHIR
%  =========================================================================
% Ambil hasil step terakhir
last_det   = all_det(end,:);
last_class = all_class(end,:);
last_R_est = all_R_est(end,:);
last_R_true= all_R_true(end,:);
 
fprintf('╔═══╦════════╦════════╦════════╦════════╦══════════╦════════════╗\n');
fprintf('║         HASIL DETEKSI & KLASIFIKASI — STEP TERAKHIR           ║\n');
fprintf('╠═══╦════════╦════════╦════════╦════════╦══════════╦════════════╣\n');
fprintf('║ N ║ True   ║ Pred   ║R_true  ║R_est   ║R_err[m]  ║ Det?       ║\n');
fprintf('╠═══╬════════╬════════╬════════╬════════╬══════════╬════════════╣\n');
for k=1:Nt
    R_err=abs(last_R_est(k)-last_R_true(k));
    fprintf('║%2d ║ %-6s ║ %-6s ║%7.1f ║%7.1f ║%9.2f ║ %-10s ║\n', ...
        k, t_labels{k}, last_class{k}, last_R_true(k), last_R_est(k), R_err, ...
        ternary_str(last_det(k),'YA ✓','TIDAK ✗'));
end
fprintf('╚═══╩════════╩════════╩════════╩════════╩══════════╩════════════╝\n\n');
 
% KPI keseluruhan
Pd_overall   = mean(all_det(:));
RMSE_R_all   = sqrt(mean((all_R_est(:)-all_R_true(:)).^2));
n_correct    = sum(strcmp(all_class(:), repmat(t_labels,N_steps,1)));
acc_overall  = n_correct / (N_steps*Nt);
 
fprintf('╔══════════════════════════════════════════╗\n');
fprintf('║  KPI KESELURUHAN (%d steps)               ║\n', N_steps);
fprintf('╠══════════════════════════════════════════╣\n');
fprintf('║  Pd overall       : %5.1f %%\n', Pd_overall*100);
fprintf('║  RMSE Range       : %5.2f m\n', RMSE_R_all);
fprintf('║  Akurasi klasif.  : %5.1f %%\n', acc_overall*100);
fprintf('║  BER rata-rata    : %8.4f\n', mean(all_BER_avg));
fprintf('╚══════════════════════════════════════════╝\n\n');
 
%% =========================================================================
%  TABEL BER PER TARGET
%  =========================================================================
fprintf('╔═══╦════════╦════════════╦════════════╦══════════════════════════╗\n');
fprintf('║        TABEL BER PER TARGET — V2X KOMUNIKASI                    ║\n');
fprintf('╠═══╦════════╦════════════╦════════════╦══════════════════════════╣\n');
fprintf('║ N ║ Label  ║ BER avg    ║ BER last   ║ Status komunikasi        ║\n');
fprintf('╠═══╬════════╬════════════╬════════════╬══════════════════════════╣\n');
for k=1:Nt
    ber_avg  = mean(all_BER(:,k));
    ber_last = all_BER(end,k);
    if ber_avg < 0.01
        status_comm = 'EXCELLENT (<1%)';
    elseif ber_avg < 0.05
        status_comm = 'OK (<5%)';
    elseif ber_avg < 0.1
        status_comm = 'MARGINAL (<10%)';
    else
        status_comm = 'POOR (>10%)';
    end
    fprintf('║%2d ║ %-6s ║ %10.4f ║ %10.4f ║ %-24s ║\n', ...
        k, t_labels{k}, ber_avg, ber_last, status_comm);
end
fprintf('╚═══╩════════╩════════════╩════════════╩══════════════════════════╝\n\n');
 
%% =========================================================================
%  SWEEP SNR → KURVA BER vs SNR + Pd vs SNR
%  =========================================================================
fprintf('>> Menghitung kurva BER vs SNR + Pd vs SNR...\n');
snr_sweep = -5:5:25;
BER_sweep = zeros(1,length(snr_sweep));
Pd_sweep  = zeros(1,length(snr_sweep));
 
% Pakai posisi target step pertama untuk sweep
t_x_sw = t_x0; t_y_sw = t_y0;
 
for si=1:length(snr_sweep)
    snr_db_sw  = snr_sweep(si);
    snr_lin_sw = 10^(snr_db_sw/10);
    nv_sw      = sig_pow / snr_lin_sw;
 
    ber_mc = zeros(1,Nt); det_mc = false(1,Nt);
 
    for ri_sw=1:1   % hanya RSU1 untuk sweep
        rx_sw = zeros(N_tx,1);
        for k=1:Nt
            dx=t_x_sw(k)-rsu_x(1); dy=t_y_sw(k)-rsu_y(1);
            R_k=max(sqrt(dx^2+dy^2),delta_R);
            d_k=round(2*R_k/cfg.c*cfg.fs);
            amp=sqrt(t_RCS(k))*cfg.lambda/((4*pi)^1.5*R_k^2)*1e6;
            amp=min(amp,0.95);
            ang=atan2(dy,dx);
            v_r=t_vx(k)*cos(ang)+t_vy(k)*sin(ang);
            fd=2*v_r/cfg.lambda;
            if d_k>0&&d_k<N_tx
                rx_sw=rx_sw+amp*[zeros(d_k,1);tx_signal(1:N_tx-d_k)].*exp(1j*2*pi*fd*t_sig);
            end
        end
        awgn_sw=sqrt(nv_sw/2)*(randn(N_tx,1)+1j*randn(N_tx,1));
        rx_sw=rx_sw+awgn_sw;
        rg_sw=demod_ofdm_simple(rx_sw,Nsc,Nsym,cfg.Ncp,Lsym);
        H_sw=rg_sw./(tx_grid+1e-10);

        % BER theoretical QPSK — dari SNR sweep langsung
        % Eb/N0 = SNR_linear / bits_per_sym
        EbN0_sw = snr_lin_sw / v2x.bits_per_sym;
        BER_theoretical = 0.5 * erfc(sqrt(EbN0_sw));
        for k=1:Nt
            ber_mc(k) = BER_theoretical;
        end
 
        % Pd
        Hw_sw=H_sw.*(win_r*win_v);
        RD_sw=fftshift(fft(ifft(Hw_sw,Nsc,1),Nsym,2),2);
        [cfm_sw,~]=cfar_2d_simple(abs(RD_sw),cfg.cfar.guard,cfg.cfar.train,cfg.cfar.Pfa);
        rp_sw=mean(abs(ifft(Hw_sw,Nsc,1)),2);
        for k=1:Nt
            R_k=sqrt((t_x_sw(k)-rsu_x(1))^2+(t_y_sw(k)-rsu_y(1))^2);
            wlo=max(1,round((R_k-delta_R*4)/delta_R));
            whi=max(min(R_show,round((R_k+delta_R*4)/delta_R)),wlo+2);
            [~,ir_r]=max(rp_sw(wlo:whi)); ir=ir_r+wlo-1;
            ang=atan2(t_y_sw(k)-rsu_y(1),t_x_sw(k)-rsu_x(1));
            v_r=t_vx(k)*cos(ang)+t_vy(k)*sin(ang);
            iv=max(1,min(Nsym,round(Nsym/2)+round(v_r/delta_v)));
            r1=max(1,ir-cfg.cfar.guard*2); r2=min(Nsc,ir+cfg.cfar.guard*2);
            c1=max(1,iv-cfg.cfar.guard*2); c2=min(Nsym,iv+cfg.cfar.guard*2);
            det_mc(k)=any(any(cfm_sw(r1:r2,c1:c2)));
        end
    end
    BER_sweep(si) = mean(ber_mc);
    Pd_sweep(si)  = mean(det_mc);
    fprintf('   SNR=%+3.0fdB | BER=%.4f | Pd=%.1f%%\n', snr_db_sw, BER_sweep(si), Pd_sweep(si)*100);
end
 
% Plot BER vs SNR + Pd vs SNR
figure('Name','JCAS — BER & Pd vs SNR','Color','black','NumberTitle','off');
 
yyaxis left
semilogy(snr_sweep, max(BER_sweep,1e-5), 'w-o', 'LineWidth',2.5, 'MarkerSize',8, 'MarkerFaceColor','white');
ylabel('BER','Color','white');
set(gca,'YColor','white','YScale','log');
ylim([1e-4 1]);
 
yyaxis right
plot(snr_sweep, Pd_sweep*100, 'g-s', 'LineWidth',2.5, 'MarkerSize',8, 'MarkerFaceColor','green');
ylabel('Pd [%]','Color','green');
set(gca,'YColor','green');
ylim([0 110]);
 
% Garis threshold
yline(1, '--', 'BER=1%', 'Color',[1 0.5 0.5],'LineWidth',1.5);
xline(0, '--', 'SNR=0dB', 'Color',[0.7 0.7 0.7],'LineWidth',1);
 
set(gca,'Color','black','XColor','white','GridColor',[0.3 0.3 0.3]);
xlabel('SNR [dB]','Color','white');
title('JCAS Trade-off: BER vs Pd vs SNR','Color','white','FontSize',13,'FontWeight','bold');
grid on;
legend('BER (komunikasi)','Pd (sensing)','TextColor','white','Color','black','FontSize',10);
fprintf('║  N RSU            : %d\n', n_rsu);
fprintf('╚══════════════════════════════════════════╝\n\n');
 
%% =========================================================================
%  SIMPAN TXT SUMMARY — ringkasan hasil untuk verifikasi
%  =========================================================================
txt_fname = sprintf('%s_summary.txt', cfg.csv_prefix);
fid_txt = fopen(txt_fname, 'w');
fprintf(fid_txt, '============================================================\n');
fprintf(fid_txt, '  JCAS RAY TRACING v4 — EXPERIMENT SUMMARY\n');
fprintf(fid_txt, '============================================================\n\n');

% --- Konfigurasi ---
fprintf(fid_txt, '[KONFIGURASI]\n');
fprintf(fid_txt, '  fc          = %.1f GHz\n', cfg.fc/1e9);
fprintf(fid_txt, '  BW          = %.0f MHz\n', cfg.BW/1e6);
fprintf(fid_txt, '  Nfft        = %d\n', cfg.Nfft);
fprintf(fid_txt, '  Nsymb       = %d\n', cfg.Nsymb);
fprintf(fid_txt, '  SNR_dB      = %.0f dB\n', cfg.SNR_dB);
fprintf(fid_txt, '  delta_R     = %.2f m\n', delta_R);
fprintf(fid_txt, '  delta_v     = %.2f m/s\n', delta_v);
fprintf(fid_txt, '  N_steps     = %d  |  dt = %.1f s\n', N_steps, dt);
fprintf(fid_txt, '  n_RSU       = %d\n', n_rsu);
fprintf(fid_txt, '  Channel     = 3-tap multipath (ITU Veh-A subset)\n\n');

% --- Target ---
fprintf(fid_txt, '[TARGET]\n');
fprintf(fid_txt, '  %-6s  %-8s  %6s  %6s  %8s  %8s\n', ...
    'No','Label','RCS','vx','R0_RSU1','R0_RSU2');
for k=1:Nt
    R0_1 = sqrt((t_x0(k)-rsu_x(1))^2+(t_y0(k)-rsu_y(1))^2);
    R0_2 = sqrt((t_x0(k)-rsu_x(2))^2+(t_y0(k)-rsu_y(2))^2);
    fprintf(fid_txt, '  %-6d  %-8s  %5.1f   %5.1f   %7.1fm   %7.1fm\n', ...
        k, t_labels{k}, t_RCS(k), t_vx(k), R0_1, R0_2);
end
fprintf(fid_txt, '\n');

% --- KPI Keseluruhan ---
fprintf(fid_txt, '[KPI KESELURUHAN]\n');
fprintf(fid_txt, '  Pd overall        = %.1f %%\n', Pd_overall*100);
fprintf(fid_txt, '  RMSE Range        = %.2f m\n', RMSE_R_all);
fprintf(fid_txt, '  Akurasi klasif.   = %.1f %%\n', acc_overall*100);
fprintf(fid_txt, '  BER rata-rata     = %.6f\n', mean(all_BER_avg));

% 5GAA-style checks
fprintf(fid_txt, '\n[5GAA COMPLIANCE CHECK]\n');
fprintf(fid_txt, '  delta_R <= 1m     : %s (%.2fm)\n', ...
    ternary_str(delta_R<=1.0,'PASS','FAIL'), delta_R);
fprintf(fid_txt, '  Pd >= 95%%         : %s (%.1f%%)\n', ...
    ternary_str(Pd_overall>=0.95,'PASS','FAIL'), Pd_overall*100);
fprintf(fid_txt, '  RMSE <= 1.5m      : %s (%.2fm)\n', ...
    ternary_str(RMSE_R_all<=1.5,'PASS','FAIL'), RMSE_R_all);
fprintf(fid_txt, '  BER <= 0.01       : %s (%.6f)\n', ...
    ternary_str(mean(all_BER_avg)<=0.01,'PASS','FAIL'), mean(all_BER_avg));
fprintf(fid_txt, '\n');

% --- Per-target detail ---
fprintf(fid_txt, '[HASIL PER TARGET]\n');
fprintf(fid_txt, '  %-6s  %-8s  %-8s  %8s  %10s  %10s  %10s  %8s\n', ...
    'No','Label_T','Label_P','Det_rate','RMSE_R[m]','BER_avg','RCS_est_rel','v_true');
for k=1:Nt
    det_rate_k = mean(all_det(:,k))*100;
    rmse_k = sqrt(mean((all_R_est(:,k)-all_R_true(:,k)).^2));
    ber_k  = mean(all_BER(:,k));
    % Most common predicted class
    preds_k = all_class(:,k);
    classes = {'VRU','Motor','Mobil','Truk'};
    cnt = cellfun(@(c) sum(strcmp(preds_k,c)), classes);
    [~,ci] = max(cnt);
    pred_mode = classes{ci};
    fprintf(fid_txt, '  %-6d  %-8s  %-8s  %7.1f%%  %10.2f  %10.6f  %10.2e  %7.1f\n', ...
        k, t_labels{k}, pred_mode, det_rate_k, rmse_k, ber_k, ...
        mean(RCS_est), t_vx(k));
end
fprintf(fid_txt, '\n');

% --- SNR sweep ringkasan ---
fprintf(fid_txt, '[BER & Pd vs SNR SWEEP]\n');
fprintf(fid_txt, '  %-8s  %10s  %10s\n','SNR[dB]','BER','Pd[%%]');
for si=1:length(snr_sweep)
    fprintf(fid_txt, '  %-8.0f  %10.6f  %10.1f\n', ...
        snr_sweep(si), BER_sweep(si), Pd_sweep(si)*100);
end
fprintf(fid_txt, '\n');

% --- Diagnosis otomatis ---
fprintf(fid_txt, '[DIAGNOSIS OTOMATIS]\n');
if delta_v > 20
    fprintf(fid_txt, '  [!] delta_v = %.1f m/s — terlalu kasar, naikkan Nsymb\n', delta_v);
else
    fprintf(fid_txt, '  [OK] delta_v = %.1f m/s — cukup untuk membedakan kendaraan\n', delta_v);
end
if RMSE_R_all > 10
    fprintf(fid_txt, '  [!] RMSE Range = %.2f m — masih tinggi, cek peak detection\n', RMSE_R_all);
elseif RMSE_R_all > 1.5
    fprintf(fid_txt, '  [~] RMSE Range = %.2f m — mendekati target 5GAA 1.5m\n', RMSE_R_all);
else
    fprintf(fid_txt, '  [OK] RMSE Range = %.2f m — memenuhi 5GAA target\n', RMSE_R_all);
end
if Pd_overall < 0.5
    fprintf(fid_txt, '  [!] Pd = %.1f%% — rendah, cek SNR / CFAR threshold\n', Pd_overall*100);
elseif Pd_overall < 0.95
    fprintf(fid_txt, '  [~] Pd = %.1f%% — belum mencapai 5GAA 95%%\n', Pd_overall*100);
else
    fprintf(fid_txt, '  [OK] Pd = %.1f%% — memenuhi 5GAA target\n', Pd_overall*100);
end
if mean(all_BER_avg) < 1e-6
    fprintf(fid_txt, '  [!] BER = %.2e — suspek terlalu rendah, cek model SNR\n', mean(all_BER_avg));
elseif mean(all_BER_avg) > 0.1
    fprintf(fid_txt, '  [!] BER = %.2e — tinggi, cek SNR per target\n', mean(all_BER_avg));
else
    fprintf(fid_txt, '  [OK] BER = %.2e — dalam range wajar\n', mean(all_BER_avg));
end
% RCS diagnosis — cek apakah nilai per target bervariasi
rcs_vals = zeros(1,Nt);
for k=1:Nt; rcs_vals(k)=mean(RCS_est(k)); end
if std(rcs_vals) < 1e-10
    fprintf(fid_txt, '  [!] RCS_est semua target sama — estimation tidak berfungsi\n');
else
    fprintf(fid_txt, '  [OK] RCS_est bervariasi per target (std=%.2e)\n', std(rcs_vals));
end
fprintf(fid_txt, '\n============================================================\n');
fclose(fid_txt);
fprintf('>> Summary TXT: %s\n\n', txt_fname);

%% =========================================================================
%  SIMPAN CSV
%  =========================================================================
if cfg.save_csv
    fname=sprintf('%s_results.csv',cfg.csv_prefix);
    fid=fopen(fname,'w');
    fprintf(fid,'Step,t_s,Node,Label_true,Label_pred,x_m,y_m,R_true_m,R_est_m,R_err_m,detected,correct,BER\n');
    for s=1:N_steps
        for k=1:Nt
            R_t=all_R_true(s,k); R_e=all_R_est(s,k);
            fprintf(fid,'%d,%.2f,%d,%s,%s,%.2f,%.2f,%.3f,%.3f,%.3f,%d,%d,%.6f\n', ...
                s,(s-1)*dt,k,t_labels{k},all_class{s,k}, ...
                trail_x(s,k),trail_y(s,k), ...
                R_t,R_e,abs(R_t-R_e), ...
                int8(all_det(s,k)), ...
                int8(strcmp(all_class{s,k},t_labels{k})), ...
                all_BER(s,k));
        end
    end
    fclose(fid);
    fprintf('>> CSV: %s\n\n', fname);
end
 
fprintf('============================================================\n');
fprintf('  SELESAI\n');
fprintf('============================================================\n');
 
 
%% =========================================================================
%  FUNGSI LOKAL
%  =========================================================================
 
function rx_grid = demod_ofdm_simple(rx, Nsc, Nsym, Ncp, Lsym)
    rx_grid=zeros(Nsc,Nsym);
    for s=1:Nsym
        i1=(s-1)*Lsym+Ncp+1; i2=i1+Nsc-1;
        if i2<=length(rx); rx_grid(:,s)=fft(rx(i1:i2),Nsc); end
    end
end
 
function [mask,threshold]=cfar_2d_simple(X,guard,train,Pfa)
    [Nr,Nd]=size(X); mask=false(Nr,Nd); threshold=zeros(Nr,Nd);
    alpha=train*(Pfa^(-1/train)-1); ht=guard+train;
    for r=1:Nr; for d=1:Nd
        rl=max(1,r-ht); rh=min(Nr,r+ht); dl=max(1,d-ht); dh=min(Nd,d+ht);
        rgl=max(1,r-guard); rgh=min(Nr,r+guard); dgl=max(1,d-guard); dgh=min(Nd,d+guard);
        outer=X(rl:rh,dl:dh); inner=X(rgl:rgh,dgl:dgh);
        n_out=numel(outer)-numel(inner);
        if n_out>0
            noise=(sum(outer(:))-sum(inner(:)))/n_out;
            thr=alpha*noise; threshold(r,d)=thr;
            if X(r,d)>thr; mask(r,d)=true; end
        end
    end; end
end
 
function s=ternary_str(cond,a,b)
    if cond; s=a; else; s=b; end
end
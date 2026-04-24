%% JCAS_GeneratePubFigs.m
%  Generate all 10 publication figures from CSV output files.
%  Compatible with JCAS_OFDM_Multistatic_Core_TB.m output.
%  Galih Nugraha Nurkahfi -- PR Telekomunikasi, BRIN -- v2.0

clear; clc; close all;

csv_dir     = 'JCAS_Results';
out_dir     = 'figures';
primary_sfx = '_Primary_BW20MHz_6nodes_ModeC';

script_dir = fileparts(mfilename('fullpath'));
out_dir = fullfile(script_dir, out_dir);
if ~exist(out_dir,'dir'); mkdir(out_dir); end

C.static=[0.122 0.471 0.706]; C.wf=[1.000 0.498 0.055];
C.adapt=[0.173 0.627 0.173];  C.ql=[0.839 0.153 0.157];
C.sarsa=[0.580 0.404 0.741];  C.bandit=[0.549 0.337 0.294];
C.pass=[0.173 0.627 0.173];   C.fail=[0.839 0.153 0.157];
colors=[C.static;C.wf;C.adapt;C.ql;C.sarsa;C.bandit];
strat_names={'Static','Water-Fill','Adaptive','Q-Learning','SARSA','Greedy Bandit'};
markers={'o','s','^','d','v','p'};

fprintf('\n=== JCAS Publication Figure Generator v2.0 ===\n\n');

%% ── PubFig2: Capacity bar chart ──────────────────────────────────────────
T1 = jcas_load(csv_dir,1,primary_sfx);
if ~isempty(T1) && ismember('Cap_Mbps',T1.Properties.VariableNames)
    fh=figure('Units','centimeters','Position',[2 2 18 11],'Color','w');
    cap=T1.Cap_Mbps; n=min(length(cap),6);
    % Abbreviated labels so x-axis font can be large
    abbr = {'Static','WF','Adapt','Q-L','SARSA','Bandit'};
    b=bar(cap(1:n),0.65,'FaceColor','flat');
    for k=1:n; b.CData(k,:)=colors(k,:); end
    hold on;
    yline(0.2,'--','Color',C.pass,'LineWidth',1.8,'Label','5GAA SLR');
    xticks(1:n); xticklabels(abbr(1:n));
    ax=gca; ax.XAxis.FontSize=13; ax.YAxis.FontSize=12;
    xlabel('Strategy','FontSize',13);
    ylabel('Capacity (Mbps)','FontSize',13);
    title('Spectral Efficiency per Strategy — Primary Scenario (6 nodes, 20 MHz, Mode C)',...
          'FontWeight','normal','FontSize',11);
    for k=1:n
        text(k,cap(k)+max(cap)*0.02,sprintf('%.1f',cap(k)),...
             'HorizontalAlignment','center','FontSize',11,...
             'FontWeight','bold','Color',colors(k,:));
    end
    % Legend with full names
    leg_h = gobjects(n,1);
    for k=1:n
        leg_h(k)=bar(nan,nan,'FaceColor',colors(k,:),'DisplayName',strat_names{k});
    end
    legend(leg_h,'Location','northeast','FontSize',10,'NumColumns',2);
    ylim([0 max(cap)*1.18]); grid on; box on;
    jcas_export(fh,out_dir,'PubFig2_BarKPI');
end

%% ── PubFig1: Pareto frontier ─────────────────────────────────────────────
T2 = jcas_load(csv_dir,2,primary_sfx);
if ~isempty(T2) && ismember('Cap_Mbps',T2.Properties.VariableNames)
    fh=figure('Units','centimeters','Position',[2 2 18 11],'Color','w'); hold on;
    strats_u=unique(T2.Strategy,'stable');
    hh=gobjects(length(strats_u),1);
    for si=1:length(strats_u)
        idx=strcmp(T2.Strategy,strats_u{si});
        rm=T2.RMSE_m(idx); cp=T2.Cap_Mbps(idx); col=colors(si,:);
        if length(unique(rm))==1
            hh(si)=scatter(rm(1),cp(1),80,col,markers{si},'filled',...
                'LineWidth',1.5,'DisplayName',strats_u{si});
        else
            hh(si)=plot(rm,cp,'-','Color',col,'LineWidth',1.8,...
                'Marker',markers{si},'MarkerSize',5,...
                'MarkerFaceColor',col,'DisplayName',strats_u{si});
        end
    end
    xline(1.5,'--','Color',C.pass,'LineWidth',1.5,'Label','5GAA RMSE 1.5 m');
    % Auto-fit x-axis to actual data range (no empty canvas)
    all_rm = T2.RMSE_m(isfinite(T2.RMSE_m));
    x_lo = max(0, min(all_rm)*0.85);
    x_hi = max(all_rm)*1.1;
    % Only use log scale if range spans more than one decade
    if x_hi/max(x_lo,0.01) > 10
        set(gca,'XScale','log');
    else
        set(gca,'XScale','linear');
        xlim([x_lo x_hi]);
    end
    xlabel('EKF RMSE (m)','FontSize',12);
    ylabel('Capacity (Mbps)','FontSize',12);
    title('Communication-Sensing Pareto Frontier — Primary Scenario',...
          'FontWeight','normal','FontSize',11);
    legend(hh,'Location','northwest','FontSize',9,'NumColumns',2);
    grid on; box on;
    jcas_export(fh,out_dir,'PubFig1_Pareto');
end

%% ── PubFig3: Pd vs SNR ───────────────────────────────────────────────────
% CSV03 columns: SNR_dB, Pd_pct, MissedDet_pct, BER, BER_QPSK_theory,
%                BER_16QAM_theory, Pfa_emp_pct, Pd_5GAA, Pfa_5GAA
scen3={primary_sfx,'Primary 6-node'; '','8-node standalone';
    '_BWsweep_BW10MHz_6nodes_ModeC','10MHz';
    '_BWsweep_BW40MHz_6nodes_ModeC','40MHz';
    '_Density_BW20MHz_4nodes_ModeC','4-node';
    '_Density_BW20MHz_10nodes_ModeC','10-node';
    '_Topology_BW20MHz_6nodes_ModeB','ModeB';
    '_Topology_BW20MHz_6nodes_Mono','Mono'};
fh=figure('Units','centimeters','Position',[2 2 18 11],'Color','w'); hold on;
sc=lines(size(scen3,1)); hh3=gobjects(size(scen3,1),1);
for si=1:size(scen3,1)
    T3=jcas_load(csv_dir,3,scen3{si,1});
    if isempty(T3)||~ismember('Pd_pct',T3.Properties.VariableNames); continue; end
    hh3(si)=plot(T3.SNR_dB,T3.Pd_pct,'-o','Color',sc(si,:),...
        'LineWidth',1.5,'MarkerSize',4,'MarkerFaceColor',sc(si,:),...
        'DisplayName',scen3{si,2});
end
yline(95,'--r','LineWidth',2,'Label','5GAA: Pd \geq 95%');
xlabel('SNR (dB)'); ylabel('Detection Probability P_d (%)');
title('Detection Probability vs. SNR — All Scenarios','FontWeight','normal');
ylim([0 110]); xlim([-6 26]); grid on; box on;
vld=arrayfun(@(x)isvalid(x)&&~strcmp(x.Type,'hggroup'),hh3);
if any(vld); legend(hh3(vld),'Location','northwest','FontSize',7,'NumColumns',2); end
jcas_export(fh,out_dir,'PubFig3_PdBER_vs_SNR');

%% ── PubFig4: BW Comparison ───────────────────────────────────────────────
% CSV05 columns: BW_MHz, Nsc, RangeRes_m, CRB_m, RMSE_approx_m, Pass_5GAA
bw4={primary_sfx,'Primary 6-node'; '','8-node standalone';
    '_Density_BW20MHz_4nodes_ModeC','4-node';
    '_Density_BW20MHz_10nodes_ModeC','10-node';
    '_Topology_BW20MHz_6nodes_ModeB','ModeB';
    '_Topology_BW20MHz_6nodes_Mono','Mono'};
fh=figure('Units','centimeters','Position',[2 2 18 11],'Color','w'); hold on;
bc=lines(size(bw4,1)); hh4=gobjects(size(bw4,1),1);
for si=1:size(bw4,1)
    T5=jcas_load(csv_dir,5,bw4{si,1});
    if isempty(T5)||~ismember('RMSE_approx_m',T5.Properties.VariableNames); continue; end
    hh4(si)=plot(T5.BW_MHz,T5.RMSE_approx_m,'-s','Color',bc(si,:),...
        'LineWidth',1.8,'MarkerSize',6,'MarkerFaceColor',bc(si,:),...
        'DisplayName',bw4{si,2});
end
yline(1.5,'--','Color',C.pass,'LineWidth',2,'Label','5GAA RMSE \leq 1.5 m');
xlabel('Bandwidth (MHz)'); ylabel('EKF RMSE (m)');
title('EKF RMSE vs. Bandwidth','FontWeight','normal');
set(gca,'XTick',[10 20 40]); grid on; box on;
vld4=arrayfun(@(x)isvalid(x)&&~strcmp(x.Type,'hggroup'),hh4);
if any(vld4); legend(hh4(vld4),'Location','northeast','FontSize',8); end
jcas_export(fh,out_dir,'PubFig4_BW_Comparison');

%% ── PubFig5: RL Convergence ──────────────────────────────────────────────
% CSV09 columns: Episode, Q-Learning_mavg, SARSA_mavg, Greedy Bandit_mavg
T9=jcas_load(csv_dir,9,primary_sfx);
if ~isempty(T9)
    cols9=T9.Properties.VariableNames;
    fh=figure('Units','centimeters','Position',[2 2 18 10],'Color','w'); hold on;
    rlc=[C.ql;C.sarsa;C.bandit];
    ep=T9.(cols9{1});
    for ai=1:min(3,length(cols9)-1)
        dlbl=strrep(strrep(cols9{ai+1},'_mavg',''),'_',' ');
        plot(ep,T9.(cols9{ai+1}),'-','Color',rlc(ai,:),...
             'LineWidth',1.5,'DisplayName',dlbl);
    end
    xlabel('Episode'); ylabel('Mean Reward (100-ep MA)');
    title('RL Convergence — Primary Scenario','FontWeight','normal');
    legend('Location','southeast','FontSize',9);
    xlim([0 max(ep)]); grid on; box on;
    jcas_export(fh,out_dir,'PubFig5_RL_Convergence');
end

%% ── PubFig6: Bootstrap CI ────────────────────────────────────────────────
% CSV07 columns: Strategy, Mean_Cap_Mbps, Std_Cap_Mbps,
%                CI_Low95_Mbps, CI_High95_Mbps, CI_Width_Mbps, ...
T7=jcas_load(csv_dir,7,primary_sfx);
if ~isempty(T7)&&ismember('Mean_Cap_Mbps',T7.Properties.VariableNames)
    fh=figure('Units','centimeters','Position',[2 2 18 10],'Color','w'); hold on;
    n7=height(T7);
    for k=1:n7
        col=colors(k,:);
        mn=T7.Mean_Cap_Mbps(k); cil=T7.CI_Low95_Mbps(k); cih=T7.CI_High95_Mbps(k);
        plot([k k],[cil cih],'-','Color',col,'LineWidth',3);
        plot([k-0.15 k+0.15],[cil cil],'-','Color',col,'LineWidth',2);
        plot([k-0.15 k+0.15],[cih cih],'-','Color',col,'LineWidth',2);
        scatter(k,mn,60,col,'d','filled');
        text(k,mn+3,sprintf('%.1f',mn),'HorizontalAlignment','center',...
             'FontSize',8,'Color',col);
    end
    xticks(1:n7);
    xlbls=T7.Strategy;
    if iscell(xlbls); xticklabels(xlbls); else; xticklabels(cellstr(xlbls)); end
    xlabel('Strategy'); ylabel('Capacity (Mbps)');
    title('Bootstrap 95% CI (BCa, n=1000) — Primary Scenario','FontWeight','normal');
    text(0.02,0.97,'p = 1.0 vs Static (deterministic policy outcomes)',...
         'Units','normalized','FontSize',8,'VerticalAlignment','top',...
         'BackgroundColor',[1 1 0.8],'EdgeColor',[0.8 0.8 0]);
    ylim([0 130]); grid on; box on;
    jcas_export(fh,out_dir,'PubFig6_Bootstrap_CI');
end

%% ── PubFig7: Sensitivity ─────────────────────────────────────────────────
% CSV08 columns: w_c, w_s, w_d, QL_Cap_Mbps, SARSA_Cap_Mbps, Diff_Mbps, QL_Tput_5GAA
T8=jcas_load(csv_dir,8,primary_sfx);
if ~isempty(T8)&&ismember('QL_Cap_Mbps',T8.Properties.VariableNames)
    fh=figure('Units','centimeters','Position',[2 2 18 10],'Color','w'); hold on;
    plot(T8.w_c,T8.QL_Cap_Mbps,'-o','Color',C.ql,'LineWidth',1.8,...
         'MarkerSize',6,'MarkerFaceColor',C.ql,'DisplayName','Q-Learning');
    plot(T8.w_c,T8.SARSA_Cap_Mbps,'-s','Color',C.sarsa,'LineWidth',1.8,...
         'MarkerSize',6,'MarkerFaceColor',C.sarsa,'DisplayName','SARSA');
    xlabel('Communication Weight w_c  (w_s = 0.9 - w_c,  w_d = 0.1)');
    ylabel('Capacity (Mbps)');
    title('Reward Weight Sensitivity — Primary Scenario','FontWeight','normal');
    legend('Location','northwest','FontSize',9);
    xticks(T8.w_c); grid on; box on;
    jcas_export(fh,out_dir,'PubFig7_Sensitivity_Weight');
end

%% ── PubFig8: Ablation ────────────────────────────────────────────────────
% CSV10 columns: Configuration, QL_Cap_Mbps, Delta_vs_Full_Mbps, Delta_pct, Tput_5GAA
abl8={primary_sfx,'Primary 6-node';
    '_BWsweep_BW40MHz_6nodes_ModeC','40MHz';
    '_Density_BW20MHz_10nodes_ModeC','10-node';
    '_Topology_BW20MHz_6nodes_ModeB','ModeB'};
abl_data=NaN(4,size(abl8,1)); abl_ok=false(1,size(abl8,1));
for si=1:size(abl8,1)
    Ta=jcas_load(csv_dir,10,abl8{si,1});
    if ~isempty(Ta)&&ismember('QL_Cap_Mbps',Ta.Properties.VariableNames)&&height(Ta)>=4
        vals=Ta.QL_Cap_Mbps(1:4);
        if iscell(vals); vals=cellfun(@str2double,vals); end
        abl_data(:,si)=vals; abl_ok(si)=true;
    end
end
if any(abl_ok)
    fh=figure('Units','centimeters','Position',[2 2 18 10],'Color','w');
    b8=bar(abl_data(:,abl_ok),0.75);
    ac=lines(sum(abl_ok));
    for k=1:sum(abl_ok); b8(k).FaceColor=ac(k,:); end
    xticks(1:4); xticklabels({'Full','No Detect','No Comm','No Sense'});
    xlabel('Reward Configuration'); ylabel('Q-Learning Capacity (Mbps)');
    title('Ablation Study: Effect of Reward Component Removal','FontWeight','normal');
    legend(abl8(abl_ok,2),'Location','northeast','FontSize',8);
    grid on; box on;
    jcas_export(fh,out_dir,'PubFig8_Ablation');
end

%% ── PubFig9: Generalization heatmap ──────────────────────────────────────
% CSV11 columns: Scenario, Cap_Mbps, RMSE_m, Tput_5GAA, RangeRes_5GAA
gen9={'_Primary_BW20MHz_6nodes_ModeC','Primary';
    '_Topology_BW20MHz_6nodes_ModeB','ModeB';
    '_Topology_BW20MHz_6nodes_Mono','Mono';
    '_BWsweep_BW10MHz_6nodes_ModeC','10 MHz';
    '_BWsweep_BW40MHz_6nodes_ModeC','40 MHz'};
gm=NaN(5,3);
for ri=1:size(gen9,1)
    Tg=jcas_load(csv_dir,11,gen9{ri,1});
    if ~isempty(Tg)&&ismember('Cap_Mbps',Tg.Properties.VariableNames)&&height(Tg)>=3
        gm(ri,:)=Tg.Cap_Mbps(1:3)';
    end
end
if any(~isnan(gm(:)))
    fh=figure('Units','centimeters','Position',[2 2 18 11],'Color','w');
    % Grouped bar: rows=config, cols=density (Low/Med/High)
    % Transpose so bars per group = density levels
    gm_plot = gm;
    gm_plot(isnan(gm_plot)) = 0;
    b9 = bar(gm_plot, 0.75, 'grouped');
    den_colors = [0.2 0.5 0.8; 0.4 0.7 0.3; 0.85 0.33 0.1];
    for k=1:3
        b9(k).FaceColor = den_colors(k,:);
        b9(k).DisplayName = {'Low','Medium','High'}{k};
    end
    hold on;
    % 5GAA throughput SLR line
    yline(0.2,'--k','LineWidth',1.5,'Label','5GAA SLR 0.2 Mbps',...
          'LabelHorizontalAlignment','left');
    % Value labels on bars
    for gi=1:size(gm_plot,1)
        for di=1:3
            if gm(gi,di)>0
                % Get bar x position
                nb = size(gm_plot,1);
                bw = 0.75/3;
                xpos = gi + (di-2)*bw;
                text(xpos, gm_plot(gi,di)+3, sprintf('%.0f',gm_plot(gi,di)),...
                     'HorizontalAlignment','center','FontSize',7,...
                     'Color',den_colors(di,:));
            end
        end
    end
    xticks(1:size(gen9,1));
    xticklabels(gen9(:,2));
    ax=gca; ax.XAxis.FontSize=11; ax.YAxis.FontSize=11;
    xlabel('Configuration','FontSize',12);
    ylabel('Q-Learning Capacity (Mbps)','FontSize',12);
    title('Cross-Scenario Generalisation of Q-Learning Policy',...
          'FontWeight','normal','FontSize',11);
    legend({'Low density','Medium density','High density'},...
           'Location','northeast','FontSize',10);
    grid on; box on; ylim([0 max(gm_plot(:))*1.2]);
    jcas_export(fh,out_dir,'PubFig9_Generalization');
end

%% ── PubFig10: Per-node RMSE ──────────────────────────────────────────────
% CSV04 columns: Node, Type, RCS_m2, Range_m, Speed_ms, Lane,
%                RawRMSE_m, EKFRMSE_m, EKF_Improv_pct, Pass_5GAA
T4=jcas_load(csv_dir,4,'');  % standalone run (8 nodes)
if isempty(T4)||~ismember('EKFRMSE_m',T4.Properties.VariableNames)
    T4=jcas_load(csv_dir,4,primary_sfx);
end
if ~isempty(T4)&&ismember('EKFRMSE_m',T4.Properties.VariableNames)
    T4=T4(~isnan(T4.Node),:);
    n4=height(T4);
    fh=figure('Units','centimeters','Position',[2 2 18 10],'Color','w'); hold on;
    bar((1:n4)-0.2,T4.RawRMSE_m,0.35,'FaceColor',[0.68 0.78 0.91],...
        'EdgeColor','none','DisplayName','Raw RMSE');
    p5g=strcmp(T4.Pass_5GAA,'PASS');
    for k=1:n4
        col=C.pass; if ~p5g(k); col=C.fail; end
        bar(k+0.2,T4.EKFRMSE_m(k),0.35,'FaceColor',col,'EdgeColor','none');
        text(k+0.2,T4.EKFRMSE_m(k)+0.12,sprintf('%.2f',T4.EKFRMSE_m(k)),...
             'HorizontalAlignment','center','FontSize',7,'Color',col);
    end
    yline(1.5,'--','Color',C.pass,'LineWidth',1.5,...
          'Label','5GAA 1.5 m','LabelHorizontalAlignment','left');
    xlbls=arrayfun(@(k)sprintf('N%d\n%s',T4.Node(k),T4.Type{k}),1:n4,'UniformOutput',false);
    xticks(1:n4); xticklabels(xlbls);
    xlabel('Node'); ylabel('RMSE (m)');
    title('Per-Node EKF RMSE vs. Raw RMSE','FontWeight','normal');
    hp(1)=bar(nan,nan,'FaceColor',[0.68 0.78 0.91],'EdgeColor','none');
    hp(2)=bar(nan,nan,'FaceColor',C.pass,'EdgeColor','none');
    hp(3)=bar(nan,nan,'FaceColor',C.fail,'EdgeColor','none');
    legend(hp,{'Raw RMSE','EKF RMSE (PASS)','EKF RMSE (FAIL)'},...
           'Location','northeast','FontSize',8);
    grid on; box on;
    jcas_export(fh,out_dir,'PubFig10_PerNode_RMSE');
end

fprintf('\n=== Done. Figures saved to: %s ===\n',out_dir);

%% ── LOCAL FUNCTIONS ──────────────────────────────────────────────────────────

function T = jcas_load(csv_dir, num, sfx)
% Load a numbered CSV file, auto-detect and skip metadata header rows.
% Header detection uses startsWith on the first-column name.
    labels={'Paper_Summary','Pareto_Sweep','Pd_vs_SNR','PerNode_RMSE',...
            'BW_Comparison','RL_Training','Statistical_Significance',...
            'Sensitivity_Analysis','Convergence','Ablation','Generalization'};
    base=sprintf('CSV%02d_%s',num,labels{num});
    fp=fullfile(csv_dir,[base sfx '.csv']);
    if ~exist(fp,'file')
        fp=fullfile(csv_dir,[base '.csv']);
    end
    if ~exist(fp,'file')
        T=table(); return;
    end
    % Known first-column names for each CSV data header row
    first_cols={'Strategy,','SNR_dB,','BW_MHz,','Node,',...
                'Episode,','w_c,','Configuration,','Scenario,Cap_Mbps'};
    raw=readlines(fp);
    for i=1:length(raw)
        ln=strtrim(raw(i));
        for p=1:length(first_cols)
            if startsWith(ln,first_cols{p})
                T=readtable(fp,'NumHeaderLines',i-1,...
                            'VariableNamingRule','preserve');
                return;
            end
        end
    end
    T=readtable(fp,'VariableNamingRule','preserve');
end

function jcas_export(fh, out_dir, fname)
    set(fh,'PaperPositionMode','auto','Color','w');
    exportgraphics(fh,fullfile(out_dir,[fname '.jpg']),...
        'Resolution',300,'BackgroundColor','white');
    fprintf('[EXPORT] %s.jpg\n',fname);
end

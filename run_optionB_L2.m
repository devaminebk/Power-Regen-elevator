%% run_optionB_L2.m  --  simulation d'un trajet de l'option B (architecture C), niveau L2
clear; clc; close all;
run('params_elevator.m');

% ---- Scénario : pire cas pour la régénération (FR-05 : même P(t) pour toutes les options)
lambda  = 1.0;      % fraction de charge
dirSign = -1;       % -1 = descente, +1 = montée
[Pdrive_tbl, info] = gen_Pt_table(lambda, dirSign, H_travel, par, 1e-3, 1.0, 2.0);
fprintf('Trajet %.1f s | pic de régénération %.2f kW | énergie régénérée %.4f kWh | énergie moteur %.4f kWh\n', ...
        info.T_trip, info.P_regen_peak/1e3, info.E_regen_kWh, info.E_mot_kWh);

% ---- Construction et simulation
build_optionB_L2;
set_param(mdl,'StopTime',num2str(Pdrive_tbl(end,1)));
out = sim(mdl);

% ---- Post-traitement sur une base de temps commune
tt = (0:Ts_ctrl:Pdrive_tbl(end,1)).';
rs = @(ts) resample_ts(ts, tt);
Vdc  = rs(out.Vdc);       Pdrv = rs(out.Pdrive);
idc  = rs(out.idc_conv);  gate = rs(out.chop_gate);
dbg  = rs(out.dbg);       Ig   = rs(out.Iabc_grid);   Vg = rs(out.Vabc_pcc);

P_conv   = idc.*Vdc;                          % puissance du convertisseur de régénération vers le réseau [W]
P_chop   = gate.*Vdc.^2/Rb_B;                 % puissance dissipée dans la résistance de secours [W]
P_grid   = sum(Vg.*Ig, 2);                    % puissance nette soutirée au réseau au PCC [W]
t_skip   = 0.5;                               % ignorer le démarrage (charge du condensateur vide, précharge hors périmètre)
win      = tt >= t_skip;
kWh      = @(p) trapz(tt(win), p(win))/3.6e6;

E_regen_dc = -kWh(min(Pdrv,0));
E_to_grid  =  kWh(max(P_conv,0));
E_chopper  =  kWh(P_chop);
fprintf('\n--- Option B, L2 (convertisseur sans pertes) ---\n');
fprintf('Énergie régénérée au bus DC : %.4f kWh\n', E_regen_dc);
fprintf('Énergie restituée au réseau : %.4f kWh  (taux de récupération %.1f %%)\n', E_to_grid, 100*E_to_grid/E_regen_dc);
fprintf('Énergie dissipée en résistance : %.4f kWh\n', E_chopper);
fprintf('Vdc maximale                : %.1f V  (DR-03 <= %d V : %s)\n', max(Vdc(win)), Vtrip, okstr(max(Vdc(win)) <= Vtrip));
fprintf('Puissance réseau minimale   : %.2f kW (négative = export)\n', min(P_grid(win))/1e3);

% ---- Graphiques
figure('Name','Option B L2','Position',[100 100 900 750]);
subplot(4,1,1); plot(tt,Pdrv/1e3,tt,-P_conv/1e3,'--'); grid on; ylabel('kW');
legend('variateur P_{DC} (+ moteur)','-P_{conv. régénération}'); title('Puissance au bus DC');
subplot(4,1,2); plot(tt,Vdc); hold on; yline(Vreg_B,'--'); yline(Vdc_diode_max,':'); yline(Vtrip,'r');
grid on; ylabel('V_{dc} [V]'); title('Tension du bus DC (régulation / max. diodes / seuil de déclenchement)');
subplot(4,1,3); plot(tt,dbg(:,1),tt,dbg(:,3),'--',tt,dbg(:,2)); grid on; ylabel('A');
legend('i_d','i_d réf.','i_q'); title('Courants dq du convertisseur de régénération (vers le réseau)');
subplot(4,1,4); plot(tt,Ig); grid on; ylabel('A'); xlabel('t [s]'); title('Courants réseau totaux');

function s = okstr(b), if b, s = 'OK'; else, s = 'VIOLÉ'; end, end

function y = resample_ts(ts, tq)
    d = squeeze(ts.Data);
    if size(d,1) ~= numel(ts.Time), d = d.'; end
    [tu, iu] = unique(ts.Time);
    y = interp1(tu, d(iu,:), tq, 'linear', 'extrap');
end

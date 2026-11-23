%% run_optionB_L2.m  --  simulation d'un trajet de l'option B (architecture C), niveau L2
clear; clc; close all;
run('params_elevator.m');
% ---- Scénario : pire cas pour la régénération (FR-05 : même P(t) pour toutes les options)
fraction_charge = 1.0;      % fraction de charge
signeSens       = -1;       % -1 = descente, +1 = montée
[Pdrive_tbl, info] = gen_Pt_table(fraction_charge, signeSens, H_travel, par, 1e-3, 1.0, 2.0);
fprintf('Trajet %.1f s | pic de régénération %.2f kW | énergie régénérée %.4f kWh | énergie moteur %.4f kWh\n', ...
        info.T_trip, info.P_regen_peak/1e3, info.E_regen_kWh, info.E_mot_kWh);
% ---- Construction et simulation
build_optionB_L2;
set_param(modele,'StopTime',num2str(Pdrive_tbl(end,1)));
sortie = sim(modele);
% ---- Post-traitement sur une base de temps commune
tt = (0:Ts_ctrl:Pdrive_tbl(end,1)).';
reech = @(ts) reechantillonner_ts(ts, tt);
Vdc   = reech(sortie.Vdc);        Pvar = reech(sortie.Pvar);
idc   = reech(sortie.idc_conv);   gachette = reech(sortie.hacheur_gachette);
diagnostic = reech(sortie.diagnostic);
Ires  = reech(sortie.Iabc_reseau);   Vres = reech(sortie.Vabc_pcc);
P_conv    = idc.*Vdc;                         % puissance du convertisseur de régénération vers le réseau [W]
P_hacheur = gachette.*Vdc.^2/Rb_B;            % puissance dissipée dans la résistance de secours [W]
P_reseau  = sum(Vres.*Ires, 2);               % puissance nette soutirée au réseau au PCC [W]
t_ignore  = 0.5;                              % ignorer le démarrage (charge du condensateur vide, précharge hors périmètre)
fenetre   = tt >= t_ignore;
kWh       = @(p) trapz(tt(fenetre), p(fenetre))/3.6e6;
E_regen_dc     = -kWh(min(Pvar,0));
E_vers_reseau  =  kWh(max(P_conv,0));
E_hacheur      =  kWh(P_hacheur);
fprintf('\n--- Option B, L2 (convertisseur sans pertes) ---\n');
fprintf('Énergie régénérée au bus DC : %.4f kWh\n', E_regen_dc);
fprintf('Énergie restituée au réseau : %.4f kWh  (taux de récupération %.1f %%)\n', E_vers_reseau, 100*E_vers_reseau/E_regen_dc);
fprintf('Énergie dissipée en résistance : %.4f kWh\n', E_hacheur);
fprintf('Vdc maximale                : %.1f V  (DR-03 <= %d V : %s)\n', max(Vdc(fenetre)), Vtrip, texteOK(max(Vdc(fenetre)) <= Vtrip));
fprintf('Puissance réseau minimale   : %.2f kW (négative = export)\n', min(P_reseau(fenetre))/1e3);
% ---- Graphiques
figure('Name','Option B L2','Position',[100 100 900 750]);
subplot(4,1,1); plot(tt,Pvar/1e3,tt,-P_conv/1e3,'--'); grid on; ylabel('kW');
legend('variateur P_{DC} (+ moteur)','-P_{conv. régénération}'); title('Puissance au bus DC');
subplot(4,1,2); plot(tt,Vdc); hold on; yline(Vreg_B,'--'); yline(Vdc_diode_max,':'); yline(Vtrip,'r');
grid on; ylabel('V_{dc} [V]'); title('Tension du bus DC (régulation / max. diodes / seuil de déclenchement)');
subplot(4,1,3); plot(tt,diagnostic(:,1),tt,diagnostic(:,3),'--',tt,diagnostic(:,2)); grid on; ylabel('A');
legend('i_d','i_d réf.','i_q'); title('Courants dq du convertisseur de régénération (vers le réseau)');
subplot(4,1,4); plot(tt,Ires); grid on; ylabel('A'); xlabel('t [s]'); title('Courants réseau totaux');

%% ======================= fonctions locales d'aide ==========================
function s = texteOK(b), if b, s = 'OK'; else, s = 'VIOLÉ'; end, end
function y = reechantillonner_ts(ts, tq)
    d = squeeze(ts.Data);
    if size(d,1) ~= numel(ts.Time), d = d.'; end
    [tu, iu] = unique(ts.Time);
    y = interp1(tu, d(iu,:), tq, 'linear', 'extrap');
end
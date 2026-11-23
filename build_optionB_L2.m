%% build_optionB_L2.m
% Construit le modèle Simulink de l'ARCHITECTURE C / OPTION B au niveau L2 :
%   réseau -> Lg,Rg -> PCC -+-> L_acd -> pont à diodes -> bus DC (variateur existant)
%                           +-> Lf -> convertisseur de régénération moyen -> bus DC
%   Bus DC : Cdc, source de courant du variateur (table P(t)), hacheur de secours.
% Nécessite : Simulink + Simscape Electrical (Specialized Power Systems, 'powerlib').
% Exécuter d'abord params_elevator.m et créer Pdrive_tbl (gen_Pt_table) -- ou utiliser run_optionB_L2.m.

if ~exist('par','var'), run('params_elevator.m'); end

% --- Commutateurs de polarité : à vérifier une fois (voir le texte d'accompagnement), inverser si besoin
SIGNE_VSRC = +1;   % polarité des bornes de la source de tension du convertisseur
SIGNE_ISRC = +1;   % sens de la source de courant commandée (+1 = prélève de DC+ vers DC-)

modele = 'ElevatorRegen_OptB_L2';
if bdIsLoaded(modele), close_system(modele,0); end
new_system(modele);  open_system(modele);
load_system('powerlib');

pos = @(col,lig,larg,haut) [40+col*190, 40+lig*110, 40+col*190+larg, 40+lig*110+haut];   % aide au placement des blocs

%% ---- Blocs : côté alternatif -------------------------------------------------
ajouter_bloc(modele,'powerlib/powergui','powergui',[20 5 110 40],'SimulationMode','Discrete','SampleTime','Ts_power');
ajouter_bloc(modele,'powerlib/Electrical Sources/Three-Phase Source','Reseau',pos(0,1,100,90), ...
    'Voltage','V_LL','Frequency','f_g','PhaseAngle','0','InternalConnection','Yg');
ajouter_bloc(modele,'powerlib/Elements/Three-Phase Series RLC Branch','Zres',pos(1,1,100,90), ...
    'BranchType','RL','Resistance','Rg','Inductance','Lg');
ajouter_bloc(modele,'powerlib/Measurements/Three-Phase V-I Measurement','Mres',pos(2,1,100,90), ...
    'VoltageMeasurement','phase-to-ground','CurrentMeasurement','yes');
% branche à diodes (variateur existant)
ajouter_bloc(modele,'powerlib/Elements/Three-Phase Series RLC Branch','Zd',pos(3,0,100,90), ...
    'BranchType','RL','Resistance','R_acd','Inductance','L_acd');
ajouter_bloc(modele,'powerlib/Power Electronics/Universal Bridge','PontD',pos(4,0,100,110), ...
    'Arms','3','Device','Diodes','Ron','1e-3','ForwardVoltage','0.8', ...
    'SnubberResistance','1e5','SnubberCapacitance','inf','Measurements','None');
% branche de régénération
ajouter_bloc(modele,'powerlib/Measurements/Three-Phase V-I Measurement','Mreg',pos(3,3,100,90), ...
    'VoltageMeasurement','no','CurrentMeasurement','yes');
ajouter_bloc(modele,'powerlib/Elements/Three-Phase Series RLC Branch','Lf',pos(4,3,100,90), ...
    'BranchType','RL','Resistance','R_f','Inductance','L_f');
phases = 'abc';
for k = 1:3
    ajouter_bloc(modele,'powerlib/Electrical Sources/Controlled Voltage Source',['Vc' phases(k)], ...
        pos(5,2+k,70,50));
end

%% ---- Blocs : côté continu ----------------------------------------------------
ajouter_bloc(modele,'powerlib/Elements/Series RLC Branch','Cdc',pos(6,0,70,50),'BranchType','C','Capacitance','Cdc');
try
    set_param([modele '/Cdc'],'SetiL0','on','InitialVoltage','Vdc0');
catch
    warning('Impossible de fixer la tension initiale de Cdc ; la régler dans la boîte de dialogue du bloc (Vdc0).');
end
ajouter_bloc(modele,'powerlib/Electrical Sources/Controlled Current Source','Ivar',pos(6,1,70,50));
ajouter_bloc(modele,'powerlib/Electrical Sources/Controlled Current Source','Iconv',pos(6,2,70,50));
ajouter_bloc(modele,'powerlib/Power Electronics/Ideal Switch','IntB',pos(6,3,70,50),'Ron','1e-3','Rs','1e6','Cs','inf');
ajouter_bloc(modele,'powerlib/Elements/Series RLC Branch','RbB',pos(7,3,70,50),'BranchType','R','Resistance','Rb_B');
ajouter_bloc(modele,'powerlib/Measurements/Voltage Measurement','MVdc',pos(7,1,70,50));

%% ---- Blocs : signaux et commande --------------------------------------------
ajouter_bloc(modele,'simulink/Sources/From Workspace','Pvar',pos(8,0,90,40),'VariableName','Pdrive_tbl', ...
    'Interpolate','on','OutputAfterFinalValue','Setting to zero');
ajouter_bloc(modele,'simulink/Discontinuities/Saturation','satV',pos(8,1,60,30),'UpperLimit','2000','LowerLimit','100');
ajouter_bloc(modele,'simulink/Math Operations/Divide','divI',pos(9,0,50,40));
ajouter_bloc(modele,'simulink/Math Operations/Gain','gI_var',pos(10,0,50,30),'Gain','SIGNE_ISRC');
ajouter_bloc(modele,'simulink/Math Operations/Gain','gI_conv',pos(10,2,50,30),'Gain','SIGNE_ISRC');
ajouter_bloc(modele,'simulink/Discrete/Zero-Order Hold','bozV',pos(8,3,50,30),'SampleTime','Ts_ctrl');   % BOZ = bloqueur d'ordre zéro
ajouter_bloc(modele,'simulink/Discrete/Zero-Order Hold','bozI',pos(8,4,50,30),'SampleTime','Ts_ctrl');
ajouter_bloc(modele,'simulink/Discrete/Zero-Order Hold','bozD',pos(8,5,50,30),'SampleTime','Ts_ctrl');
ajouter_bloc(modele,'simulink/User-Defined Functions/MATLAB Function','ctrl_regen',pos(9,3,120,110));
% Les retards unitaires brisent les boucles algébriques (les sources commandées sont à action
% directe sur les mesures) et modélisent le retard de calcul d'un échantillon supposé dans Td = 1,5*Ts_ctrl.
for k = 1:3
    ajouter_bloc(modele,'simulink/Discrete/Unit Delay',['rV' phases(k)],pos(10,2+k,40,25),'SampleTime','Ts_ctrl');
end
ajouter_bloc(modele,'simulink/Discrete/Unit Delay','rIc',  pos(11,2,40,25),'SampleTime','Ts_ctrl');
ajouter_bloc(modele,'simulink/Discrete/Unit Delay','rIvar',pos(11,0,40,25),'SampleTime','Ts_power');
ajouter_bloc(modele,'simulink/Discontinuities/Relay','relaisHacheur',pos(9,5,60,30), ...
    'OnSwitchValue','Vchop_on','OffSwitchValue','Vchop_off','OnOutputValue','1','OffOutputValue','0');

%% ---- Loi de commande (intégrée sous forme de texte ; constantes issues de l'espace de travail)
% NB : les commentaires du code embarqué restent en ASCII (sans accents) par prudence d'encodage.
code = [
"function [vca, vcb, vcc, idc, diagnostic] = ctrl_regen(vabc, iabc, vdc)"
"% Commande de regeneration seule, option B : boucle a verrouillage de phase (PLL) en repere synchrone,"
"% boucle de tension DC unidirectionnelle, boucle de courant dq."
"persistent th wint xid xiq xv"
"if isempty(th)"
"    th = 0; wint = 0; xid = 0; xiq = 0; xv = 0;"
"end"
"Ts = @Ts@; w0 = @w0@; Vpk = @Vpk@; Lf = @Lf@; SGN = @SGN@;"
"kpi = @kpi@; kii = @kii@; kpv = @kpv@; kiv = @kiv@; kpp = @kpp@; kip = @kip@;"
"Vreg = @Vreg@; Idmax = @Idmax@;"
"va = vabc(1); vb = vabc(2); vc = vabc(3);"
"ia = -iabc(1); ib = -iabc(2); ic = -iabc(3);   % courant INJECTE dans le reseau"
"% Clarke (invariant en amplitude) et Park"
"va_al = (2/3)*(va - 0.5*vb - 0.5*vc);  va_be = (vb - vc)/sqrt(3);"
"ia_al = (2/3)*(ia - 0.5*ib - 0.5*ic);  ia_be = (ib - ic)/sqrt(3);"
"cs = cos(th); sn = sin(th);"
"vd =  va_al*cs + va_be*sn;   vq = -va_al*sn + va_be*cs;"
"id =  ia_al*cs + ia_be*sn;   iq = -ia_al*sn + ia_be*cs;"
"% PLL (PI sur vq normalise)"
"e_pll = vq/Vpk;"
"wint  = wint + kip*Ts*e_pll;"
"w     = w0 + kpp*e_pll + wint;"
"th_n  = th + w*Ts;  th_n = th_n - 2*pi*floor(th_n/(2*pi));"
"% Boucle de tension DC : sortie limitee a [0, Idmax] -> le convertisseur ne soutire jamais de puissance"
"vlim   = max(vdc,100)/sqrt(3);"
"ev     = vdc - Vreg;"
"idr_u  = kpv*ev + xv;"
"idref  = min(max(idr_u,0), Idmax);"
"if ~(idr_u >= Idmax && ev > 0)"
"    xv = min(max(xv + kiv*Ts*ev, 0), Idmax);"
"end"
"% Boucle de courant dq avec decouplage croise et anticipation de la tension reseau"
"eid = idref - id;  eiq = 0 - iq;"
"xid = min(max(xid + kii*Ts*eid, -vlim), vlim);"
"xiq = min(max(xiq + kii*Ts*eiq, -vlim), vlim);"
"vcd = vd - w*Lf*iq + kpi*eid + xid;"
"vcq = vq + w*Lf*id + kpi*eiq + xiq;"
"vm = hypot(vcd, vcq);"
"if vm > vlim, vcd = vcd*vlim/vm; vcq = vcq*vlim/vm; end"
"% retour en abc"
"v_al = vcd*cs - vcq*sn;  v_be = vcd*sn + vcq*cs;"
"va_c = v_al;  vb_c = -0.5*v_al + (sqrt(3)/2)*v_be;  vc_c = -0.5*v_al - (sqrt(3)/2)*v_be;"
"% cote DC moyen : bilan de puissance instantane (sans pertes au niveau L2)"
"p   = va_c*ia + vb_c*ib + vc_c*ic;      % puissance fournie au reseau"
"idc = p/max(vdc,100);                   % courant preleve sur le bus DC"
"vca = SGN*va_c; vcb = SGN*vb_c; vcc = SGN*vc_c;"
"diagnostic = [id; iq; idref; w/(2*pi); vd];"
"th = th_n;"
];
code = strjoin(code, newline);
jetons = {'@Ts@',Ts_ctrl; '@w0@',w_g; '@Vpk@',V_ph_pk; '@Lf@',L_f; '@SGN@',SIGNE_VSRC; ...
          '@kpi@',kp_i; '@kii@',ki_i; '@kpv@',kp_v; '@kiv@',ki_v; '@kpp@',kp_pll; ...
          '@kip@',ki_pll; '@Vreg@',Vreg_B; '@Idmax@',Id_max};
for n = 1:size(jetons,1)
    code = strrep(code, jetons{n,1}, num2str(jetons{n,2},'%.9g'));
end
racine = sfroot;
graphe = racine.find('-isa','Stateflow.EMChart','Path',[modele '/ctrl_regen']);
graphe.Script = char(code);
drawnow;

%% ---- Blocs d'enregistrement --------------------------------------------------
enreg_signal(modele,'Vdc',              'MVdc','O1',          pos(11,1,70,25));
enreg_signal(modele,'Pvar',             'Pvar','O1',          pos(11,0,70,25));
enreg_signal(modele,'Vabc_pcc',         'Mres','O1',          pos(3,5,70,25));
enreg_signal(modele,'Iabc_reseau',      'Mres','O2',          pos(4,5,70,25));
enreg_signal(modele,'Iabc_regen',       'Mreg','O1',          pos(5,6,70,25));
enreg_signal(modele,'idc_conv',         'ctrl_regen','O4',    pos(11,3,70,25));
enreg_signal(modele,'diagnostic',       'ctrl_regen','O5',    pos(11,4,70,25));
enreg_signal(modele,'hacheur_gachette', 'relaisHacheur','O1', pos(11,5,70,25));

%% ---- Câblage de puissance : côté alternatif (ports triphasés L1..3 / R1..3) ----
for k = 1:3
    s = num2str(k);
    relier(modele,'Reseau',['R' s],  'Zres', ['L' s]);
    relier(modele,'Zres',  ['R' s],  'Mres', ['L' s]);
    relier(modele,'Mres',  ['R' s],  'Zd',   ['L' s]);      % branche PCC 1 : chemin des diodes
    relier(modele,'Mres',  ['R' s],  'Mreg', ['L' s]);      % branche PCC 2 : chemin de régénération
    relier(modele,'Zd',    ['R' s],  'PontD',['L' s]);
    relier(modele,'Mreg',  ['R' s],  'Lf',   ['L' s]);
    relier(modele,'Lf',    ['R' s],  ['Vc' phases(k)], 'T2');  % borne de phase du convertisseur
end
relier(modele,'Vca','T1','Vcb','T1');  relier(modele,'Vca','T1','Vcc','T1');   % neutre flottant

%% ---- Câblage de puissance : bus DC (DC+ = PontD/R1, DC- = PontD/R2) ----------
for nom = {'Cdc','Ivar','Iconv','IntB','MVdc'}
    relier(modele,'PontD','R1', nom{1},'T1');                % DC+
end
for nom = {'Cdc','Ivar','Iconv','RbB','MVdc'}
    relier(modele,'PontD','R2', nom{1},'T2');                % DC-
end
relier(modele,'IntB','T2','RbB','T1');

%% ---- Câblage des signaux -----------------------------------------------------
sig = @(a,pa,b,pb) relier(modele,a,pa,b,pb);
sig('Mres','O1','bozV','I1');   sig('bozV','O1','ctrl_regen','I1');
sig('Mreg','O1','bozI','I1');   sig('bozI','O1','ctrl_regen','I2');
sig('MVdc','O1','bozD','I1');   sig('bozD','O1','ctrl_regen','I3');
sig('bozD','O1','relaisHacheur','I1');  sig('relaisHacheur','O1','IntB','I1');
sig('ctrl_regen','O4','rIc','I1');  sig('rIc','O1','gI_conv','I1');  sig('gI_conv','O1','Iconv','I1');
sig('Pvar','O1','divI','I1');   sig('MVdc','O1','satV','I1');
sig('satV','O1','divI','I2');   sig('divI','O1','gI_var','I1');
sig('gI_var','O1','rIvar','I1');  sig('rIvar','O1','Ivar','I1');
% sorties O1..O3 de ctrl_regen -> sources de tension commandées
sig('ctrl_regen','O1','rVa','I1');  sig('rVa','O1','Vca','I1');
sig('ctrl_regen','O2','rVb','I1');  sig('rVb','O1','Vcb','I1');
sig('ctrl_regen','O3','rVc','I1');  sig('rVc','O1','Vcc','I1');

%% ---- Solveur -----------------------------------------------------------------
set_param(modele,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','Ts_power','StopTime','25');
save_system(modele);
fprintf('Modèle %s construit et enregistré.\n', modele);

%% ======================= fonctions locales d'aide ==========================
function ajouter_bloc(modele, bibli, nom, position, varargin)
    add_block(bibli, [modele '/' nom], 'Position', position, varargin{:});
end

function enreg_signal(modele, variable, bloc, port, position)
    nom = ['enreg_' variable];
    add_block('simulink/Sinks/To Workspace', [modele '/' nom], 'Position', position, ...
        'VariableName', variable, 'SaveFormat', 'Timeseries');
    relier(modele, bloc, port, nom, 'I1');
end

function relier(modele, a, pa, b, pb)
% Relie le port pa du bloc a au port pb du bloc b.
% Codes de port : Ln/Rn = port de connexion n à gauche/droite, Tn = n-ième borne électrique
% (ports de gauche d'abord), In/On = entrée/sortie de signal Simulink.
    try
        add_line(modele, obtenir_port(modele,a,pa), obtenir_port(modele,b,pb), 'autorouting','on');
    catch ErrM
        warning('construction:cablage','Impossible de relier %s/%s -> %s/%s : %s  (à câbler manuellement)', ...
                a, pa, b, pb, ErrM.message);
    end
end

function h = obtenir_port(modele, bloc, p)
    ports = get_param([modele '/' bloc], 'PortHandles');
    n  = str2double(p(2:end));
    switch p(1)
        case 'L', ensemble = ports.LConn;
        case 'R', ensemble = ports.RConn;
        case 'T', ensemble = [ports.LConn ports.RConn];
        case 'I', ensemble = ports.Inport;
        case 'O', ensemble = ports.Outport;
    end
    if n > numel(ensemble)
        error('port %s introuvable sur %s (le bloc en possède %d de ce type)', p, bloc, numel(ensemble));
    end
    h = ensemble(n);
end
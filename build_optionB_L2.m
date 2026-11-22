%% build_optionB_L2.m
% Construit le modèle Simulink de l'ARCHITECTURE C / OPTION B au niveau L2 :
%   réseau -> Lg,Rg -> PCC -+-> L_acd -> pont à diodes -> bus DC (variateur existant)
%                           +-> Lf -> convertisseur de régénération moyen -> bus DC
%   Bus DC : Cdc, source de courant du variateur (table P(t)), hacheur de secours.
% Nécessite : Simulink + Simscape Electrical (Specialized Power Systems, 'powerlib').
% Exécuter d'abord params_elevator.m et créer Pdrive_tbl (gen_Pt_table) -- ou utiliser run_optionB_L2.m.

if ~exist('par','var'), run('params_elevator.m'); end

% --- Commutateurs de polarité : à vérifier une fois (voir le texte d'accompagnement), inverser si besoin
SIGN_VSRC = +1;   % polarité des bornes de la source de tension du convertisseur
SIGN_ISRC = +1;   % sens de la source de courant commandée (+1 = prélève de DC+ vers DC-)

mdl = 'ElevatorRegen_OptB_L2';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);  open_system(mdl);
load_system('powerlib');

P = @(c,r,w,h) [40+c*190, 40+r*110, 40+c*190+w, 40+r*110+h];   % aide au placement des blocs

%% ---- Blocs : côté alternatif -------------------------------------------------
add(mdl,'powerlib/powergui','powergui',[20 5 110 40],'SimulationMode','Discrete','SampleTime','Ts_power');
add(mdl,'powerlib/Electrical Sources/Three-Phase Source','Grid',P(0,1,100,90), ...
    'Voltage','V_LL','Frequency','f_g','PhaseAngle','0','InternalConnection','Yg');
add(mdl,'powerlib/Elements/Three-Phase Series RLC Branch','Zg',P(1,1,100,90), ...
    'BranchType','RL','Resistance','Rg','Inductance','Lg');
add(mdl,'powerlib/Measurements/Three-Phase V-I Measurement','Mgrid',P(2,1,100,90), ...
    'VoltageMeasurement','phase-to-ground','CurrentMeasurement','yes');
% branche à diodes (variateur existant)
add(mdl,'powerlib/Elements/Three-Phase Series RLC Branch','Zd',P(3,0,100,90), ...
    'BranchType','RL','Resistance','R_acd','Inductance','L_acd');
add(mdl,'powerlib/Power Electronics/Universal Bridge','Dbr',P(4,0,100,110), ...
    'Arms','3','Device','Diodes','Ron','1e-3','ForwardVoltage','0.8', ...
    'SnubberResistance','1e5','SnubberCapacitance','inf','Measurements','None');
% branche de régénération
add(mdl,'powerlib/Measurements/Three-Phase V-I Measurement','Mreg',P(3,3,100,90), ...
    'VoltageMeasurement','no','CurrentMeasurement','yes');
add(mdl,'powerlib/Elements/Three-Phase Series RLC Branch','Lf',P(4,3,100,90), ...
    'BranchType','RL','Resistance','R_f','Inductance','L_f');
phs = 'abc';
for k = 1:3
    add(mdl,'powerlib/Electrical Sources/Controlled Voltage Source',['Vc' phs(k)], ...
        P(5,2+k,70,50));
end

%% ---- Blocs : côté continu ----------------------------------------------------
add(mdl,'powerlib/Elements/Series RLC Branch','Cdc',P(6,0,70,50),'BranchType','C','Capacitance','Cdc');
try
    set_param([mdl '/Cdc'],'SetiL0','on','InitialVoltage','Vdc0');
catch
    warning('Impossible de fixer la tension initiale de Cdc ; la régler dans la boîte de dialogue du bloc (Vdc0).');
end
add(mdl,'powerlib/Electrical Sources/Controlled Current Source','Idrv',P(6,1,70,50));
add(mdl,'powerlib/Electrical Sources/Controlled Current Source','Iconv',P(6,2,70,50));
add(mdl,'powerlib/Power Electronics/Ideal Switch','SwB',P(6,3,70,50),'Ron','1e-3','Rs','1e6','Cs','inf');
add(mdl,'powerlib/Elements/Series RLC Branch','RbB',P(7,3,70,50),'BranchType','R','Resistance','Rb_B');
add(mdl,'powerlib/Measurements/Voltage Measurement','VdcM',P(7,1,70,50));

%% ---- Blocs : signaux et commande --------------------------------------------
add(mdl,'simulink/Sources/From Workspace','Pdrive',P(8,0,90,40),'VariableName','Pdrive_tbl', ...
    'Interpolate','on','OutputAfterFinalValue','Setting to zero');
add(mdl,'simulink/Discontinuities/Saturation','satV',P(8,1,60,30),'UpperLimit','2000','LowerLimit','100');
add(mdl,'simulink/Math Operations/Divide','divI',P(9,0,50,40));
add(mdl,'simulink/Math Operations/Gain','gI_drv',P(10,0,50,30),'Gain','SIGN_ISRC');
add(mdl,'simulink/Math Operations/Gain','gI_conv',P(10,2,50,30),'Gain','SIGN_ISRC');
add(mdl,'simulink/Discrete/Zero-Order Hold','zohV',P(8,3,50,30),'SampleTime','Ts_ctrl');
add(mdl,'simulink/Discrete/Zero-Order Hold','zohI',P(8,4,50,30),'SampleTime','Ts_ctrl');
add(mdl,'simulink/Discrete/Zero-Order Hold','zohD',P(8,5,50,30),'SampleTime','Ts_ctrl');
add(mdl,'simulink/User-Defined Functions/MATLAB Function','regen_ctrl',P(9,3,120,110));
% Les retards unitaires brisent les boucles algébriques (les sources commandées sont à action
% directe sur les mesures) et modélisent le retard de calcul d'un échantillon supposé dans Td = 1,5*Ts_ctrl.
dly = 'abc';
for k = 1:3
    add(mdl,'simulink/Discrete/Unit Delay',['dV' dly(k)],P(10,2+k,40,25),'SampleTime','Ts_ctrl');
end
add(mdl,'simulink/Discrete/Unit Delay','dIc',  P(11,2,40,25),'SampleTime','Ts_ctrl');
add(mdl,'simulink/Discrete/Unit Delay','dIdrv',P(11,0,40,25),'SampleTime','Ts_power');
add(mdl,'simulink/Discontinuities/Relay','chopRelay',P(9,5,60,30), ...
    'OnSwitchValue','Vchop_on','OffSwitchValue','Vchop_off','OnOutputValue','1','OffOutputValue','0');

%% ---- Loi de commande (intégrée sous forme de texte ; constantes issues de l'espace de travail)
% NB : les commentaires du code embarqué restent en ASCII (sans accents) par prudence d'encodage.
code = [
"function [vca, vcb, vcc, idc, dbg] = regen_ctrl(vabc, iabc, vdc)"
"% Commande de regeneration seule, option B : PLL SRF, boucle de tension DC unidirectionnelle, boucle de courant dq."
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
"dbg = [id; iq; idref; w/(2*pi); vd];"
"th = th_n;"
];
code = strjoin(code, newline);
tok = {'@Ts@',Ts_ctrl; '@w0@',w_g; '@Vpk@',V_ph_pk; '@Lf@',L_f; '@SGN@',SIGN_VSRC; ...
       '@kpi@',kp_i; '@kii@',ki_i; '@kpv@',kp_v; '@kiv@',ki_v; '@kpp@',kp_pll; ...
       '@kip@',ki_pll; '@Vreg@',Vreg_B; '@Idmax@',Id_max};
for n = 1:size(tok,1)
    code = strrep(code, tok{n,1}, num2str(tok{n,2},'%.9g'));
end
rt = sfroot;
ch = rt.find('-isa','Stateflow.EMChart','Path',[mdl '/regen_ctrl']);
ch.Script = char(code);
drawnow;

%% ---- Blocs d'enregistrement --------------------------------------------------
logsig(mdl,'Vdc',      'VdcM','O1', P(11,1,70,25));
logsig(mdl,'Pdrive',   'Pdrive','O1',P(11,0,70,25));
logsig(mdl,'Vabc_pcc', 'Mgrid','O1', P(3,5,70,25));
logsig(mdl,'Iabc_grid','Mgrid','O2', P(4,5,70,25));
logsig(mdl,'Iabc_reg', 'Mreg','O1',  P(5,6,70,25));
logsig(mdl,'idc_conv', 'regen_ctrl','O4', P(11,3,70,25));
logsig(mdl,'dbg',      'regen_ctrl','O5', P(11,4,70,25));
logsig(mdl,'chop_gate','chopRelay','O1',  P(11,5,70,25));

%% ---- Câblage de puissance : côté alternatif (ports triphasés L1..3 / R1..3) ----
for k = 1:3
    s = num2str(k);
    c(mdl,'Grid',['R' s],  'Zg',  ['L' s]);
    c(mdl,'Zg',  ['R' s],  'Mgrid',['L' s]);
    c(mdl,'Mgrid',['R' s], 'Zd',   ['L' s]);      % branche PCC 1 : chemin des diodes
    c(mdl,'Mgrid',['R' s], 'Mreg', ['L' s]);      % branche PCC 2 : chemin de régénération
    c(mdl,'Zd',  ['R' s],  'Dbr',  ['L' s]);
    c(mdl,'Mreg',['R' s],  'Lf',   ['L' s]);
    c(mdl,'Lf',  ['R' s],  ['Vc' phs(k)], 'T2');  % borne de phase du convertisseur
end
c(mdl,'Vca','T1','Vcb','T1');  c(mdl,'Vca','T1','Vcc','T1');   % neutre flottant

%% ---- Câblage de puissance : bus DC (DC+ = Dbr/R1, DC- = Dbr/R2) --------------
for nm = {'Cdc','Idrv','Iconv','SwB','VdcM'}
    c(mdl,'Dbr','R1', nm{1},'T1');                % DC+
end
for nm = {'Cdc','Idrv','Iconv','RbB','VdcM'}
    c(mdl,'Dbr','R2', nm{1},'T2');                % DC-
end
c(mdl,'SwB','T2','RbB','T1');

%% ---- Câblage des signaux -----------------------------------------------------
s_ = @(a,pa,b,pb) c(mdl,a,pa,b,pb);
s_('Mgrid','O1','zohV','I1');   s_('zohV','O1','regen_ctrl','I1');
s_('Mreg','O1','zohI','I1');    s_('zohI','O1','regen_ctrl','I2');
s_('VdcM','O1','zohD','I1');    s_('zohD','O1','regen_ctrl','I3');
s_('zohD','O1','chopRelay','I1');  s_('chopRelay','O1','SwB','I1');
s_('regen_ctrl','O4','dIc','I1');  s_('dIc','O1','gI_conv','I1');  s_('gI_conv','O1','Iconv','I1');
s_('Pdrive','O1','divI','I1');  s_('VdcM','O1','satV','I1');
s_('satV','O1','divI','I2');    s_('divI','O1','gI_drv','I1');
s_('gI_drv','O1','dIdrv','I1');  s_('dIdrv','O1','Idrv','I1');
% sorties O1..O3 de regen_ctrl -> sources de tension commandées
s_('regen_ctrl','O1','dVa','I1');  s_('dVa','O1','Vca','I1');
s_('regen_ctrl','O2','dVb','I1');  s_('dVb','O1','Vcb','I1');
s_('regen_ctrl','O3','dVc','I1');  s_('dVc','O1','Vcc','I1');

%% ---- Solveur -----------------------------------------------------------------
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','Ts_power','StopTime','25');
save_system(mdl);
fprintf('Modèle %s construit et enregistré.\n', mdl);

%% ======================= fonctions locales d'aide ==========================
function add(mdl, lib, name, position, varargin)
    add_block(lib, [mdl '/' name], 'Position', position, varargin{:});
end

function logsig(mdl, var, blk, port, position)
    nm = ['log_' var];
    add_block('simulink/Sinks/To Workspace', [mdl '/' nm], 'Position', position, ...
        'VariableName', var, 'SaveFormat', 'Timeseries');
    c(mdl, blk, port, nm, 'I1');
end

function c(mdl, a, pa, b, pb)
% Relie le port pa du bloc a au port pb du bloc b.
% Codes de port : Ln/Rn = port de connexion n à gauche/droite, Tn = n-ième borne électrique
% (ports de gauche d'abord), In/On = entrée/sortie de signal Simulink.
    try
        add_line(mdl, getport(mdl,a,pa), getport(mdl,b,pb), 'autorouting','on');
    catch ME
        warning('build:wire','Impossible de relier %s/%s -> %s/%s : %s  (à câbler manuellement)', ...
                a, pa, b, pb, ME.message);
    end
end

function h = getport(mdl, blk, p)
    ph = get_param([mdl '/' blk], 'PortHandles');
    n  = str2double(p(2:end));
    switch p(1)
        case 'L', pool = ph.LConn;
        case 'R', pool = ph.RConn;
        case 'T', pool = [ph.LConn ph.RConn];
        case 'I', pool = ph.Inport;
        case 'O', pool = ph.Outport;
    end
    if n > numel(pool)
        error('port %s introuvable sur %s (le bloc en possède %d de ce type)', p, blk, numel(pool));
    end
    h = pool(n);
end

%% params_elevator.m  --  script de paramètres unique et partagé (Cahier des charges v0.1, §6.1)
% À exécuter avant tout modèle. Les éléments marqués PRELIM sont les valeurs préliminaires du cahier des charges.

%% ---- 3.1 Ascenseur de référence --------------------------------------------
g         = 9.81;
Q         = 1000;            % charge nominale [kg]
v_r       = 1.6;             % vitesse nominale [m/s]
m_c       = 1200;            % masse de la cabine vide [kg]        PRELIM
k_cw      = 0.45;            % facteur d'équilibrage du contrepoids PRELIM
m_cw      = m_c + k_cw*Q;    % 1650 kg
H_travel  = 30;              % hauteur de course [m]                PRELIM
a_max     = 0.8;             % [m/s^2]                              PRELIM
jerk_max  = 1.0;             % [m/s^3]                              PRELIM
eta_drive = 0.88;            % rendement global machine+onduleur    PRELIM
m_tot_full = m_c + Q + m_cw; % 3850 kg

par = struct('g',g,'Q',Q,'v_r',v_r,'m_c',m_c,'k_cw',k_cw,'m_cw',m_cw, ...
             'a_max',a_max,'jerk_max',jerk_max,'eta_drive',eta_drive);

% Puissances de crête, éq. (2)-(4)
P_mech_peak     = ((1-k_cw)*Q*g + m_tot_full*a_max)*v_r;   % ~13,6 kW
P_DC_regen_peak = P_mech_peak*eta_drive;                   % ~11,9 kW
P_DC_mot_peak   = P_mech_peak/eta_drive;                   % ~15,4 kW

%% ---- Réseau (A3, GR-01, GR-06/07) -------------------------------------------
V_LL   = 400;  f_g = 50;  w_g = 2*pi*f_g;
V_ph_pk = V_LL*sqrt(2/3);    % crête de la tension simple [V]
SCR    = 100;                % 100 = nominal (GR-06), 20 = réseau faible (GR-07)
XR     = 3;                  % rapport X/R de l'impédance réseau  [valeur provisoire du cahier]

%% ---- Calibres de l'option B et grandeurs de base en p.u. (§3.4, 7.3) --------
S_B = 12e3;                          % puissance apparente nominale [VA]
I_B = S_B/(sqrt(3)*V_LL);            % ~17,3 A
Z_B = V_LL^2/S_B;                    % ~13,3 ohm
L_B = Z_B/w_g;   C_B = 1/(w_g*Z_B);

% Impédance réseau déduite du SCR, supposée sur la base du convertisseur (à préciser dans le cahier)
Zg = Z_B/SCR;  Rg = Zg/sqrt(1+XR^2);  Lg = XR*Rg/w_g;

%% ---- Côté redresseur à diodes (variateur existant) --------------------------
Vdc_diode_nom = 1.35*V_LL;           % ~540 V
Vdc_diode_max = 1.35*V_LL*1.10;      % ~594 V à +10 %
L_acd = 0.5e-3;  R_acd = 5e-3;       % inductance de ligne côté variateur      PRELIM
Vdc0  = Vdc_diode_nom;               % tension initiale du bus DC (précharge hors périmètre)

%% ---- Seuils du bus DC, option B (DR-02, DR-03) ------------------------------
Vreg_B   = 650;                      % seuil de régulation de la régénération   PRELIM
Vtrip    = 800;                      % seuil de surtension du variateur (A6)
Vchop_on = 760;  Vchop_off = 740;    % hacheur de freinage de secours (filet de sécurité)
Rb_B     = Vchop_on^2/P_DC_regen_peak;
assert(Vdc_diode_max < Vreg_B && Vreg_B < Vchop_on && Vchop_on < Vtrip, ...
       'Ordre des seuils DR-02 non respecté');

%% ---- Cadences de commande et filtre (L2 = inductance totale idéale) ---------
fsw      = 10e3;
Ts_ctrl  = 1/fsw;                    % 100 us
Ts_power = Ts_ctrl/2;                % 50 us (modèle moyen, sans commutation)
I_pk_B   = sqrt(2)*I_B;
dI_max   = 0.20*I_pk_B;              % FL-01 : ondulation de 10 à 25 %
L1  = Vreg_B/(8*fsw*dI_max);         % inductance côté convertisseur (première estimation)
L2  = 1.0e-3;                        % inductance côté réseau, valeur provisoire (Tâche 3)
L_f = L1 + L2;                       % filtre L idéal au niveau L2
R_f = 0.005*Z_B;                     % résistance série de l'inductance (0,5 % p.u.)

%% ---- Condensateur du bus DC d'après l'éq. (5) -------------------------------
fv_bw   = 30;                                  % bande passante de la boucle de tension DC [Hz]
dt_resp = 1/(2*pi*fv_bw);
Vhi = 1.05*Vreg_B;  Vlo = 0.95*Vreg_B;         % DR-04 : +/-5 %
Cdc_min = 2*P_DC_regen_peak*dt_resp/(Vhi^2 - Vlo^2);
Cdc     = 2.2e-3;                              % choisi >= Cdc_min
assert(Cdc >= Cdc_min, 'Cdc inférieur au minimum de l''éq. (5)');

%% ---- Gains des régulateurs (règles classiques, SM-02) -----------------------
zeta   = 1/sqrt(2);
Td     = 1.5*Ts_ctrl;                          % retard MLI + calcul
kp_i   = L_f/(2*Td);                           % optimum modulaire
ki_i   = kp_i*R_f/L_f;                         % Ti = L/R
f_pll  = 50;  wn_pll = 2*pi*f_pll;             % PLL synchrone (SRF), normalisée par Vpk
kp_pll = 2*zeta*wn_pll;  ki_pll = wn_pll^2;
wv     = 2*pi*fv_bw;
K_dc   = 1.5*V_ph_pk/(Vreg_B*Cdc);             % gain id -> dVdc/dt
kp_v   = 2*zeta*wv/K_dc;  ki_v = wv^2/K_dc;
Id_max = 1.10*P_DC_regen_peak/(1.5*V_ph_pk);   % limite de courant incluant la marge PR-05

%% ---- Rapport -----------------------------------------------------------------
fprintf('Puissance DC de régénération maximale %.2f kW | Cdc_min %.2f mF (choisi %.2f mF)\n', ...
        P_DC_regen_peak/1e3, Cdc_min*1e3, Cdc*1e3);
fprintf('L1 %.2f mH, L_f %.2f mH = %.3f pu (FL-06 <= 0,10) | Lg %.3f mH\n', ...
        L1*1e3, L_f*1e3, L_f/L_B, Lg*1e3);
fprintf('kp_i %.2f, ki_i %.1f | kp_v %.3f, ki_v %.1f | Id_max %.1f A\n', ...
        kp_i, ki_i, kp_v, ki_v, Id_max);

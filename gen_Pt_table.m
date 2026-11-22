function [tbl, info] = gen_Pt_table(lambda, dirSign, H, par, dt, t_pre, t_post)
%GEN_PT_TABLE  Table de puissance P(t) au bus DC pour un trajet (Cahier des charges §3, FR-05, PR-03).
%   lambda  : fraction de charge (0..1)      dirSign : +1 montée, -1 descente
%   H       : hauteur de course [m]          par     : structure issue de params_elevator
%   tbl     : [t, P_dc]  avec P_dc > 0 absorbée par le variateur (moteur), < 0 restituée (régénération)
if nargin < 5, dt = 1e-3; end
if nargin < 6, t_pre = 1.0; end
if nargin < 7, t_post = 2.0; end

prof  = scurve_profile(H, par.v_r, par.a_max, par.jerk_max, dt);
m_tot = par.m_c + lambda*par.Q + par.m_cw;
v = dirSign*prof.v;   a = dirSign*prof.a;                 % signés : + = montée

P_mech = ((lambda - par.k_cw)*par.Q*par.g + m_tot*a) .* v;   % éq. (1)+(2), sans frottements
P_dc   = P_mech;
mot = P_mech > 0;
P_dc(mot)  = P_mech(mot)  / par.eta_drive;                % moteur : réseau -> machine
P_dc(~mot) = P_mech(~mot) * par.eta_drive;                % régénération : machine -> bus DC

t   = [0; t_pre + prof.t; t_pre + prof.t(end) + t_post];
P   = [0; P_dc; 0];
tbl = [t, P];

info.T_trip       = prof.t(end);
info.v_peak       = max(prof.v);
info.E_mot_kWh    =  trapz(t, max(P,0))/3.6e6;
info.E_regen_kWh  = -trapz(t, min(P,0))/3.6e6;
info.P_regen_peak = -min(P);
info.P_mot_peak   =  max(P);
end

function prof = scurve_profile(H, vmax, amax, jmax, dt)
% Profil en S à 7 phases ; repli sur une vitesse de crête réduite pour les trajets courts.
t_acc   = @(vp) (vp >= amax^2/jmax).*(vp/amax + amax/jmax) ...
              + (vp <  amax^2/jmax).*(2*sqrt(vp/jmax));
s_ramps = @(vp) vp.*t_acc(vp);                 % distance accélération + décélération
if s_ramps(vmax) <= H
    vp = vmax;
else
    vp = fzero(@(x) s_ramps(x) - H, [1e-3 vmax]);
end
ap = min(amax, sqrt(vp*jmax));
tj = ap/jmax;
ta = max(vp/ap - tj, 0);
tc = max((H - vp*(2*tj + ta))/vp, 0);

edges = cumsum([0 tj ta tj tc tj ta tj]);
jv    = jmax*[1 0 -1 0 -1 0 1];
t     = (0:dt:edges(end)).';
seg   = 1 + sum(t >= edges(2:7), 2);           % indice de segment 1..7
jerk  = reshape(jv(seg), [], 1);

prof.t = t;
prof.a = cumtrapz(t, jerk);
prof.v = max(cumtrapz(t, prof.a), 0);
end

clear, clc

%% Paramètres du systèmes

mb = 0.7;
mw = 0.1;
Ix = 0.001;
Iz = 0.001;
Iwx = 0.001;
Iwy = 0.001;
Iwz = 0.001;
l = 0.22;
d = 0.2;
R = 2.05;
k = 0.258;
rho = 0.04;

ubar = 1;
psibar = 0;

% Ecritures simplifiées

M = mb + 2*mw;
Ipsi = Ix + Iz + Iwx + Iwz + (mw*l^2)/2 + (Iwy*l^2);
alpha = M + (2*Iwy)/rho^2;
beta = Ipsi +mb*d^2;
mu = (l*k)/(2*beta*rho*R);
lambda = k/(alpha*rho*R);
mu2 = (mb*d)/beta;


%% Paramètres à l'équilibre

ybar = 0;
ubar = 1;
psibar = 0;
qbar = 0;
% A t=0
pbar = 0;
xbar = 0;

%% Sous-système vitesse (entrée UPlus)

% On utilise Xvitesse=[p u]
Avitesse = [0 1;
    0 (-2*k*lambda)/rho];
Bvitesse = [0;lambda];
% On utilise Yvitesse = [PhiPlus]
Cvitesse = [2/rho 0];
Dvitesse = 0;

sysVitesse = ss(Avitesse, Bvitesse, Cvitesse, Dvitesse);
set(sysVitesse,'InputName',{'UPlus'},'OutputName',{'PhiPlus'})

%figure(1), step(sysVitesse), grid

%% Sous-système rotation (entrée UMoins)

% On utilise Xrot = [y psi q]
Arot = [0 ubar 0;
    0 0 1;
    0 0 (-k*l*mu)/rho + (mb*d*beta*ubar)/beta];
Brot = [0;0;lambda];
% On utilise Yrot = [ym PhiMoins]
Crot = [1 0 0;
    0 l/rho 0];
Drot = [0;0];

sysRot = ss(Arot, Brot, Crot, Drot);
set(sysRot,'InputName',{'UMoins'},'OutputName',{'ym','PhiMoins'})

%figure(2), step(sysRot), grid

%% Analyse des valeurs propres

sort(eig(Avitesse))
sort(eig(Arot))


%% Correcteur PD du sous-système vitesse (dynamique de u déjà rapide)

kpvitesse = 6.01382806472515;
kdvitesse = 17.8115189243961; 

% PD approximé ([Phir PhiPlus])

Tdf = kdvitesse/kpvitesse;
n = 814.157747769439;

M = -n/Tdf;
N = [n/Tdf -n/Tdf];
P = -n*kpvitesse;
Q = [kpvitesse*(n+1) -kpvitesse*(n+1)];

Correcteur = ss(M,N,P,Q);
set(Correcteur,'InputName',{'PhiR','PhiPlus'},'OutputName',{'UPlus'})

figure(3), bode(Correcteur), grid

% FTBO

FTBO = tf((lambda*kpvitesse*rho/2)*[Tdf*(n+1)/n 1], [Tdf/n (((Tdf*2*lambda*k)/rho*n) + 1) 2*lambda*k/rho 0]);
FTBO
figure(4), margin(FTBO), grid

% FTBF

FTBF = feedback(FTBO,tf(1));
FTBF
figure(5), bode(FTBF), grid

% Pôles

pole(FTBF)

%% Correcteur PD du sous-système rotation

k1 = 11238.494291118;
k2 = 783.815251959048;
k3 = 1;
k4 = 1;

T1 = k2/k1;
T2 = k4/k3;
n = 7647.53330101884;

omega = ((k*mu*l)/rho) - mu2*ubar;

M = [-n/T1 0;
    0 -n/T2];
N = M;
P = [-n*k1 -n*k3];
Q = [-k1*(n+1) -k3*(n+1)];

Correcteur = ss(M,N,P,Q);
set(Correcteur,'InputName',{'PhiMoins','ym'},'OutputName',{'UMoins'})

figure(6), bode(Correcteur), grid

% FTBO
FTBO = tf((mu*l*k1/rho)*[T1*(n+1)/n 1], [T1/n (1 + T1*omega/n) omega 0]);
FTBO
figure(7), margin(FTBO), grid

% FTBF
FTBF = feedback(FTBO,tf(1));
FTBF
figure(8), bode(FTBF), grid


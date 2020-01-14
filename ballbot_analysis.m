%%%
%  Ballbot robust stability analysis and validation script
%
%   Contents:
%       - Nonlinear model synthesis with distrubances
%       - Parametric linearisation       
%       - Parametric uncertainty introduction
%           - optimisation based worst case search
%       - Disturbance rejection and noise attenutation
%       - h-infinity controller synthesis
%            - based on linear systems
%            - based on worst case system
%            
%       - comparison of behavior with nonlinear model
%
%
%
%
%  Created January 2020 by Antun Skuric
%%%
disp('Starting full nonlinear Ballbot model derivation');
disp('This will take few minutes');

% %clear all
%% Model physical parameters definition
disp('Model physical parameters with uncertainties');
global rK_n rW_n  rA_n  l_n  mAW_n mK_n  A_ThetaAWx_n  A_ThetaAWy_n  A_ThetaAWz_n  ThetaKi_n  ThetaWi_n;

% Uncertain parameters
rK_n = 0.120; % radius of the ball
rW_n = 0.05; % radius of omni wheel=5cm
rA_n = 0.126; % radius of the body  
l_n = 0.22634; % distance between centre of ball and centre of gravity of the body 
mAW_n = 6.71; % mass of the body and omni wheel  
mK_n = 0.625; % mass of the basket ball  
A_ThetaAWx_n = 1.41271413; % Inertia of the body and Omni wheels in the body reference frame A  
A_ThetaAWy_n = 1.41271311;
A_ThetaAWz_n = 0.05359646;
ThetaKi_n = 0.003606375;
ThetaWi_n = 0.01504; 

% Angle of motors
alpha = 47*pi/180;
beta1 = 0;
beta2 = 2/3*pi;
beta3 = 4/3*pi;

% vector of nominal parameters
params_n = [rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n];
% 
%% Derivation script of the nonlinear model 
model_script_disturbance;
 
%% Linear nominal model sysnthesys
disp('Nominal model calculation');

% A matrix
tic
A_n= fA(rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n);
fprintf('A matrix :  %f sec\n',toc);
% B matrix
tic
B_n= fB(rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n);
fprintf('B matrix :  %f sec\n',toc);
% B_w matrix
tic
B_w= fW(rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n);
fprintf('B_w matrix :  %f sec\n',toc);

% state space model from matrices
G_n = ss(A_n, B_n, eye(10), zeros(10,3));

% LQR controller design
Q = diag([100 50 100 50 40 20 20 10 20 10]);
R = diag([100 100 100]);
K_lqr = lqr(G_n,Q,R);

%% linear and nonlinear model initial condition response comparison
disp('Compare linear and nonlinear nominal model');
X0 = [pi/20 0 pi/20 0 pi/10 0 0 0 0 0]';
T_max = 4; %s
initialplot_compare_nlin(@nlin_model_n,G_n,K_lqr,X0,T_max,1,2)


%% Introducing the uncertainties
disp('Introducing physical parameter uncertainties');

% Uncertain variation of the parameters
A_ThetaAWx_var = 15; % percent
l_var = 15; % percent
A_ThetaAWz_var = 15;% percent
ThetaKi_var = 15;% percent
A_ThetaAWy_var = 15; % percent
ThetaWi_var = 15;% percent

% sample the uncertain models
visualize_uncertainty_samples

%% Find the worst case system based on parameter uncertainties
disp('Find the worst case parameters (minimal stability margin)');
wcu = find_wcu_optimisation(l_var, ThetaWi_var, ThetaKi_var, A_ThetaAWx_var, A_ThetaAWy_var, A_ThetaAWz_var, K_lqr)
% variable wcu contains 
% wcu.l,ThetaKi,... - the worst case parameters 
% wcu.min_margin    - worst case margin
% wcu.max_gain      - worst case gain
% wcu.G             - linear state space model of the worst case transfer function

%% Nominal and worst case linear model comparison
disp('Nominal and worst case linear model comparison')
disp('Initial condition response');

% initialplot
X0 = [pi/20 0 pi/20 0 pi/20 0 0 0 0 0]';
T_sim = 3; %sec
figure(110);
initialplot(feedback(wcu.G,K_lqr),'b',X0,T_sim); 
hold on;
initialplot(feedback(G_n,K_lqr),'.-r',X0,T_sim);
legend('worst case','nominal')
sgtitle('Closed loop (LQR) - Inirtial condition resposnce comparitons nominal and worst case')

%% Singular value plota
disp('Singular value plot');
figure(111);
sigma(feedback(wcu.G,K_lqr),'b');
hold on;
sigma(feedback(G_n,K_lqr),'r');
legend('worst case','nominal')
grid on;
sgtitle('Closed loop (LQR) - Singular value plot comparitons nominal and worst case')
%% Validate worst case performance with the nonlinear model
visualize_nonlinear_worst_case;

%% Distrubance rejection and noise attenuation
% Ballbot has two type of disturbance 
% - force acting on the body - hunam interaction
% - measurement noise 
%   > acceletometer and gyro
%   > influences thetax, d_thetax, thetay, d_thetay  

% Ballbot system
% dx = Ax + [B_n B_w B_u][w n u]'
% y = Cx + [D_n D_w D_u][w n u]'
% u = Kx

% definining the standardized testing input for distrubance rejection
t_d = linspace(0,10,1000);
N_d = length(t_d);
n = randn(4,N_d);
n = n/max(n(:));
wx = [zeros(1,0.2*N_d), ones(1,0.1*N_d), zeros(1,0.7*N_d)];
wy = [zeros(1,0.7*N_d), -ones(1,0.1*N_d) zeros(1,0.2*N_d)];
dist = [n; wx; wy];

figure(200);
for i = 1:4
    subplot(6,1,i);
    plot(t_d,dist(i,:),'LineWidth',1.5);
    hold on
    title(strcat('n',num2str(i)))
end
for i = 1:2
    subplot(6,1,4+i);
    plot(t_d,dist(4+i,:),'LineWidth',1.5);
    hold on
    title(strcat('w',num2str(i)))
end
sgtitle('Testing input signal - measurement noiser and disturbance force on the body')

%% Creating the factorial representaion
% disturbance matrix
B_w = fW(rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n);
% noise matrix
B_n = zeros(10,4);
% measurement noise variance 1 degree 
D_n = 0.5/180*pi*eye(10,4); 
% input D matrix
D_u = zeros(10,5);
C_z = [G_n.c; zeros(3,10)];
D_z = [D_n  D_u ; [zeros(3,6) 30*eye(3)]];
% model for h-infinity synthesis containing z output
Pz = ss(G_n.a, [B_n B_w G_n.b], [C_z; G_n.c], [ D_z; [D_n D_u] ]);
D_y = [D_n  D_u ; [zeros(3,6) eye(3)]];
% model for simulation using y as output
P = ss(G_n.a, [B_n B_w G_n.b], [C_z; G_n.c], [ D_y; [D_n D_u] ]);

% complarison linear nonlinear disturbance rejection
nlsim(@nlin_model_n,K_lqr,dist,t_d,P,201);
sgtitle('Closed loop LQR linear vs nonlinear model with nominal parameters parameters')

% worst case model
% disturbance matrix
B_w_wcu = fW(rK_n, rW_n, rA_n, wcu.l, mAW_n, mK_n, wcu.A_ThetaAWx, wcu.A_ThetaAWy, wcu.A_ThetaAWz, wcu.ThetaKi, wcu.ThetaWi);
% model for h-infinity synthesis containing z output
Pz_wc = ss(wcu.G.a, [B_n B_w_wcu wcu.G.b], [C_z; wcu.G.c], [ D_z; [D_n D_u]]);
% model for simulation using y as output
P_wc = ss(wcu.G.a, [B_n B_w_wcu wcu.G.b], [C_z; wcu.G.c], [ D_y; [D_n D_u]]);

% complarison linear nonlinear disturbance rejection
nlsim(@nlin_model_wcu,K_lqr,dist,t_d,P_wc,202);
sgtitle('Closed loop LQR linear vs nonlinear model with worst case parameters')

%% plot singular values of distrurbance rejection loop
figure(301)
sigma(lft(P,-K_lqr),'r',lft(P_wc,-K_lqr),'b',(0.1:0.01:100));
legend('nominal','worst case');
grid on
title('Singular values plot for distrubance rejection closed loop with LQR')

%% H-infinity controller design fixed
% hinf fixed structure - gain
K_tun = realp('K_t',ones(3,10));   
K_tun.Minimum = -100*ones(3,10); 
K_tun.Maximum = 100*ones(3,10);
[C,gamma,info] = hinfstruct(Pz_wc,-K_tun);
K_hinf = C.Blocks.K_t.Value;

%% plot singular values of distrurbance rejection loop
figure(302)
sigma(lft(P,-K_hinf),'r',lft(P_wc,-K_hinf),'b',lft(P,-K_lqr),'--r',lft(P_wc,-K_lqr),'--b',(0.1:0.01:100));
hold on
sigma(lft(P,K),'y');
legend('nominal H-inf','worst case H-inf','nominal LQR','worst case LQR');
grid on
title('Singular values plot for distrubance rejection closed loop with H-infinity')

%% complarison H-infinity and LQR linear models
figure(303)
lsim(lft(P,-K_hinf),lft(P,-K_lqr),dist,t_d);
sgtitle('H-Infinity and LQR comparison on disturbance rejection nominal linear proces')
figure(304)
lsim(lft(P_wc,-K_hinf),lft(P_wc,-K_lqr),dist,t_d);
sgtitle('H-Infinity and LQR comparison on disturbance rejection worst case linear proces')
%% complarison H-infinity and nonlinear model
nlsim(@nlin_model_wcu,K_hinf,dist,t_d,P_wc,305);
sgtitle('Closed loop H-infinity linear vs nonlinear model with worst case parameters')
nlsim(@nlin_model_n,K_hinf,dist,t_d,P,306);
sgtitle('Closed loop H-infinity linear vs nonlinear model with nominal parameters')

%% H-infinity controller design not fixed
%% hinf controller general
nmeas = 10;
ncont = 3;
opts = hinfsynOptions('Display','on');
[K_hinf_f,CL,gamma] = hinfsyn(Pz,nmeas,ncont,opts);
%% Comparison LQR and H-inifnity full
lsim(lft(P,K),lft(P,-K_lqr),dist,t_d);

%% Bode diagram of LQR, Hinfinity fixed and Hinfinity full
bodemag(lft(P,K_lqr),'r',lft(P,K_hinf),'b',lft(P,K_hinf_f),'k');
legend('lqr','h-inf gain','h-inf full')



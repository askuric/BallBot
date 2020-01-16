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
%  Created January 2020 by Antun Skuric
%%%
disp('Starting full nonlinear Ballbot model derivation');
disp('This will take few minutes');

clear all
mkdir('generated')
addpath('./generated')
addpath('./visualisation')
addpath('./optimisation')
%% Model physical parameters definition
% Introducing the physical parameters of the system
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

%% Derivation script of the nonlinear model
% Modeling ot the nonlinear system - extended matlab version of the script FInal.nb 
% made in Mathematica
% 
% - Parametric nonlinear model creation
% - Linearization around 0
% - Creation of simulation functions for further analyisis - linear and nonlinear
% 
% Execution can take 5-10 minutes
model_generation;

%% Linear nominal model sysnthesys
% Creation of linear transfer funciton based on linearised matrices of the Ballbot 
% and the nominal parameters
% 
% Additionally the LQR controller is provided (based on the LQR controller in 
% the report) as a default controller to be compared in further analysis.

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

%% LQR controller design
Q = diag([100 50 100 50 40 20 20 10 20 10]);
R = diag([100 100 100]);
K_lqr = lqr(G_n,Q,R);

%% Linear and nonlinear model initial condition response comparison
% Finally, simple graph of initial condition response for linear and nonlinear 
% system with nominal parameters to see how well does the lienarized system corespond 
% with the nonlinear

%% linear and nonlinear model initial condition response compariso
disp('Compare linear and nonlinear nominal model');
X0 = [pi/20 0 pi/20 0 pi/10 0 0 0 0 0]';
T_max = 4; %s
initialplot_compare_nlin(@nlin_model_n,G_n,K_lqr,X0,T_max,1,2)
drawnow

%% Introducing the uncertainties
% The uncertainties are introduced into the physiscal parameters of the system. 
% The idea is that for the variables you cannot measure you should have some uncertainty 
% about their values. 
% 
% The uncertainties are set as the percentage.For example if l_var is 20, that 
% means that your height of center of mass can be +-20% different than your nominal 
% value.
disp('Introducing physical parameter uncertainties');
% Uncertain variation of the parameters
A_ThetaAWx_var = 15; % percent
l_var = 10; % percent
A_ThetaAWz_var = 15;% percent
ThetaKi_var = 15;% percent
A_ThetaAWy_var = 15; % percent
ThetaWi_var = 15;% percent

% sample the uncertain models
visualize_uncertainty_samples

%% Find the worst case system based on parameter uncertainties
% To be able to find the worst case parameters we define the optimisation problem 
% based on disk margin. 
% 
% Basically we are changing the parameter values and searching to the minimum 
% of the stability margin (diskmargin)
% 
% The minimal margin corresponds to the worst case parameters
disp('Find the worst case parameters (minimal stability margin)');
wcu = find_wcu_optimisation(l_var, ThetaWi_var, ThetaKi_var, A_ThetaAWx_var, A_ThetaAWy_var, A_ThetaAWz_var, K_lqr)
% variable wcu contains 
% wcu.l,ThetaKi,... - the worst case parameters 
% wcu.min_margin    - worst case margin
% wcu.max_gain      - worst case gain
% wcu.G             - linear state space model of the worst case transfer function

%% Nominal and worst case linear model comparison
disp('Nominal and worst case linear model comparison')
%% Margin comparison
disp('Margin comparison')
% nominal parameters
MMIO_n = diskmargin(G_n,K_lqr)
%worst case parameters
MMIO_wc = diskmargin(wcu.G,K_lqr)

%% Initial condition response
disp('Initial condition response');
% initialplot
X0 = [pi/20 0 pi/20 0 pi/10 0 0 0 0 0]';
T_sim = 3; %sec=
initialplt(feedback(wcu.G,K_lqr),'b',X0,T_sim,110); 
initialplt(feedback(G_n,K_lqr),'r',X0,T_sim,110);
legend('worst case','nominal')
sgtitle('Closed loop (LQR) - Inirtial condition response comparison nominal and worst case')

%% Singular value plota
disp('Singular value plot');
figure(111);
sigma(feedback(wcu.G,K_lqr),'b');
hold on;
sigma(feedback(G_n,K_lqr),'r');
legend('worst case','nominal')
grid on;
sgtitle('Closed loop (LQR) - Singular value plot comparison nominal and worst case')
	   
%% Validate worst case performance with the nonlinear model
% generate the worst  case parameters nonlinear model ans save it to  nlin_model_wcu function
model_generation_worst_case;

% Simulate and compare worst case linear and nonliner
initialplot_compare_nlin(@nlin_model_wcu,wcu.G,K_lqr,X0,T_sim,120,121)
figure(120);
sgtitle('Linearized and nonlinear WC system with LQR controller - states')
figure(121);
sgtitle('Linearized and nonlinear WC system with LQR controller - control signals')

%% Distrubance rejection and noise attenuation
% Ballbot has two type of disturbance 
% - force acting on the body - hunam interaction (w)_
% - measurement noise  (_n_)
% > acceletometer and gyro
% > influences thetax, d_thetax, thetay, d_thetay  
% 
% Ballbot system
% dx = Ax + [B_n B_w B_u]*[w n u]'
% y = Cx + [D_n D_w D_u]*[w n u]'
% u = K*x

% definining the standardized testing input for distrubance rejection
t_d = linspace(0,20,2000);
N_d = length(t_d);
n = randn(4,N_d);
% measurement noise
n = n/max(n(:));
% forces
wx = [zeros(1,0.2*N_d), ones(1,0.1*N_d), zeros(1,0.7*N_d)];
wy = [zeros(1,0.7*N_d), -ones(1,0.1*N_d) zeros(1,0.2*N_d)];
dist = [n; wx; wy];

% plotting the distrubance signal
% measurement noise
figure(200);
state_var_names = {'\vartheta_x', '\vartheta_x`','\vartheta_y', '\vartheta_y`'};
for i = 1:4
    subplot(6,1,i);
    plot(t_d,dist(i,:),'LineWidth',1.5);
    hold on
    title(strcat(state_var_names{i},'(n',num2str(i),')'))
    if mod(i,2)
        ylabel('rad')
    else
        ylabel('rad/s')
    end
end
w_names={'T_x','T_y'};
for i = 1:2
    subplot(6,1,4+i);
    plot(t_d,dist(4+i,:),'LineWidth',1.5);
    hold on
    title(strcat(w_names{i},'(w',num2str(i),')'))
    ylabel('Nm')
end
sgtitle('Testing input signal - measurement noise and disturbance force on the body')

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
D_z = [D_n  D_u ; [zeros(3,6) 5*eye(3)]];
% model for h-infinity synthesis containing z output
Pz = ss(G_n.a, [B_n B_w G_n.b], [C_z; G_n.c], [ D_z; [D_n D_u] ]);
D_y = [D_n  D_u ; [zeros(3,6) eye(3)]];
% model for simulation using y as output
P = ss(G_n.a, [B_n B_w G_n.b], [C_z; G_n.c], [ D_y; [D_n D_u] ]);

% worst case model
% disturbance matrix
B_w_wcu = fW(rK_n, rW_n, rA_n, wcu.l, mAW_n, mK_n, wcu.A_ThetaAWx, wcu.A_ThetaAWy, wcu.A_ThetaAWz, wcu.ThetaKi, wcu.ThetaWi);
% model for h-infinity synthesis containing z output
Pz_wc = ss(wcu.G.a, [B_n B_w_wcu wcu.G.b], [C_z; wcu.G.c], [ D_z; [D_n D_u]]);
% model for simulation using y as output
P_wc = ss(wcu.G.a, [B_n B_w_wcu wcu.G.b], [C_z; wcu.G.c], [ D_y; [D_n D_u]]);
%% Initial LQR disturbance rejection
% Compariosn of linear and nonliner performcane using nominal and worst case 
% parameters
% complarison linear nonlinear disturbance rejection
nlsim(@nlin_model_n,K_lqr,dist,t_d,P,201,202);
figure(201)
sgtitle('LQR linear vs nonlinear model with nominal parameters - states')
figure(202)
sgtitle('LQR linear vs nonlinear model with nominal parameters - control signals')
% complarison linear nonlinear disturbance rejection
nlsim(@nlin_model_wcu,K_lqr,dist,t_d,P_wc,203,204);
figure(203)
sgtitle('LQR linear vs nonlinear model with worst case parameters - states')
figure(204)
sgtitle('LQR linear vs nonlinear model with worst case parameters - control signals')
%% 
% Singular value plot for disturbance rejection fonominal and worst case model
% plot singular values of distrurbance rejection loop
figure(301)
sigma(lft(P,-K_lqr),'r',lft(P_wc,-K_lqr),'b',(0.1:0.01:100));
legend('nominal','worst case');
grid on
title('Singular values plot for distrubance rejection closed loop with LQR')
%% H-infinity controller design
% Fixed structure - gain U=K*x; 
K_tun = realp('K_t',ones(3,10));   
K_tun.Minimum = -100*ones(3,10); 
K_tun.Maximum = 100*ones(3,10);
[C,gamma,info] = hinfstruct(Pz_wc,-K_tun);
K_hinf = C.Blocks.K_t.Value

%% Comparison of H-infinity and LQR 
%% plot singular values of distrurbance rejection loop
figure(302)
sigma(lft(P,-K_hinf),'r',lft(P_wc,-K_hinf),'--r',lft(P,-K_lqr),'b',lft(P_wc,-K_lqr),'--b',(0.1:0.01:100));
legend('nominal H-inf','worst case H-inf','nominal LQR','worst case LQR');
grid on
title('Singular values plot for distrubance rejection closed loop with H-infinity')
%% complarison H-infinity and LQR linear models
linsim(lft(P,-K_hinf),'b',dist,t_d,303,304);
linsim(lft(P,-K_lqr),'r',dist,t_d,303,304);
figure(303)
sgtitle('LQR and H-inifnity disturbance nominal parmas - states')
legend('H-infinity','LQR')
figure(304)
sgtitle('LQR and H-inifnity disturbance nominal parmas - control signal')
legend('H-infinity','LQR')
linsim(lft(P_wc,-K_hinf),'b',dist,t_d,305,306);
linsim(lft(P_wc,-K_lqr),'r',dist,t_d,305,306);
figure(305)
sgtitle('LQR and H-inifnity disturbance worst case params - states')
legend('H-infinity','LQR')
figure(306)
sgtitle('LQR and H-inifnity disturbance worst case  params - control signal')
legend('H-infinity','LQR')

%% Bode diagram of comparison 
H_lqr = lft(P,-K_lqr);
H_hinf = lft(P,-K_hinf);
bodemg(H_lqr(1:10,1),'r',311);
hold on
bodemg(H_hinf(1:10,1),'b',311);
legend('LQR', 'H-inf')
sgtitle('Noise attenuation control loop to the input n_1');

bodemg(H_lqr(1:10,5),'r',312);
hold on
bodemg(H_hinf(1:10,5),'b',312);
legend('LQR', 'H-inf')
sgtitle('Disturbance rejection control loop to the input T_x (w_1)');

%% complarison H-infinity and nonlinear model
nlsim(@nlin_model_wcu,K_hinf,dist,t_d,P_wc,307,308);
figure(307);
sgtitle('H-inifnity nominal linear vs nonlinear - states')
figure(308)
sgtitle('H-inifnity nominal linear vs nonlinear - control signals')
nlsim(@nlin_model_n,K_hinf,dist,t_d,P,309,310);
figure(309)
sgtitle('H-inifnity worst case linear vs nonlinear - states')
figure(310)
sgtitle('H-inifnity worct case linear vs nonlinear - control signals')

%% H-infinity controller design not fixed 
% State space controller				  
nmeas = 10;
ncont = 3;
opts = hinfsynOptions('Display','on','LimitGain','on');
[K_hinf_ss,CL,gamma,info] = hinfsyn(Pz_wc,nmeas,ncont,opts);
K_hinf_ss
%% Comparison LQR and H-inifnity full
linsim(lft(P,-K_lqr),'b',dist,t_d,404,405);
linsim(lft(P,K_hinf_ss),'r',dist,t_d,404,405);
figure(404)
sgtitle('LQR and H-inif (state space) disturbance nominal parmas - states')
legend('H-infinity','LQR')
figure(405)
sgtitle('LQR and H-inif (state space)  disturbance nominal parmas - control signal')
legend('H-inf ss','LQR')
%% Bode diagram of LQR, H-infinity fixed and H-infinity full
% We can see that Full H-infinity controller has lowest gain for almost all 
% the transfrer functions on the plot making it the most robustly stable one.

figure(406)
bodemag(lft(P,-K_lqr),'r',lft(P,-K_hinf),'b',lft(P,K_hinf_ss),'k');
legend('LQR','H-inf gain','F-inf ss');
%% H-infintiy norms for nominal and worst case parameter linear model comparison of LQR, fixed H-inifnity and Full H-Infinity controller
hinfnorm(lft(P,-K_lqr))
hinfnorm(lft(P,-K_hinf))
hinfnorm(lft(P,K_hinf_ss))

hinfnorm(lft(P_wc,-K_lqr))
hinfnorm(lft(P_wc,-K_hinf))
hinfnorm(lft(P_wc,K_hinf_ss))

%% Closed loop singular value plot comparison
% plot singular values of distrurbance rejection loop
figure(407)
sigma(lft(P,-K_lqr),'r',lft(P,-K_hinf),'b',lft(P,-K_hinf_ss),'k',lft(P_wc,-K_lqr),'--r',lft(P_wc,-K_hinf),'--b',lft(P_wc,-K_hinf_ss),'--k',(0.05:0.01:1000));
legend('LQR','H-inf','H-inf ss','LQR (wc)','H-inf (wc)','H-inf ss (wc)');
grid on;

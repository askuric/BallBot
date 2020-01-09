% Model physical parameters
disp('Model physical parameters with uncertainties');

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

params_n = [rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n];


% Uncertain variation of the parameters
A_ThetaAWx_var = 15; % percent
l_var = 10; % percent
A_ThetaAWz_var = 15;% percent
ThetaKi_var = 15;% percent
A_ThetaAWy_var = 15; % percent
ThetaWi_var = 15;% percent

%% nominal model
disp('Nominal model calculation');

% A matrix
tic
A_u= fA(rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n);
fprintf('A matrix :  %f sec\n',toc);
% B matrix
tic
B_u= fB(rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n);
fprintf('B matrix :  %f sec\n',toc);

% state space model from matrices
G_n = ss(A_u,B_u,eye(10),zeros(10,3));

% LQR controller design
Q = diag([1 1 1 1 1 1 1 1 1 1]);
R = diag([1 1 1]);
K = lqr(G_n,Q,R);

% nominal closed loop system
T_n = feedback(G_n,K);

%% Optimisation problem
disp('Constrained optimisation approach to finding worst case condition');
% initial set of weights
par0 = [1 1 1 1 1 1]; 
% bounds
par_lb = 1 - [l_var ThetaWi_var ThetaKi_var A_ThetaAWx_var A_ThetaAWy_var A_ThetaAWz_var]/100; 
par_ub= 1 + [l_var ThetaWi_var ThetaKi_var A_ThetaAWx_var A_ThetaAWy_var A_ThetaAWz_var]/100; 

options = optimoptions('fmincon','Display','iter','MaxIterations',30);
f = @(x)optimisiation_func(x, K , rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n);
[X,FVAL,EXITFLAG] = fmincon(f,par0,[],[],[],[],par_lb,par_ub,[],options)

%% Construct the worst case sample
disp('Construct the worst case sample');

% build the worst case sample
%G_wcu = wcu.G;
wcu.l = X(1)*l_n;
wcu.ThetaWi = X(2)*ThetaWi_n;
wcu.ThetaKi = X(3)*ThetaKi_n;
wcu.A_ThetaAWx = X(4)*A_ThetaAWx_n;
wcu.A_ThetaAWy = X(5)*A_ThetaAWy_n;
wcu.A_ThetaAWz = X(6)*A_ThetaAWz_n;
wcu.min_margin = FVAL;


% A matrix
tic
A_u= fA(rK_n, rW_n, rA_n, wcu.l, mAW_n, mK_n, wcu.A_ThetaAWx, wcu.A_ThetaAWy, wcu.A_ThetaAWz, wcu.ThetaKi, wcu.ThetaWi);
fprintf('A matrix :  %f sec\n',toc);
% B matrix
tic
B_u= fB(rK_n, rW_n, rA_n, wcu.l, mAW_n, mK_n, wcu.A_ThetaAWx, wcu.A_ThetaAWy, wcu.A_ThetaAWz, wcu.ThetaKi, wcu.ThetaWi);
fprintf('B matrix :  %f sec\n',toc);

% state space model from matrices
wcu.G = ss(A_u,B_u,eye(10),zeros(10,3));



%% Creating a matlab function from nonlinear model
disp('Creating a matlab function from nonlinear model');
disp('This may take few minutes');
tic; 

% take symbolic params
syms rK rW rA l mAW mK A_ThetaAWx A_ThetaAWy A_ThetaAWz ThetaKi ThetaWi real;
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 u1 u2 u3 real
load nlin_model n_model;
params_s = [rK rW rA l mAW mK A_ThetaAWx A_ThetaAWy A_ThetaAWz ThetaKi ThetaWi];
% parameter values
params_wcu = [rK_n, rW_n, rA_n, wcu.l, mAW_n, mK_n, wcu.A_ThetaAWx, wcu.A_ThetaAWy, wcu.A_ThetaAWz, wcu.ThetaKi, wcu.ThetaWi];
n_model_wcu = subs(n_model, params_s, params_wcu);
% create a file
disp('Writing function in the file');
nlin_model_wcu = matlabFunction(n_model_wcu,'Optimize',false,'Vars',[x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 u1 u2 u3],'File','nlin_model_wcu');
toc;

%% Simulate and compare
disp('Simulate')
X0 = [pi/10 0 -pi/10 0 pi/10 0 0 0 0 0]';
initialplot_compare_nlin(@nlin_model_wcu,wcu.G,K,X0,4)

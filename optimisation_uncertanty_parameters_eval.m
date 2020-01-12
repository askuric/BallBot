%%%
%  Ballbot uncertainty based on physiscal parameters analysis
%
%   Contents:
%       - create parametric nonlinear model
%           - create m-function with nonlinear model with nominal parameters       
%       - linarise nonlinear model around 0 with physiscal parameters of
%       the system as variables - for uncertainty analysis
%           - create m-functions for matrices A,B_u and B_w 
%
%  Created January 2020 by Antun Skuric
%%%


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

wcu.max_gain = getPeakGain(wcu.G);


%% Creating a matlab function from nonlinear model
disp('Creating a matlab function from nonlinear model');
disp('This may take few minutes');
tic; 

% take symbolic params
syms rK rW rA l mAW mK A_ThetaAWx A_ThetaAWy A_ThetaAWz ThetaKi ThetaWi real;
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 u1 u2 u3 w1 w2 real
load nlin_model n_model;
params_s = [rK rW rA l mAW mK A_ThetaAWx A_ThetaAWy A_ThetaAWz ThetaKi ThetaWi];
% parameter values
params_wcu = [rK_n, rW_n, rA_n, wcu.l, mAW_n, mK_n, wcu.A_ThetaAWx, wcu.A_ThetaAWy, wcu.A_ThetaAWz, wcu.ThetaKi, wcu.ThetaWi];
n_model_wcu = subs(n_model, params_s, params_wcu);
% create a file
disp('Writing function in the file');
nlin_model_wcu = matlabFunction(n_model_wcu,'Optimize',false,'Vars',[x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 u1 u2 u3 w1 w2],'File','nlin_model_wcu');
toc;

%% Simulate and compare worst case linear and nonliner
disp('Simulate and compare worst case linear and nonliner')
X0 = [pi/10 0 -pi/10 0 pi/10 0 0 0 0 0]';
T_sim = 4; %sec
initialplot_compare_nlin(@nlin_model_wcu,wcu.G,K,X0,T_sim)

%% Nominal and worst case linear model comparison
disp('Nominal and worst case linear model comparison')
disp('Initial condition response');

% initialplot
X0 = [pi/10 0 pi/10 0 pi/10 0 0 0 0 0]';
T_sim = 3; %sec
figure(100);
initialplot(feedback(wcu.G,K),'b',X0,T_sim); 
hold on;
initialplot(feedback(G_n,K),'r',X0,T_sim);
legend('worst case','nominal')

% bode
disp('Bode plot');
figure(101);
bodemag(feedback(wcu.G,K),'b');
hold on;
bodemag(feedback(G_n,K),'r');
legend('worst case','nominal')

%% Singular value plot
disp('Singular value plot');
figure(102);
sigma(feedback(wcu.G*K,eye(10)),'b');
hold on;
sigma(feedback(G_n*K,eye(10)),'r');
legend('worst case','nominal')


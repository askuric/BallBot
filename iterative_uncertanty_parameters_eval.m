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
l_var = 15; % percent
A_ThetaAWz_var = 15;% percent
ThetaKi_var = 15;% percent
A_ThetaAWy_var = 15; % percent
ThetaWi_var = 15;% percent

%% nominal model
disp('Nominal model calculation');

A_u = [];
B_u = [];
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

%% Sample uncertain model family
disp('Sample uncertain models');
% Cell array of sampled models
clear G_arr;

% number of samples
N = 500; % for N < 25 the plots are going to be shown 

% random samples of  parameter paramter values  
sample_coef = 1 + diag([l_var ThetaWi_var ThetaKi_var A_ThetaAWx_var A_ThetaAWy_var A_ThetaAWz_var])*( rand(6,N)*2-1)./100; 

% calculate model for all N samples
for iter = 1:N
    fprintf('Iteration: %d/ %d \n',iter,N);
    l_v = l_n*sample_coef(1,iter);
    ThetaWi_v = ThetaWi_n*sample_coef(2,iter);
    ThetaKi_v = ThetaKi_n*sample_coef(3,iter);
    A_ThetaAWx_v = A_ThetaAWx_n*sample_coef(4,iter);
    A_ThetaAWy_v = A_ThetaAWy_n*sample_coef(5,iter);
    A_ThetaAWz_v = A_ThetaAWz_n*sample_coef(6,iter);
    % Enable uncertainty analysis
    disp('Enable uncertainty analysis');
    disp('This might take several minutes');
    tic
    
    A_u = [];
    B_u = [];
    
    % A matrix
    tic
    A_u= fA(rK_n, rW_n, rA_n, l_v, mAW_n, mK_n, A_ThetaAWx_v, A_ThetaAWy_v, A_ThetaAWz_v, ThetaKi_v, ThetaWi_v);
    fprintf('A matrix :  %f sec\n',toc);
    % B matrix
    tic
    B_u= fB(rK_n, rW_n, rA_n, l_v, mAW_n, mK_n, A_ThetaAWx_v, A_ThetaAWy_v, A_ThetaAWz_v, ThetaKi_v, ThetaWi_v);
    fprintf('B matrix :  %f sec\n',toc);
    
    % state space model creation
    G_arr(:,:,iter) = ss(A_u,B_u,eye(10),zeros(10,3));
end

%% Calculate closed loop
disp('Calculate closed loop and check for stability');
tic;
T_arr={};
unstable_params = [];
for iter = 1:N
    T_arr{iter} = feedback(G_arr(:,:,iter),K);
    if ~isstable(T_arr{iter})
        fprintf('%d - unstable\n',iter);   
        unstable_params = [unstable_params, sample_coef(:,iter)];
    end
end
if isempty(unstable_params)
    disp('No unstable samples found');  
end
toc

%% Uncertainty model visualisation initialplot
disp('Uncertain model visualisation - initial condition response');
disp('It may take few minutes');
if N <= 20 % a bit of protection of too much plotting
    tic
    X0 = [pi/10 0 pi/10 0 pi/10 0 0 0 0 0]'
    figure(100);
    t_sim = [0:0.01:4];
    initialplot(T_arr{:},'b',X0,t_sim); 
    hold on;
    initialplot(T_n,'r',X0,t_sim);
    toc
else
    disp('Too mmany samples to draw plot');    
end

%% Uncertainty model visualisation  bode
disp('Uncertain model visualisation - bode plot');
disp('It may take few minutes');
if N <= 20 % a bit of protection of too much plotting
    tic
    figure(101);
    bodemag(T_arr{:},'b');
    hold on;
    bodemag(T_n,'r');
    toc
else
    disp('Too mmany samples to draw plot');
end

%% Calculating stability margin bounds
disp('Calculating stability margin bounds');
tic;
wcu.min_margin=inf;
for i=1:N
    fprintf('%d/%d\n',i,N);
    MMIO = diskmargin(G_arr(:,:,i),K);
    if wcu.min_margin > MMIO.DiskMargin
        wcu.min_margin = MMIO.DiskMargin;
        wcu.index = i;
        wcu.G = G_arr(:,:,i);
    end
end
toc

fprintf('Maximal closed loop gain bounds are: [%f, %f]\n', gain.min,gain.max);
fprintf('Minimal system DiskNorm is: %f  for index %d \n', wcu.min_margin, wcu.index);

% build the worst case sample
G_wcu = wcu.G;
wcu.l = sample_coef(1,wcu.index)*l_n;
wcu.ThetaWi = sample_coef(2,wcu.index)*ThetaWi_n;
wcu.ThetaKi = sample_coef(3,wcu.index)*ThetaKi_n;
wcu.A_ThetaAWx = sample_coef(4,wcu.index)*A_ThetaAWx_n;
wcu.A_ThetaAWy = sample_coef(5,wcu.index)*A_ThetaAWy_n;
wcu.A_ThetaAWz = sample_coef(6,wcu.index)*A_ThetaAWz_n;

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


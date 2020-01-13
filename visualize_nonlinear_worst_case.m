%% Creating a matlab function for nonlinear model and worst case parameters
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
nlin_model_w = matlabFunction(n_model_wcu,'Optimize',false,'Vars',[x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 u1 u2 u3 w1 w2],'File','nlin_model_wcu');
toc;

%% Simulate and compare worst case linear and nonliner
disp('Simulate and compare worst case linear and nonliner')
X0 = [pi/20 0 -pi/20 0 pi/20 0 0 0 0 0]';
T_sim = 4; %sec
initialplot_compare_nlin(@nlin_model_wcu,wcu.G,K_lqr,X0,T_sim,120,121)
figure(120);
sgtitle('Initial condition response - Linearized and nonlinear WC system with LQR controller - states')
figure(121);
sgtitle('Initial condition response - Linearized and nonlinear WC system with LQR controller - control signals')

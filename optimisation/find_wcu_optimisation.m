function wcu = find_wcu_optimisation(l_var, ThetaWi_var, ThetaKi_var, A_ThetaAWx_var, A_ThetaAWy_var, A_ThetaAWz_var, K)

%% Function findig the worst case disk margin stability of the closed loop system (LQR)
%  based on the lineatrised matrices A and B from the files fA and fB and
%  the parameter bounds l_var ThetaWi_var ThetaKi_var A_ThetaAWx_var A_ThetaAWy_var A_ThetaAWz_var

global rK_n rW_n  rA_n  l_n  mAW_n mK_n  A_ThetaAWx_n  A_ThetaAWy_n  A_ThetaAWz_n  ThetaKi_n  ThetaWi_n;

%% Defining the Optimisation problem
disp('Constrained optimisation approach to finding worst case condition');
% bounds
par_lb = 1 - [l_var ThetaWi_var ThetaKi_var A_ThetaAWx_var A_ThetaAWy_var A_ThetaAWz_var]/100; 
par_ub= 1 + [l_var ThetaWi_var ThetaKi_var A_ThetaAWx_var A_ThetaAWy_var A_ThetaAWz_var]/100; 

% initial set of weights
par0 = (par_ub - par_lb).*rand(1,6) + par_lb; 

options = optimoptions('fmincon','Display','iter','MaxIterations',30,'Algorithm','active-set');
f = @(x)optimisiation_func(x, K , rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n);
[X,FVAL,EXITFLAG] = fmincon(f,par0,[],[],[],[],par_lb,par_ub,[],options)

%% Construct the worst case sample
disp('Construct the worst case sample');

% build the worst case sample
wcu.l = X(1)*l_n;
wcu.ThetaWi = X(2)*ThetaWi_n;
wcu.ThetaKi = X(3)*ThetaKi_n;
wcu.A_ThetaAWx = X(4)*A_ThetaAWx_n;
wcu.A_ThetaAWy = X(5)*A_ThetaAWy_n;
wcu.A_ThetaAWz = X(6)*A_ThetaAWz_n;
wcu.min_margin = FVAL;

% A matrix
A_u= fA(rK_n, rW_n, rA_n, wcu.l, mAW_n, mK_n, wcu.A_ThetaAWx, wcu.A_ThetaAWy, wcu.A_ThetaAWz, wcu.ThetaKi, wcu.ThetaWi);
% B matrix
B_u= fB(rK_n, rW_n, rA_n, wcu.l, mAW_n, mK_n, wcu.A_ThetaAWx, wcu.A_ThetaAWy, wcu.A_ThetaAWz, wcu.ThetaKi, wcu.ThetaWi);
% state space model from matrices
wcu.G = ss(A_u,B_u,eye(10),zeros(10,3));
wcu.max_gain = getPeakGain(feedback(wcu.G,K));

end


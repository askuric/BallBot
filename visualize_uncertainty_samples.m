
%% Sample uncertain model family
disp('Sample uncertain models');
% Cell array of sampled models
clear G_arr;
T_arr={};

% number of samples
N = 15; % 
% random samples of  parameter paramter values  
sample_coef =1 + diag([l_var ThetaWi_var ThetaKi_var A_ThetaAWx_var A_ThetaAWy_var A_ThetaAWz_var])*( rand(6,N)*2-1)./100; 

% calculate model for all N samples
for iter = 1:N
    l_v = l_n*sample_coef(1,iter);
    ThetaWi_v = ThetaWi_n*sample_coef(2,iter);
    ThetaKi_v = ThetaKi_n*sample_coef(3,iter);
    A_ThetaAWx_v = A_ThetaAWx_n*sample_coef(4,iter);
    A_ThetaAWy_v = A_ThetaAWy_n*sample_coef(5,iter);
    A_ThetaAWz_v = A_ThetaAWz_n*sample_coef(6,iter);
        
    % A matrix
    A_u= fA(rK_n, rW_n, rA_n, l_v, mAW_n, mK_n, A_ThetaAWx_v, A_ThetaAWy_v, A_ThetaAWz_v, ThetaKi_v, ThetaWi_v);
    % B matrix
    B_u= fB(rK_n, rW_n, rA_n, l_v, mAW_n, mK_n, A_ThetaAWx_v, A_ThetaAWy_v, A_ThetaAWz_v, ThetaKi_v, ThetaWi_v);
    
    % state space model creation
    G_arr(:,:,iter) = ss(A_u,B_u,eye(10),zeros(10,3)); 
    T_arr{iter} = feedback(G_arr(:,:,iter),K_lqr);
    if ~isstable(T_arr{iter})
        fprintf('%d - unstable\n',iter);  
    end
end

% nominal model closed looop
T_n = feedback(G_n,K_lqr);

%% Uncertainty model visualisation initialplot
disp('Uncertain model visualisation - initial condition response');
disp('It may take few minutes');
if N <= 20 % a bit of protection of too much plotting
    tic
    X0 = [pi/20 0 pi/20 0 0 0 0 0 0 0]';
    t_sim = 4;
    for i = 1:N
        initialplt(T_arr{i},'b',X0,t_sim,100); 
    end
    initialplt(T_n,'.-r',X0,t_sim,100);
    toc
else
    disp('Too mmany samples to draw plot');    
end
sgtitle('Closed loop (LQR) - initial condition response with parameter uncertainty')

%% Uncertainty model visualisation  singular value plot
disp('Uncertain model visualisation - singular value plot');
disp('It may take few minutes');
if N <= 20 % a bit of protection of too much plotting
    tic
    figure(101);
    for i = 1:N
        sigma(T_arr{i},'b');
        hold on;
    end
    sigma(T_n,'.-r');
    grid on;
    toc
else
    disp('Too mmany samples to draw plot');
end
sgtitle('Closed loop (LQR) - Singular value plot with parameter uncertainty')

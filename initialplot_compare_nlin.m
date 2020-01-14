function initialplot_compare_nlin(fun_nlin,G_lin,K,X0,T_max,fig1,fig2)
%   Function plotting comparison of initial condiiton (X0) responses of the
%   nonlinear system provided by (fun_nlin) and linear system (G_lin) 
%   by closing the loop with the controller (K)
    
% nonlinear simulation
Ts = 0.01;
t_nlin = [0:Ts:T_max];
X = X0;

x_nlin = zeros(length(t_nlin),10);
u_nlin = zeros(length(t_nlin),3);
disp('nonlinear system simulation');
tic;

fprintf('Nonlinear simulation: %2.2f sec / %2.2f sec\n',T_max,0)
for i = 1:length(t_nlin)
    % input values
    U = -K*X;
    dX = fun_nlin(X(1), X(2), X(3), X(4), X(5), X(6), X(7), X(8), X(9), X(10), U(1), U(2), U(3), 0, 0);

    % integration
    X = X + dX*Ts;
    % save for display 
    x_nlin(i,:) = X;
    u_nlin(i,:) = U;
    fprintf('\b\b\b\b\b\b\b\b\b%2.2f sec\n',t_nlin(i));
end
toc

% linear simulation
disp('lienar system simulation');
tic;
[y_lin,t_lin,x_lin] = initial(feedback(G_lin,K),X0,T_max);
u_lin = -x_lin*K';
toc

%% plotting x
figure(fig1);
for i=1:10
    subplot(5,2,i)
    plot(t_lin,x_lin(:,i));
    hold on
    plot(t_nlin,x_nlin(:,i));
    title(strcat('x',num2str(i)));
    grid on
end
subplot(5,2,1)
legend('linear','non-linear')
subplot(5,2,2)
legend('linear','non-linear')
sgtitle('Initial condition response - Linearized and nonlinear system with LQR controller - states')
% plotting u
figure(fig2)
for i=1:3
    subplot(3,1,i)
    plot(t_lin,u_lin(:,i));
    hold on
    plot(t_nlin,u_nlin(:,i),'r');
    title(strcat('u',num2str(i)));
    grid on
end
subplot(3,1,1)
legend('linear','non-linear')
sgtitle('Initial condition response - Linearized and nonlinear system with LQR controller - control signals')
end


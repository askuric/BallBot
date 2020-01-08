% inital condition
X0 = [pi/10 0 pi/10 0 -pi/10 0 0 0 0 0]';

% nonlinear simulation
Ts = 0.01;
T_max = 4;
t_nlin = [0:Ts:T_max];
X = X0;

x_nlin = zeros(length(t_nlin),10);
u_nlin = zeros(length(t_nlin),3);
disp('nonlinear system simulation');
tic;
for i = 1:length(t_nlin)
    % input values
    U = -K*X;
    dX = nlin_model_wcu(X(1), X(2), X(3), X(4), X(5), X(6), X(7), X(8), X(9), X(10), U(1), U(2), U(3));
    
    % integration
    X = X + dX*Ts;
    % save for display 
    x_nlin(i,:) = X;
    u_nlin(i,:) = U;
    
    fprintf('Nonlinear simulation: %2.2f sec / %2.2f sec\n',t_nlin(i),T_max)
end
toc

% linear simulation
disp('lienar system simulation');
tic;
[y_lin,t_lin,x_lin] = initial(feedback(G_wcu,K),X0,T_max);
toc

%% plotting x
figure(1);
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

% plotting u
figure(2);
for i=1:3
    subplot(3,1,i)
    plot(t_nlin,u_nlin(:,i),'r');
    title(strcat('u',num2str(i)));
    grid on
end
subplot(3,1,1)
legend('non-linear')
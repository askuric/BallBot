function nlsim(fun_nlin, K, u,t_nlin, P, figure_x,figure_u)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% inital condition
    % nonlinear simulation
    Ts = diff(t_nlin(1:2));
    T_max = t_nlin(end);
    X = zeros(10,1);
    U = zeros(3,1);

    x_nlin = zeros(length(t_nlin),10);
    u_nlin = zeros(length(t_nlin),3);
    disp('nonlinear system simulation');
    tic;
	fprintf('Nonlinear simulation: %2.2f sec / %05.2f sec\n',T_max,0)
    for i = 1:length(t_nlin)
        % input values
        U = -K*X;
        dX = fun_nlin(X(1), X(2), X(3), X(4), X(5), X(6), X(7), X(8), X(9), X(10), U(1), U(2), U(3), u(5,i), u(6,i));

        % integration
        X = X + dX*Ts;
        % additive noise measurement
        X(1:4) = X(1:4) + P.d(1:4,1:4)*u(1:4,i); 
        
        % save for display 
        x_nlin(i,:) = X;
        u_nlin(i,:) = U;

        fprintf('\b\b\b\b\b\b\b\b\b\b%05.2f sec\n',t_nlin(i));
    end
    toc

    [y_lin t_lin c]= lsim(lft(P,-K),u,t_nlin);
    
    %% plotting x
    figure(figure_x)
    hold on
    for i=1:10
        subplot(5,2,i)
        plot(t_nlin,y_lin(:,i));
        hold on
        plot(t_nlin,x_nlin(:,i),'r');
        title(strcat('x',num2str(i)));
        grid on
    end
    figure(figure_u)
    % plotting u
    for i=1:3
        subplot(3,1,i)
        plot(t_nlin,y_lin(:,10+i));
        hold on
        plot(t_nlin,u_nlin(:,i),'r');
        title(strcat('u',num2str(i)));
        grid on
    end
%     subplot(12,1,i)
%     legend('linear','non-linear')
%     subplot(5,2,2)
%     legend('linear','non-linear')

    
end


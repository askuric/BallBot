function nlsim(fun_nlin, K, u,t_nlin, P, figure_x, figure_u)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% inital condition
    % nonlinear simulation
    Ts = diff(t_nlin(1:2));
    T_max = t_nlin(end);
    X = zeros(10,1);
    Y = zeros(10,1);
    U = zeros(3,1);

    if(isprop(K,'A'))
        % full state space controller
        nstates = length(K.A(:,1));
        Xk = zeros(nstates,1);
        Kd = c2d(K,Ts);
    else
        % linear gain controller
        Kd = ss([0],zeros(1,10),zeros(3,1),K);
        Xk = 0;
    end
    
    x_nlin = zeros(length(t_nlin),10);
    u_nlin = zeros(length(t_nlin),3);
    disp('nonlinear system simulation');
    tic;
	fprintf('Nonlinear simulation: %2.2f sec / %05.2f sec\n',T_max,0)
    for i = 1:length(t_nlin)
        % input values
        Xk = Kd.a*Xk + Kd.b*Y;
        U = Kd.c*Xk + Kd.d*Y;
        % integration
        %Xk = Xk + dXk*Ts;
        
        dX = fun_nlin(X(1), X(2), X(3), X(4), X(5), X(6), X(7), X(8), X(9), X(10), U(1), U(2), U(3), u(5,i), u(6,i));
           
        % integration
        X = X + dX*Ts;
        % measurement 
        Y = X;
        % additive noise measurement
        Y(1:4) = Y(1:4) + P.d(1:4,1:4)*u(1:4,i); 
        
        % save for display 
        x_nlin(i,:) = Y;
        u_nlin(i,:) = U;

        fprintf('\b\b\b\b\b\b\b\b\b\b%05.2f sec\n',t_nlin(i));
    end
    toc

    [y_lin t_lin c]= lsim(lft(P,K),u,t_nlin);
    
     %% plotting x
    state_var_names = {'\vartheta_x', '\vartheta_x`','\vartheta_y', '\vartheta_y`','\vartheta_z', '\vartheta_z`', '\phi_x', '\phi_x`', '\phi_y', '\phi_y`'};
    figure(figure_x)
    hold on
    for i=1:10
        subplot(5,2,i)
        plot(t_nlin,y_lin(:,i));
        hold on
        plot(t_nlin,x_nlin(:,i));
        title(strcat(state_var_names{i},'(x_{',num2str(i),'}) '));
        if mod(i,2)
            ylabel('rad')
        else
            ylabel('rad/s')
        end
        grid on
    end
    subplot(5,2,1)
    legend('linear','nonlinear')
    drawnow
    
    figure(figure_u)
    % plotting u
    for i=1:3
        subplot(3,1,i)
        plot(t_nlin,y_lin(:,10+i));
        hold on
        plot(t_nlin,u_nlin(:,i));
        title(strcat('T_',num2str(i),'(u_',num2str(i),')'));
        ylabel('Nm')
        grid on
    end
    subplot(3,1,1)
    legend('linear','nonlinear')
    drawnow

    
end

function linsim(G1,color,u,t,fy,fu)
% overwrite of the lsim function for different subplots
    [y,t,x] = lsim(G1,color,u,t);
    
    
    %% plotting x
    state_var_names = {'\vartheta_x', '\vartheta_x`','\vartheta_y', '\vartheta_y`','\vartheta_z', '\vartheta_z`', '\phi_x', '\phi_x`', '\phi_y', '\phi_y`'};
    figure(fy)
    hold on
    for i=1:10
        subplot(5,2,i)
        plot(t,y(:,i));
        hold on
        title(strcat(state_var_names{i},'(x_',num2str(i),') '));
        if mod(i,2)
            ylabel('rad')
        else
            ylabel('rad/s')
        end
        grid on
    end
    drawnow
    figure(fu)
    % plotting u
    for i=1:3
        subplot(3,1,i)
        plot(t,y(:,10+i));
        hold on
        title(strcat('T_',num2str(i),'(u_',num2str(i),')'));
        ylabel('Nm')
        grid on
    end
    drawnow
end


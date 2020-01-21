function initialplt(G,color,X0,T_sim,f)
% overwrite of the initiaplot function for different subplots
    [y t x] = initial(G,color,X0,T_sim);
    %% plotting x
    state_var_names = {'\vartheta_x', '\vartheta_x`','\vartheta_y', '\vartheta_y`','\vartheta_z', '\vartheta_z`', '\phi_x', '\phi_x`', '\phi_y', '\phi_y`'};
    figure(f)
    hold on
    for i=1:10
        subplot(5,2,i)
        plot(t,y(:,i),color);
        hold on
        title(strcat(state_var_names{i},'(x_{',num2str(i),'}) '));
        if mod(i,2)
            ylabel('rad')
        else
            ylabel('rad/s')
        end
        grid on
    end
    drawnow
end


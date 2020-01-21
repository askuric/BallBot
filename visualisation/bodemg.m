function bodemg(G,color,fig)
%overwrite of the bodemag function for better plot
    [mag,phase,wout] = bode(G,{0.1,1e2});
    
    %% plotting x
    state_var_names = {'\vartheta_x', '\vartheta_x`','\vartheta_y', '\vartheta_y`','\vartheta_z', '\vartheta_z`', '\phi_x', '\phi_x`', '\phi_y', '\phi_y`'};
    figure(fig)
    hold on
    for i=1:10
        subplot(5,2,i)
        loglog(wout,mag(i,:),color);
        xlim([0.1,1e2])
        hold on
        title(strcat(state_var_names{i},'(x_{',num2str(i),'}) '));
        ylabel('dB')
        grid on
    end
    drawnow
    
end


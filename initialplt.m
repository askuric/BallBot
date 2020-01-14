function initialplt(G,color,X0,T_sim,f)
% overwrite of the initiaplot function for different subplots
    [y t x] = initial(G,color,X0,T_sim);
    %% plotting x
    figure(f)
    hold on
    for i=1:10
        subplot(5,2,i)
        plot(t,y(:,i),color);
        hold on
        title(strcat('x',num2str(i)));
        grid on
    end
end


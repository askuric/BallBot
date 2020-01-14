function linsim(G1,color,u,t,fy,fu)
% overwrite of the lsim function for different subplots
    [y,t,x] = lsim(G1,color,u,t);
    
    %% plotting x
    figure(fy)
    hold on
    for i=1:10
        subplot(5,2,i)
        plot(t,y(:,i));
        hold on
        grid on
    end
    figure(fu)
    % plotting u
    for i=1:3
        subplot(3,1,i)
        plot(t,y(:,10+i));
        hold on
        grid on
    end
end


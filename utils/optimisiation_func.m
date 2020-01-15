function y = optimisiation_func(x, K , rK_n, rW_n, rA_n, l_n, mAW_n, mK_n, A_ThetaAWx_n, A_ThetaAWy_n, A_ThetaAWz_n, ThetaKi_n, ThetaWi_n)
    %% Criterium function for constraint optimisation
    % Function calculates the DiskMargin of the system provided by the
    % parameres and Controller K
    %
    % Also it throws an error if system is unstable

    % interprete the parameter values
    l_v = l_n*x(1);
    ThetaWi_v = ThetaWi_n*x(2);
    ThetaKi_v = ThetaKi_n*x(3);
    A_ThetaAWx_v = A_ThetaAWx_n*x(4);
    A_ThetaAWy_v = A_ThetaAWy_n*x(5);
    A_ThetaAWz_v = A_ThetaAWz_n*x(6);
    
    
    % A matrix
    A_u= fA(rK_n, rW_n, rA_n, l_v, mAW_n, mK_n, A_ThetaAWx_v, A_ThetaAWy_v, A_ThetaAWz_v, ThetaKi_v, ThetaWi_v);
    % B matrix
    B_u= fB(rK_n, rW_n, rA_n, l_v, mAW_n, mK_n, A_ThetaAWx_v, A_ThetaAWy_v, A_ThetaAWz_v, ThetaKi_v, ThetaWi_v);
        
    if ~isstable(feedback(ss(A_u,B_u,eye(10),zeros(10,3)),K)) 
        disp('This controller cannot stabilize full set of suggested parameters - consider narrowing the uncertainties')
        error('System is unstable for parameters - X : %f %f %f %f %f %f %f',...
                                            x(1), x(2), x(3), x(4),x(5),x(6));
    else
        % criteria calculation
        MMIO = diskmargin(ss(A_u,B_u,eye(10),zeros(10,3)),K);
        y = MMIO.DiskMargin;
    end
end


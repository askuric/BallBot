%% Boundaries of system variables system positions
% x =[ thetax   thetay thetaz phix phiy d_thetax d_thetay d_thetaz d_phix
% d_phiy ]
x_min_max = [[-pi/10 -pi pi/10 -pi 0 -pi 0 0 0 0]',...
            [pi/10 pi pi/10 pi 0  pi 0 0 0 0]'];
% maximal torques on motors
T_max = 10; %Nm
u_min_max = [[0 0 0]',...
             [0 0 0]'];    
    
         
%% Adding current stationary point
disp('Adding current stationary point');
% nominal posiiton
%x0 = [0 0 0 0 0  0 0 0 0 0]';
%u0 = [0 0 0]';
% arbitraty positon
x0 = [0 0 0 0 0 0 0 0 0 0]';
u0 = [0 0 0]';


%% Linearisation
disp('Linearisation');
tic;
F1 = simplify(vpa(subs(diff(f,x1), [x; u], [x0; u0]),2));
F2 = simplify(vpa(subs(diff(f,x2), [x; u], [x0; u0]),2));
F3 = simplify(vpa(subs(diff(f,x3), [x; u], [x0; u0]),2));
F4 = simplify(vpa(subs(diff(f,x4), [x; u], [x0; u0]),2));
F5 = simplify(vpa(subs(diff(f,x5), [x; u], [x0; u0]),2));
F6 = simplify(vpa(subs(diff(f,x6), [x; u], [x0; u0]),2));
F7 = simplify(vpa(subs(diff(f,x7), [x; u], [x0; u0]),2));
F8 = simplify(vpa(subs(diff(f,x8), [x; u], [x0; u0]),2));
F9 = simplify(vpa(subs(diff(f,x9), [x; u], [x0; u0]),2));
F10 = simplify(vpa(subs(diff(f,x10), [x; u], [x0; u0]),2));
F11 = simplify(vpa(subs(diff(f,u1), [x; u], [x0; u0]),2));
F12 = simplify(vpa(subs(diff(f,u2), [x; u], [x0; u0]),2));
F13 = simplify(vpa(subs(diff(f,u3), [x; u], [x0; u0]),2));
toc

%% Linear state space model
disp('Linear state space model');
A = eval([F1 F2 F3 F4 F5 F6 F7 F8 F9 F10]);
B = eval([F11 F12 F13]);
         
A_m = {};
B_m = {};
for min_max = 1:2
    for iter = 1:10
        fprintf(' Iteration: %d, min_max : %d \n', iter, min_max);
        %% Adding current stationary point
        disp('Adding current stationary point');
        % nominal posiiton
        %x0 = [0 0 0 0 0  0 0 0 0 0]';
        %u0 = [0 0 0]';
        % arbitraty positon
        x0 = [0 0 0 0 0 0 0 0 0 0]';
        x0(iter) = x_min_max(iter,min_max);
        if x_min_max(iter,min_max) == 0 
            A_m{iter,min_max} = A;
            B_m{iter,min_max} = B;
            disp('nominal model');
            continue;
        end
        u0 = [0 0 0]';


        %% Linearisation
        disp('Linearisation');
        tic;
        F1 = simplify(vpa(subs(diff(f,x1), [x; u], [x0; u0]),2));
        F2 = simplify(vpa(subs(diff(f,x2), [x; u], [x0; u0]),2));
        F3 = simplify(vpa(subs(diff(f,x3), [x; u], [x0; u0]),2));
        F4 = simplify(vpa(subs(diff(f,x4), [x; u], [x0; u0]),2));
        F5 = simplify(vpa(subs(diff(f,x5), [x; u], [x0; u0]),2));
        F6 = simplify(vpa(subs(diff(f,x6), [x; u], [x0; u0]),2));
        F7 = simplify(vpa(subs(diff(f,x7), [x; u], [x0; u0]),2));
        F8 = simplify(vpa(subs(diff(f,x8), [x; u], [x0; u0]),2));
        F9 = simplify(vpa(subs(diff(f,x9), [x; u], [x0; u0]),2));
        F10 = simplify(vpa(subs(diff(f,x10), [x; u], [x0; u0]),2));
        F11 = simplify(vpa(subs(diff(f,u1), [x; u], [x0; u0]),2));
        F12 = simplify(vpa(subs(diff(f,u2), [x; u], [x0; u0]),2));
        F13 = simplify(vpa(subs(diff(f,u3), [x; u], [x0; u0]),2));
        toc

        %% Linear state space model
        disp('Linear state space model');
        A_m{iter,min_max} = eval([F1 F2 F3 F4 F5 F6 F7 F8 F9 F10]);
        B_m{iter,min_max} = eval([F11 F12 F13]);

    end
end


%% check 

A_M = zeros(size(A));
B_M = zeros(size(B));

for min_max = 1:2
    for iter = 1:10
        A_M( A_M < A_m{iter,min_max} ) = A_m{iter,min_max}( A_M < A_m{iter,min_max} );
        B_M( B_M < B_m{iter,min_max} ) = B_m{iter,min_max}( B_M < B_m{iter,min_max} );
    end
end

clear all

%% System symbolic parameters
syms t real
% state variables
syms q thetax thetay thetaz phix phiy d_thetax d_thetay d_thetaz d_phix d_phiy real
% additional variables
syms psi1 psi2 psi3 d_psi1 d_psi2 d_psi3 real


%% Model physical parameters
disp('Model physical parameters');

% take symbolic params
syms rK rW rA l mAW mK A_ThetaAWx A_ThetaAWy A_ThetaAWz ThetaKi ThetaWi real;
parmas_s = [rK rW rA l mAW mK A_ThetaAWx A_ThetaAWy A_ThetaAWz ThetaKi ThetaWi];

A_ThetaAW = [ A_ThetaAWx, 0, 0; 0, A_ThetaAWy, 0; 0, 0, A_ThetaAWz]; % Inertia of ball in the intertial reference frame I or L  
ThetaK = [ ThetaKi, 0, 0; 0, ThetaKi, 0; 0, 0, ThetaKi]; % Inertia of the omni wheel and motor about the motor axis  

% Angle of motors
alpha = 47*pi/180;
beta1 = 0;
beta2 = 2/3*pi;
beta3 = 4/3*pi;

%% Coordinate system transformations
disp('Coordinate system transformations');

% Rotation speed vector of the ball in the Lisa reference frame L (no Rotation of the ball around the z-axis in L, which is same as the z-axis in I)
L_OmegaK = [d_phix, d_phiy, 0]';

% gravity vector
G = [0, 0, -9.81];

% minimal coordinates
q = [thetax, thetay, thetaz, phix, phiy]';
d_q = [d_thetax, d_thetay, d_thetaz, d_phix, d_phiy];

% Coordinates and rotation matrices (I - L - A)
% Rotations around z, y and x Axes
Rz = [cos(thetaz), -sin(thetaz), 0;...
    sin(thetaz), cos(thetaz), 0;...
    0, 0, 1];
Ry = [cos(thetay), 0, sin(thetay);...
    0, 1, 0;...
    -sin(thetay), 0, cos(thetay)];
Rx = [1, 0, 0;...
    0, cos(thetax), -sin(thetax);...
    0, sin(thetax), cos(thetax)];

% Rotation Matrix (L to I and I to L)
R_IL = Rz;
R_LI = R_IL';
% Rotation Matrix (A to I and I to A)
R_IA = R_IL*Ry*Rx;
R_AI = R_IA';

% Ball velocity
OmegaK = R_IL*L_OmegaK;

% Jacobian
J = [ 1, 0, -sin(thetay);... 
    0, cos(thetax), sin(thetax)*cos(thetay);... 
    0, -sin(thetax), cos(thetax)*cos(thetay)];

% Time variation of Tait-Bryan angles
ThetaDot = [d_thetax, d_thetay, d_thetaz]';

% Rotation Vector of body velocity
A_OmegaA = J*ThetaDot;


%% Binding equations
disp('Binding equations');

% Absolut rotation of the omniwheels
% Vector from intersection of the motor directions M to the center point of the omniwheels (W1, W2, W3) in A
A_MW1 = [cos(beta1) * sin(alpha), sin(alpha)*sin(beta1), -cos(alpha)];
A_MW2 = [cos(beta2) * sin(alpha), sin(alpha)*sin(beta2), -cos(alpha)];
A_MW3 = [cos(beta3) * sin(alpha), sin(alpha)*sin(beta3), -cos(alpha)];

% Dependency on the rotation of the omniwheels
% Vector from center of the ball (P) to the contact point with the omniwheels (K1, K2, K3) in A
A_rPK1 = [rK * sin(alpha)*cos(beta1), rK*sin(alpha)*sin(beta1), rK*cos(alpha)];
A_rPK2 = [rK * sin(alpha)*cos(beta2), rK*sin(alpha)*sin(beta2), rK*cos(alpha)];
A_rPK3 = [rK * sin(alpha)*cos(beta3), rK*sin(alpha)*sin(beta3), rK*cos(alpha)];
% Direction of the tangential speed of the rotation of the omniwheels in A
A_d1 = [-sin(beta1), cos(beta1), 0]';
A_d2 = [-sin(beta2), cos(beta2), 0]';
A_d3 = [-sin(beta3), cos(beta3), 0]';
% Rotation speed of the ball in A
A_OmegaK = R_AI*OmegaK;
% Rotation speed of the ball relative to the body in A
A_omegaK = simplify(A_OmegaK - A_OmegaA);
% Speed on the surface of the ball (in omniwheel direction) has to be the same speed as the tangetial speed of the omniwheel
E1 = (cross(A_omegaK, A_rPK1)*A_d1 == d_psi1*rW);
E2 = (cross(A_omegaK, A_rPK2)*A_d2 == d_psi2*rW);
E3 = (cross(A_omegaK, A_rPK3)*A_d3 == d_psi3*rW);
% solve euqations
sol = solve([E1, E2, E3], {d_psi1, d_psi2, d_psi3});
% Simplify
d_psi1 = simplify(sol.d_psi1);
d_psi2 = simplify(sol.d_psi2);
d_psi3 = simplify(sol.d_psi3);
% Relative rotation speed between the omniwheels and body about the motor axis in the body reference frame A (scalar,encoder value)
A_omegaW1 = d_psi1;
A_omegaW2 = d_psi2;
A_omegaW3 = d_psi3;
% Absolute rotation speed of the omniwheels about the motor axis in the body reference frame A (scalar)
A_OmegaW1 = simplify(A_omegaW1 + A_MW1*A_OmegaA);
A_OmegaW2 = simplify(A_omegaW2 + A_MW2*A_OmegaA);
A_OmegaW3 = simplify(A_omegaW3 + A_MW3*A_OmegaA);

% Translation of the ball
% Vector from the ground to center of the ball (P)
r_BP = [0, 0, rK];
% Speed vector of the center of the ball (P) in I
r_Pdot = simplify(cross(OmegaK, r_BP))';

%% Energies
disp('Energies');

% Ball Kinetic
T_K = simplify(1/2*mK*(r_Pdot'*r_Pdot) + 1/2*L_OmegaK'*ThetaK*L_OmegaK);

% Body and Omniwheels
% Vector from center of the ball to the center of gravity of the body in A
A_rPSA = [0, 0, l]';
% Kinetic
T_AW = simplify(1/2*mAW*(r_Pdot'*r_Pdot)...
    + mAW*(R_AI*r_Pdot)'*cross(A_OmegaA, A_rPSA)...
    + 1/2*A_OmegaA'*A_ThetaAW*A_OmegaA);
% Potential
V_AW = -mAW * G*R_IA*A_rPSA;
% Rotational energy of the omniwheels
%Considering only the rotation energy of the omniwheel and motors about the motor axis
T_W1 = 1/2*ThetaWi*A_OmegaW1^2;
T_W2 = 1/2*ThetaWi*A_OmegaW2^2;
T_W3 = 1/2*ThetaWi*A_OmegaW3^2;


%% Non-potential forces
disp('Non-potential forces');

% Torque on omniwheels
[R1, J_T1] = polynomialReduce(A_omegaW1, d_q, d_q);
[R2, J_T2] = polynomialReduce(A_omegaW2, d_q, d_q);
[R3, J_T3] = polynomialReduce(A_omegaW3, d_q, d_q);
J_T1 = simplify(J_T1);
J_T2 = simplify(J_T2);
J_T3 = simplify(J_T3);

% Torques
syms T1 T2 T3;

% Counter torques on body
[RC1, J_C1] = polynomialReduce(A_OmegaA(1), d_q, d_q);
[RC2, J_C2] = polynomialReduce(A_OmegaA(2), d_q, d_q);
[RC3, J_C3] = polynomialReduce(A_OmegaA(3), d_q, d_q);

J_CT = [simplify(J_C1); simplify(J_C2); simplify(J_C3)];
% Counter torques
TC1 = A_MW1'*(-T1);
TC2 = A_MW2'*(-T2);
TC3 = A_MW3'*(-T3);
% f_NP
fNP = simplify(J_T1'*T1 + J_T2'*T2+ J_T3'*T3 + J_CT'*TC1 + J_CT'*TC2 + J_CT'*TC3);

%% Lagrange 
disp('Lagrange');
tic;
T = T_K + T_AW + T_W1 + T_W2 + T_W3;
V = V_AW;

% temporal substitution
% state variables
syms q_t th_x(t) th_y(t) th_z(t) ph_x(t) ph_y(t) real
% additional variables
syms psi_1(t) psi_2(t) psi_3(t) real
assume(th_x(t),'real');
assume(th_y(t),'real');
assume(th_z(t),'real');
assume(ph_x(t),'real');
assume(ph_y(t),'real');
assume(psi_1(t),'real');
assume(psi_2(t),'real');
assume(psi_3(t),'real');
assume(q_t,'real');

qt = [th_x(t) th_y(t) th_z(t) ph_x(t) ph_y(t)];
d_qt = diff(qt);
% d2T/dq'dt
L1_ = [diff(T,d_thetax) diff(T,d_thetay) diff(T,d_thetaz) diff(T,d_phix) diff(T,d_phiy)]';
L1 =  subs(L1_, [q', d_q], [qt, d_qt]); 
L1 = diff(L1,t);
% dT/dq
L2_ = [diff(T,thetax) diff(T,thetay) diff(T,thetaz) diff(T,phix) diff(T,phiy)]';
L2 =  subs(L2_, [q', d_q], [qt, d_qt]); 
% dV/dq
L3_ = [diff(V,thetax) diff(V,thetay) diff(V,thetaz) diff(V,phix) diff(V,phiy)]';
L3 =  subs(L3_, [q', d_q], [qt, d_qt]); 
% non-potential forces
fNP = subs(fNP, [q', d_q], [qt, d_qt]); 
% Full lagrange
EQ_ = L1 + L2 + L3 - fNP;

% Solution
syms d2_th_x d2_th_y  d2_th_z d2_ph_y d2_ph_x real
EQ = subs(EQ_,{diff(diff(th_x,t),t), diff(diff(th_y,t),t) ,diff(diff(th_z,t),t) , diff(diff(ph_x,t),t), diff(diff(ph_y,t),t)},{d2_th_x,d2_th_y,d2_th_z,d2_ph_x,d2_ph_y});

varList = [d2_th_x d2_th_y d2_th_z d2_ph_x d2_ph_y];
[r1, q1] = polynomialReduce(EQ(1), varList, varList);
[r2, q2] = polynomialReduce(EQ(2), varList, varList);
[r3, q3] = polynomialReduce(EQ(3), varList, varList);
[r4, q4] = polynomialReduce(EQ(4), varList, varList);
[r5, q5] = polynomialReduce(EQ(5), varList, varList);

toc
%% Linear equation solving
disp('Linear equation solving');
tic;
syms q11 q12 q13 q14 q15 q21 q22 q23 q24 q25 q31 q32 q33 q34 q35 q41 q42 q43 q44 q45 q51 q52 q53 q54 q55 real
syms r1s r2s r3s r4s r5s real

q1s = [q11 q12 q13 q14 q15];
q2s = [q21 q22 q23 q24 q25];
q3s = [q31 q32 q33 q34 q35];
q4s = [q41 q42 q43 q44 q45];
q5s = [q51 q52 q53 q54 q55];

linsol = linsolve([q1s; q2s; q3s; q4s; q5s],[-r1s -r2s -r3s -r4s -r5s]');

% final model equations
model = subs(linsol, {q11 q12 q13 q14 q15 q21 q22 q23 q24 q25 q31 q32 q33 q34 q35 q41 q42 q43 q44 q45 q51 q52 q53 q54 q55 r1s r2s r3s r4s r5s},...
     {q1(1) q1(2) q1(3) q1(4) q1(5) q2(1) q2(2) q2(3) q2(4) q2(5) q3(1) q3(2) q3(3) q3(4) q3(5) q4(1) q4(2) q4(3) q4(4) q4(5) q5(1) q5(2) q5(3) q5(4) q5(5) r1 r2 r3 r4 r5});

toc

%% State space representation
disp('State space representation');
tic;
% state variables
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 u1 u2 u3 real
assume(x1,'real');
assume(x2,'real');
assume(x3,'real');
assume(x4,'real');
assume(x5,'real');
assume(x6,'real');
assume(x7,'real');
assume(x8,'real');
assume(x9,'real');
assume(x10,'real');
assume(u1,'real');
assume(u2,'real');
assume(u3,'real');

x = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10]';
u = [u1 u2 u3]';
h = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10]';

% state space equations of movement
f_ = [x2, model(1), x4, model(2), x6, model(3), x8, model(4), x10, model(5)]';
% state substitution siplification
f_ = subs(f_, {diff(th_x,t), diff(th_y,t), diff(th_z,t), diff(ph_x,t), diff(ph_y,t)}, {x2 x4 x6 x8 x10});
f_ = subs(f_, {th_x, th_y, th_z, ph_x, ph_y}, {x1 x3 x5 x7 x9});
f_par = subs(f_, {T1, T2, T3}, {u1 u2 u3});
toc

% saving the nonlinear model to the mat file for further use
n_model = f_par;
save nlin_model n_model;

%% Adding linearisation point
disp('Adding linearisation point');
% nominal posiiton
x0 = [0 0 0 0 0  0 0 0 0 0]';
u0 = [0 0 0]';
%% Linearisation
disp('Linearisation');
tic;
A_ = {};
B_ = {};
A_{1} = simplify(vpa(subs(diff(n_model,x1), [x; u], [x0; u0]),2));
A_{2} = simplify(vpa(subs(diff(n_model,x2), [x; u], [x0; u0]),2));
A_{3} = simplify(vpa(subs(diff(n_model,x3), [x; u], [x0; u0]),2));
A_{4} = simplify(vpa(subs(diff(n_model,x4), [x; u], [x0; u0]),2));
A_{5} = simplify(vpa(subs(diff(n_model,x5), [x; u], [x0; u0]),2));
A_{6} = simplify(vpa(subs(diff(n_model,x6), [x; u], [x0; u0]),2));
A_{7} = simplify(vpa(subs(diff(n_model,x7), [x; u], [x0; u0]),2));
A_{8} = simplify(vpa(subs(diff(n_model,x8), [x; u], [x0; u0]),2));
A_{9} = simplify(vpa(subs(diff(n_model,x9), [x; u], [x0; u0]),2));
A_{10} = simplify(vpa(subs(diff(n_model,x10), [x; u], [x0; u0]),2));
B_{1} = simplify(vpa(subs(diff(n_model,u1), [x; u], [x0; u0]),2));
B_{2} = simplify(vpa(subs(diff(n_model,u2), [x; u], [x0; u0]),2));
B_{3} = simplify(vpa(subs(diff(n_model,u3), [x; u], [x0; u0]),2));
toc

%% Create A and B parametric functions
disp('Create A and B parametric functions');
tic;
matlabFunction(A_,'file','fA','Optimize',false, 'Vars',[rK rW rA l mAW mK A_ThetaAWx A_ThetaAWy A_ThetaAWz ThetaKi ThetaWi]);
matlabFunction(B_,'file','fB','Optimize',false, 'Vars',[rK rW rA l mAW mK A_ThetaAWx A_ThetaAWy A_ThetaAWz ThetaKi ThetaWi]);
toc

%% Finish
disp('Linear model created');
clear all

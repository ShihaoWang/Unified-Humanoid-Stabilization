function Eleven_Link_Model_Sym()

% This function is used to generate the equation of motion for the 11-link
% humanoid robot

% The item sequence is [foot, shank, thigh, body ,head, forearm, arm]
p = Humanoid_Physical_Parameter();
Length_Array = p.l;
Mass_Array = p.m;
Inertia_Array = p.I;
g = 9.81;

syms theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 real
syms thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot real
syms thetaddot q1ddot q2ddot q3ddot q4ddot q5ddot q6ddot q7ddot q8ddot q9ddot q10ddot real
syms u1 u2 u3 u4 u5 u6 u7 u8 u9 u10 real

syms rAx rAxdot rAxddot rAy rAydot rAyddot real % The default configuration is that the front side of the stance leg is contacting the ground

% rO = [0 0]';
% rOA = [x_A, y_A]';
% rA = rOA + rO;

rA = [rAx, rAy]';

% Stance foot
rAB = Length_Array(1) * lamda_direction(theta);
rB = rA + rAB; 
rE = (rA + rB)/2;

% Stance shank
Ang_E = q6 + theta - pi;
rEF = Length_Array(2) * lamda_direction(Ang_E);
rF = rE + rEF;

% Stance thigh
Ang_F = pi - (q5 - Ang_E);
rFI = Length_Array(3) * lamda_direction(Ang_F);
rI = rF + rFI;

% Body
Ang_I = 2 * pi - q4 - (pi - Ang_F);
rIJ = Length_Array(4) * lamda_direction(Ang_I);
rJ = rI + rIJ;

% Head
rJK = Length_Array(5) * lamda_direction(Ang_I);
rK = rJ + rJK;

% Stance side forearm
Ang_J = q9 - (pi - Ang_I);
rJL = Length_Array(6) * lamda_direction(Ang_J);
rL = rJ + rJL;

% Stance side arm
Ang_L = q10 + Ang_J;
rLM = Length_Array(7) * lamda_direction(Ang_L);
rM = rL + rLM;

% Swing side forearm
Ang_N = Ang_I - q7;
rJN = Length_Array(6) * lamda_direction(-(pi - Ang_N));
rN = rJ + rJN;

% Swing side arm
Ang_O = q8 + Ang_N;
rNO = Length_Array(7) * lamda_direction( - (pi - Ang_O));
rO = rN + rNO;

% Swing side thigh
rIH = Length_Array(3) * lamda_direction(-(2 * pi  - Ang_I - q3));
rH = rI + rIH;

% Swing side shank
Ang_H = pi - (2 * pi - Ang_I - q3);
rHG = Length_Array(2) * lamda_direction(-(2*pi - Ang_H - q2));
rG = rH + rHG;

% Swing side foot
Ang_G = pi - (2 * pi - Ang_H - q2);
Ang_GC = -(q1 - Ang_G);
rGC = 0.5 * Length_Array(1) * lamda_direction(Ang_GC);
rC = rGC + rG;
rD = rG - rGC;

% Due to the multiple contact case, it is desired to have a generalized
% equation of motion to take care of the robot motion in different modes.

% Due to the logical case at each contact point, there are additional
% contact mode to be considered. 


% This part is the generalized coordinate of the robots
q = [rAx, rAy, theta,...
    q1,q2,q3,q4,q5,q6,q7,q8,q9,q10]';
qdot = [rAxdot, rAydot, thetadot,...
       q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot]';
qddot = [rAxddot, rAyddot, thetaddot,...
    q1ddot,q2ddot,q3ddot,q4ddot,q5ddot,q6ddot,q7ddot,q8ddot,q9ddot,q10ddot]';

% Velocity computation
vA = jacobian(rA, q) * qdot; 
vB = jacobian(rB, q) * qdot; 
vC = jacobian(rC, q) * qdot; 
vD = jacobian(rD, q) * qdot; 
vE = jacobian(rE, q) * qdot; 
vF = jacobian(rF, q) * qdot; 
vG = jacobian(rG, q) * qdot; 
vH = jacobian(rH, q) * qdot; 
vI = jacobian(rI, q) * qdot; 
vJ = jacobian(rJ, q) * qdot; 
vK = jacobian(rK, q) * qdot; 
vL = jacobian(rL, q) * qdot; 
vM = jacobian(rM, q) * qdot; 
vN = jacobian(rN, q) * qdot; 
vO = jacobian(rO, q) * qdot; 

% Stance foot energy
Ang_Rate_Stance_Foot = thetadot;
T_Stance_Foot = 1/2 * Mass_Array(1) * dot(vE, vE) + 1/2 * Inertia_Array(1) * Ang_Rate_Stance_Foot * Ang_Rate_Stance_Foot;
V_Stance_Foot = Mass_Array(1) * g * rE(2);

% Stance shank energy
Ang_Rate_Stance_Shank = jacobian(Ang_E, q) * qdot;
T_Stance_Shank = 1/2 * Mass_Array(2) * dot(0.5 * (vE + vF), 0.5 * (vE + vF)) + 1/2 * Inertia_Array(2) * dot(Ang_Rate_Stance_Shank,Ang_Rate_Stance_Shank);
V_Stance_Shank = Mass_Array(2) * g * (rE(2) + rF(2))/2;

% Stance thigh energy
Ang_Rate_Stance_Thigh = jacobian(Ang_F, q) * qdot;
T_Stance_Thigh = 1/2 * Mass_Array(3) * dot(0.5 * (vF + vI), 0.5 * (vF + vI)) + 1/2 * Inertia_Array(3) * dot(Ang_Rate_Stance_Thigh,Ang_Rate_Stance_Thigh);
V_Stance_Thigh = Mass_Array(3) * g * (rF(2) + rI(2))/2;

% Body
Ang_Rate_Body = jacobian(Ang_I, q) * qdot;
T_Body = 1/2 * (Mass_Array(4) +  Mass_Array(5))* dot(0.5 * (vI + vK), 0.5 * (vI + vK)) + 1/2 * (Inertia_Array(4) + Inertia_Array(5) )* dot(Ang_Rate_Body,Ang_Rate_Body);
V_Body = (Mass_Array(4) +  Mass_Array(5)) * g * (rI(2) + rK(2))/2;

% Stance forearm
Ang_Rate_Stance_Forearm = jacobian(Ang_J, q) * qdot;
T_Stance_Forearm = 1/2 * Mass_Array(6) * dot(0.5 * (vJ + vL), 0.5 * (vJ + vL)) + 1/2 * Inertia_Array(6) * dot(Ang_Rate_Stance_Forearm,Ang_Rate_Stance_Forearm);
V_Stance_Forearm = Mass_Array(6) * g * (rJ(2) + rL(2))/2;

% Stance arm
Ang_Rate_Stance_Arm = jacobian(Ang_L, q) * qdot;
T_Stance_Arm = 1/2 * Mass_Array(7) * dot(0.5 * (vL + vM), 0.5 * (vL + vM)) + 1/2 * Inertia_Array(7) * dot(Ang_Rate_Stance_Arm,Ang_Rate_Stance_Arm);
V_Stance_Arm = Mass_Array(7) * g * (rL(2) + rM(2))/2;

% Swing forearm
Ang_Rate_Swing_Forearm= jacobian(Ang_N, q) * qdot;
T_Swing_Forearm = 1/2 * Mass_Array(6) * dot(0.5 * (vJ + vN), 0.5 * (vJ + vN)) + 1/2 * Inertia_Array(6) * dot(Ang_Rate_Swing_Forearm,Ang_Rate_Swing_Forearm);
V_Swing_Forearm = Mass_Array(6) * g * (rJ(2) + rN(2))/2;

% Swing arm
Ang_Rate_Swing_Arm= jacobian(Ang_O, q) * qdot;
T_Swing_Arm = 1/2 * Mass_Array(7) * dot(0.5 * (vN + vO), 0.5 * (vN + vO)) + 1/2 * Inertia_Array(7) * dot(Ang_Rate_Swing_Arm,Ang_Rate_Swing_Arm);
V_Swing_Arm = Mass_Array(7) * g * (rN(2) + rO(2))/2;

% Swing thigh
Ang_Rate_Swing_Thigh = jacobian(Ang_H, q) * qdot;
T_Swing_Thigh = 1/2 * Mass_Array(3) * dot(0.5 * (vI + vH), 0.5 * (vI + vH)) + 1/2 * Inertia_Array(3) * dot(Ang_Rate_Swing_Thigh,Ang_Rate_Swing_Thigh);
V_Swing_Thigh = Mass_Array(3) * g * (rI(2) + rH(2))/2;

% Swing shank
Ang_Rate_Swing_Shank = jacobian(Ang_G, q) * qdot;
T_Swing_Shank = 1/2 * Mass_Array(2) * dot(0.5 * (vH + vG), 0.5 * (vH + vG)) + 1/2 * Inertia_Array(2) * dot(Ang_Rate_Swing_Shank,Ang_Rate_Swing_Shank);
V_Swing_Shank = Mass_Array(2) * g * (rH(2) + rG(2))/2;

% Swing foot
Ang_Rate_Swing_Foot = jacobian(Ang_GC, q) * qdot;
T_Swing_Foot = 1/2 * Mass_Array(1) * dot(0.5 * (vD + vC), 0.5 * (vD + vC)) + 1/2 * Inertia_Array(1) * dot(Ang_Rate_Swing_Foot,Ang_Rate_Swing_Foot);
V_Swing_Foot = Mass_Array(1) * g * (rC(2) + rD(2))/2;

T = T_Stance_Foot + T_Stance_Shank + T_Stance_Thigh + T_Body + T_Stance_Forearm + T_Stance_Arm + ...
    T_Swing_Foot + T_Swing_Shank + T_Swing_Thigh + T_Swing_Forearm + T_Swing_Arm;
V = V_Stance_Foot + V_Stance_Shank + V_Stance_Thigh + V_Body + V_Stance_Forearm + V_Stance_Arm + ...
    V_Swing_Foot + V_Swing_Shank + V_Swing_Thigh + V_Swing_Forearm + V_Swing_Arm; 

% T = simplify(T);
V = simplify(V);

% T = 

% T = 
L = T - simplify(V);
pL_pq = jacobian(L, q);
pL_pqdot = jacobian(L,qdot);
d_pL_pqdot_dt = jacobian(pL_pqdot, [q; qdot]) * [qdot; qddot];

Eqn = pL_pq' - d_pL_pqdot_dt - [0 0 0 u1 u2 u3 u4 u5 u6 u7 u8 u9 u10]';

end
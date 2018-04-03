function Eleven_Link_Model_WRT_Hip()

% This function is used to generate the equation of motion for the 11-link
% humanoid robot with a different point of view to treat the 
% The item sequence is [foot, shank, thigh, body ,head, forearm, arm]
p = Humanoid_Physical_Parameter();
linklength = p.l;
linkmass = p.m;
linkinertia = p.I;
g = 9.81;

P.mg = sum(linkmass) * g;
P.contact_position_name = char('rA', 'rB', 'rC', 'rD', 'rM', 'rO');
P.contact_velocity_name = char('vA', 'vB', 'vC', 'vD', 'vM', 'vO');

syms theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 real
syms thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot real
syms thetaddot q1ddot q2ddot q3ddot q4ddot q5ddot q6ddot q7ddot q8ddot q9ddot q10ddot real
syms u1 u2 u3 u4 u5 u6 u7 u8 u9 u10 real

syms rIx rIxdot rIxddot rIy rIydot rIyddot real % The default configuration is that the front side of the stance leg is contacting the ground

AngxIK = theta;
AngxIH = -(2 * pi - theta - q3);
AngxIF = -(2 * pi - theta - q4);
AngIFx = pi + AngxIF;
AngIHx = pi + AngxIH;
AngxFE = -(2 * pi - AngIFx - q5);
AngFEx = pi + AngxFE;
AngxBA = -(q6 - AngFEx);
AngxHG = -(2 * pi - AngIHx  -q2);
AngHGx = pi + AngxHG;
AngxDC = -(q1 - AngHGx);
AngxJL = -(pi - AngxIK - q9);
AngJLx = pi + AngxJL;
AngMLx = q10 - (pi - AngJLx);
AngxJN = -(q7 + q9 -AngxJL);
AngJNx = pi + AngxJN;
AngxNO = -(pi - q8 - AngJNx);
AngNOx = pi + AngxNO;

rI = [rIx, rIy]';
rIF = linklength(3) * lamdadirection(AngxIF);
rF = rI + rIF;
rE = linklength(2) * lamdadirection(AngxFE) + rF;
rA = 1/2 * linklength(1) * lamdadirection(AngxBA) + rE;
rB = 1/2 * linklength(1) * lamdadirection(pi + AngxBA) + rE;
rH = rI + linklength(3) * lamdadirection(AngxIH);
rG = rH + linklength(2) * lamdadirection(AngxHG);
rC = rG + 1/2 * linklength(1) * lamdadirection(AngxDC);
rD = rG - 1/2 * linklength(1) * lamdadirection(AngxDC);
rJ = rI + linklength(4) * lamdadirection(AngxIK);
rK = rJ + linklength(5) * lamdadirection(AngxIK);
rL = rJ + linklength(6) * lamdadirection(AngxJL);
rM = rL + linklength(7) * lamdadirection(AngMLx);
rN = rJ + linklength(6) * lamdadirection(AngxJN);
rO = rN + linklength(7) * lamdadirection(AngxNO);

% Here Q is a structure used to save the symbolic expression
Q.rA = rA;
Q.rB = rB;
Q.rC = rC;
Q.rD = rD;
Q.rE = rE;
Q.rF = rF;
Q.rG = rG;
Q.rH = rH;
Q.rI = rI;
Q.rJ = rJ;
Q.rK = rK;
Q.rL = rL;
Q.rM = rM;
Q.rN = rN;
Q.rO = rO;

% Due to the multiple contact case, it is desired to have a generalized
% equation of motion to take care of the robot motion in different modes.

% Due to the logical case at each contact point, there are additional
% contact mode to be considered. 


% This part is the generalized coordinate of the robots
q = [rIx, rIy, theta,...
    q1,q2,q3,q4,q5,q6,q7,q8,q9,q10]';
qdot = [rIxdot, rIydot, thetadot,...
       q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot]';
qddot = [rIxddot, rIyddot, thetaddot,...
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

Q.vA = vA;
Q.vB = vB;
Q.vC = vC;
Q.vD = vD;
Q.vE = vE;
Q.vF = vF;
Q.vG = vG;
Q.vH = vH;
Q.vI = vI;
Q.vJ = vJ;
Q.vK = vK;
Q.vL = vL;
Q.vM = vM;
Q.vN = vN;
Q.vO = vO;

% Stance foot energy
AngRateAB = jacobian(AngxBA,q) * qdot;
TAB = 1/2 * linkmass(1) * dot(0.5 * (vA + vB), 0.5 * (vA + vB)) + 1/2 * linkinertia(1) * AngRateAB * AngRateAB;
VAB = linkmass(1) * g * rE(2);

% Stance shank energy
AngRateEF = jacobian(AngFEx, q) * qdot;
TEF = 1/2 * linkmass(2) * dot(0.5 * (vE + vF), 0.5 * (vE + vF)) + 1/2 * linkinertia(2) * dot(AngRateEF,AngRateEF);
VEF = linkmass(2) * g * (rE(2) + rF(2))/2;

% Stance thigh energy
AngRateFI = jacobian(AngIFx, q) * qdot;
TFI = 1/2 * linkmass(3) * dot(0.5 * (vF + vI), 0.5 * (vF + vI)) + 1/2 * linkinertia(3) * dot(AngRateFI,AngRateFI);
VFI = linkmass(3) * g * (rF(2) + rI(2))/2;

% Body
AngRateIK = jacobian(AngxIK, q) * qdot;
TIK = 1/2 * (linkmass(4) +  linkmass(5))* dot(0.5 * (vI + vK), 0.5 * (vI + vK)) + 1/2 * (linkinertia(4) + linkinertia(5) )* dot(AngRateIK,AngRateIK);
VIK = (linkmass(4) +  linkmass(5)) * g * (rI(2) + rK(2))/2;

% Stance forearm
AngRateJL = jacobian(AngxJL, q) * qdot;
TJL = 1/2 * linkmass(6) * dot(0.5 * (vJ + vL), 0.5 * (vJ + vL)) + 1/2 * linkinertia(6) * dot(AngRateJL,AngRateJL);
VJL = linkmass(6) * g * (rJ(2) + rL(2))/2;

% Stance arm
AngRateLM = jacobian(AngMLx, q) * qdot;
TLM = 1/2 * linkmass(7) * dot(0.5 * (vL + vM), 0.5 * (vL + vM)) + 1/2 * linkinertia(7) * dot(AngRateLM,AngRateLM);
VLM = linkmass(7) * g * (rL(2) + rM(2))/2;

% Swing forearm
AngRateJN= jacobian(AngJNx, q) * qdot;
TJN = 1/2 * linkmass(6) * dot(0.5 * (vJ + vN), 0.5 * (vJ + vN)) + 1/2 * linkinertia(6) * dot(AngRateJN,AngRateJN);
VJN = linkmass(6) * g * (rJ(2) + rN(2))/2;

% Swing arm
AngRateNO= jacobian(AngNOx, q) * qdot;
TNO = 1/2 * linkmass(7) * dot(0.5 * (vN + vO), 0.5 * (vN + vO)) + 1/2 * linkinertia(7) * dot(AngRateNO,AngRateNO);
VNO = linkmass(7) * g * (rN(2) + rO(2))/2;

% Swing thigh
AngRateIH = jacobian(AngIHx, q) * qdot;
TIH = 1/2 * linkmass(3) * dot(0.5 * (vI + vH), 0.5 * (vI + vH)) + 1/2 * linkinertia(3) * dot(AngRateIH,AngRateIH);
VIH = linkmass(3) * g * (rI(2) + rH(2))/2;

% Swing shank
AngRateHG = jacobian(AngHGx, q) * qdot;
THG = 1/2 * linkmass(2) * dot(0.5 * (vH + vG), 0.5 * (vH + vG)) + 1/2 * linkinertia(2) * dot(AngRateHG,AngRateHG);
VHG = linkmass(2) * g * (rH(2) + rG(2))/2;

% Swing foot
AngRateDC = jacobian(AngxDC, q) * qdot;
TDC = 1/2 * linkmass(1) * dot(0.5 * (vD + vC), 0.5 * (vD + vC)) + 1/2 * linkinertia(1) * dot(AngRateDC,AngRateDC);
VDC = linkmass(1) * g * (rC(2) + rD(2))/2;

T = TAB + TEF + TFI + TIK + TJL + TLM + TDC + THG + TIH + TJN + TNO;
V = VAB + VEF + VFI + VIK + VJL + VLM + VDC + VHG + VIH + VJN + VNO; 

T = simplify(T);
V = simplify(V);

Q.T = T;

T_fn = matlabFunction(T);
P.T_fn = T_fn;

L = T - V;
pL_pq = simplify(jacobian(L, q));
pL_pqdot = simplify(jacobian(L,qdot));
d_pL_pqdot_dt = simplify(jacobian(pL_pqdot, [q; qdot]) * [qdot; qddot]);

Eqn = d_pL_pqdot_dt - pL_pq' - [0 0 0 u1 u2 u3 u4 u5 u6 u7 u8 u9 u10]';
Eqn = simplify(Eqn);
u = [u1 u2 u3 u4 u5 u6 u7 u8 u9 u10]'; 
D_q = jacobian(Eqn, qddot);
B_q = -jacobian(Eqn, u);
C_q_qdot = simplify(Eqn - D_q * qddot + B_q * u);

Q.D_q = D_q;
Q.B_q = B_q;
Q.C_q_qdot = C_q_qdot;

% Now let us come to the computation of the full dynamics constraint jacobian matrix

syms rAx rAy rBx rBy rCx rCy rDx rDy rMx rMy rOx rOy real
Phi_A = [rAx; rAy] - rA;
Phi_B = [rBx; rBy] - rB;
Phi_C = [rCx; rCy] - rC;
Phi_D = [rDx; rDy] - rD;
Phi_M = [rMx; rMy] - rM;
Phi_O = [rOx; rOy] - rO;

Phi_Eqn = [-Phi_A; -Phi_B; -Phi_C; -Phi_D; -Phi_M; -Phi_O];
Jac_Full = jacobian(Phi_Eqn, q);
Q.Jac_Full = Jac_Full;

P.Phi_A_fn = matlabFunction(Phi_A); %@(q4,q5,q6,rAx,rAy,rIx,rIy,theta)
P.Phi_B_fn = matlabFunction(Phi_B); %@(q4,q5,q6,rBx,rBy,rIx,rIy,theta)
P.Phi_C_fn = matlabFunction(Phi_C); %@(q1,q2,q3,rCx,rCy,rIx,rIy,theta)
P.Phi_D_fn = matlabFunction(Phi_D); %@(q1,q2,q3,rDx,rDy,rIx,rIy,theta)
P.Phi_M_fn = matlabFunction(Phi_M); %@(q9,q10,rIx,rIy,rMx,rMy,theta)
P.Phi_O_fn = matlabFunction(Phi_O); %@(q7,q8,rIx,rIy,rOx,rOy,theta)

% Jac_A = simplify((jacobian(Phi_A,q) * qdot));
% Jac_B = simplify((jacobian(Phi_A,q) * qdot));
% Jac_C = simplify((jacobian(Phi_A,q) * qdot));
% Jac_D = simplify((jacobian(Phi_A,q) * qdot));
% Jac_M = simplify((jacobian(Phi_A,q) * qdot));
% Jac_O = simplify((jacobian(Phi_A,q) * qdot));
% 
% P.Jac_A_fn = matlabFunction(Jac_A);%@(q4,q5,q6,theta)
% P.Jac_B_fn = matlabFunction(Jac_B);%@(q4,q5,q6,theta)
% P.Jac_C_fn = matlabFunction(Jac_C);%@(q1,q2,q3,theta)
% P.Jac_D_fn = matlabFunction(Jac_D);%@(q1,q2,q3,theta)
% P.Jac_M_fn = matlabFunction(Jac_M);%@(q9,q10,theta)
% P.Jac_O_fn = matlabFunction(Jac_O);%@(q7,q8,theta)


P.Jac_Full_fn = matlabFunction(Jac_Full); %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta)

Jac_Full_dot_Eqn = simplify(jacobian(Jac_Full * qdot, [q;qdot]) * [qdot;qddot]);
Jacdot_qdot = simplify(Jac_Full_dot_Eqn - Jac_Full * qddot);

Q.Jac_Full_dot_Eqn = Jac_Full_dot_Eqn; 
Q.Jacdot_qdot = Jacdot_qdot;

P.Jacdot_qdot_fn = matlabFunction(Jacdot_qdot); %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta)

% Jac_A_x_dot = simplify(jacobian(Jac_A(1), q) * qdot);
% Jac_A_y_dot = simplify(jacobian(Jac_A(2), q) * qdot);
% Jac_B_x_dot = simplify(jacobian(Jac_B(1), q) * qdot);
% Jac_B_y_dot = simplify(jacobian(Jac_B(2), q) * qdot);
% Jac_C_x_dot = simplify(jacobian(Jac_C(1), q) * qdot);
% Jac_C_y_dot = simplify(jacobian(Jac_C(2), q) * qdot);
% Jac_D_x_dot = simplify(jacobian(Jac_D(1), q) * qdot);
% Jac_D_y_dot = simplify(jacobian(Jac_D(2), q) * qdot);
% Jac_M_x_dot = simplify(jacobian(Jac_M(1), q) * qdot);
% Jac_M_y_dot = simplify(jacobian(Jac_M(2), q) * qdot);
% Jac_O_x_dot = simplify(jacobian(Jac_O(1), q) * qdot);
% Jac_O_y_dot = simplify(jacobian(Jac_O(2), q) * qdot);
% 
% Jac_A_x_dot_fn = matlabFunction(Jac_A_x_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_A_y_dot_fn = matlabFunction(Jac_A_y_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_B_x_dot_fn = matlabFunction(Jac_B_x_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_B_y_dot_fn = matlabFunction(Jac_B_y_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_C_x_dot_fn = matlabFunction(Jac_C_x_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_C_y_dot_fn = matlabFunction(Jac_C_y_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_D_x_dot_fn = matlabFunction(Jac_D_x_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_D_y_dot_fn = matlabFunction(Jac_D_y_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_M_x_dot_fn = matlabFunction(Jac_M_x_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_M_y_dot_fn = matlabFunction(Jac_M_y_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_O_x_dot_fn = matlabFunction(Jac_O_x_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)
% Jac_O_y_dot_fn = matlabFunction(Jac_O_y_dot); %@(q4,q5,q6,q4dot,q5dot,q6dot,thetadot,theta)

D_q_fn = matlabFunction(D_q);%@(q2,q3,q4,q5,q7,q8,q9,q10,theta)
B_q_fn = matlabFunction(B_q);
C_q_qdot_fn = matlabFunction(C_q_qdot);%@(q2,q3,q4,q5,q7,q8,q9,q10,q10dot,q2dot,q3dot,q4dot,q5dot,q7dot,q8dot,q9dot,thetadot,theta)
% Phi_Eqn_fn = matlabFunction(Phi_Eqn);%(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,rAx,rAy,rBx,rBy,rCx,rCy,rDx,rDy,rIx,rIy,rMx,rMy,rOx,rOy,theta)
% Jac_Full_fn = matlabFunction(Jac_Full);%(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta)

P.AngxIK_fn = matlabFunction(AngxIK);%@(theta)
P.AngxIH_fn = matlabFunction(AngxIH);%@(q3,theta)
P.AngxIF_fn = matlabFunction(AngxIF);%@(q4,theta)
P.AngIFx_fn = matlabFunction(AngIFx);%@(q4,theta)
P.AngIHx_fn = matlabFunction(AngIHx);%@(q3,theta)
P.AngxFE_fn = matlabFunction(AngxFE);%@(q4,q5,theta)
P.AngFEx_fn = matlabFunction(AngFEx);%@(q4,q5,theta)
P.AngxBA_fn = matlabFunction(AngxBA);%@(q4,q5,q6,theta)
P.AngxHG_fn = matlabFunction(AngxHG);%@(q2,q3,theta)
P.AngHGx_fn = matlabFunction(AngHGx);%@(q2,q3,theta)
P.AngxDC_fn = matlabFunction(AngxDC);%@(q1,q2,q3,theta)
P.AngxJL_fn = matlabFunction(AngxJL);%@(q9,theta)
P.AngJLx_fn = matlabFunction(AngJLx);%@(q9,theta)
P.AngMLx_fn = matlabFunction(AngMLx);%@(q9,q10,theta)
P.AngxJN_fn = matlabFunction(AngxJN);%@(q7,theta)
P.AngJNx_fn = matlabFunction(AngJNx);%@(q7,theta)
P.AngxNO_fn = matlabFunction(AngxNO);%@(q7,q8,theta)
P.AngNOx_fn = matlabFunction(AngNOx);%@(q7,q8,theta)

rA_fn = matlabFunction(rA);%@(q4,q5,q6,rIx,rIy,theta)
rB_fn = matlabFunction(rB);%@(q4,q5,q6,rIx,rIy,theta)
rC_fn = matlabFunction(rC);%@(q1,q2,q3,rIx,rIy,theta)
rD_fn = matlabFunction(rD);%@(q1,q2,q3,rIx,rIy,theta)
rE_fn = matlabFunction(rE);%@(q4,q5,rIx,rIy,theta)
rF_fn = matlabFunction(rF);%@(q4,rIx,rIy,theta)
rG_fn = matlabFunction(rG);%@(q2,q3,rIx,rIy,theta)
rH_fn = matlabFunction(rH);%@(q3,rIx,rIy,theta)
rI_fn = matlabFunction(rI);%@(rIx,rIy)
rJ_fn = matlabFunction(rJ);%@(rIx,rIy,theta)
rK_fn = matlabFunction(rK);%@(rIx,rIy,theta)
rL_fn = matlabFunction(rL);%@(q9,rIx,rIy,theta)
rM_fn = matlabFunction(rM);%@(q9,q10,rIx,rIy,theta)
rN_fn = matlabFunction(rN);%@(q7,rIx,rIy,theta)
rO_fn = matlabFunction(rO);%@(q7,q8,rIx,rIy,theta)

P.vA_fn = matlabFunction(vA);  %@(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta)
P.vB_fn = matlabFunction(vB);  %@(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta)
P.vC_fn = matlabFunction(vC);  %@(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta)
P.vD_fn = matlabFunction(vD);  %@(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta)
P.vE_fn = matlabFunction(vE);  %@(q4,q5,q4dot,q5dot,rIxdot,rIydot,thetadot,theta)
P.vF_fn = matlabFunction(vF);  %@(q4,q4dot,rIxdot,rIydot,thetadot,theta)
P.vG_fn = matlabFunction(vG);  %@(q2,q3,q2dot,q3dot,rIxdot,rIydot,thetadot,theta)
P.vH_fn = matlabFunction(vH);  %@(q3,q3dot,rIxdot,rIydot,thetadot,theta)
P.vI_fn = matlabFunction(vI);  %@(rIxdot,rIydot)
P.vJ_fn = matlabFunction(vJ);  %@(rIxdot,rIydot,thetadot,theta)
P.vK_fn = matlabFunction(vK);  %@(rIxdot,rIydot,thetadot,theta)
P.vL_fn = matlabFunction(vL);  %@(q9,q9dot,rIxdot,rIydot,thetadot,theta)
P.vM_fn = matlabFunction(vM);  %@(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta)
P.vN_fn = matlabFunction(vN);  %@(q7,q7dot,rIxdot,rIydot,thetadot,theta)
P.vO_fn = matlabFunction(vO);  %@(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta)

P.D_q_fn = D_q_fn;
P.B_q_fn = B_q_fn;
P.C_q_qdot_fn = C_q_qdot_fn;

P.rA_fn = rA_fn;
P.rB_fn = rB_fn;
P.rC_fn = rC_fn;
P.rD_fn = rD_fn;
P.rE_fn = rE_fn;
P.rF_fn = rF_fn;
P.rG_fn = rG_fn;
P.rH_fn = rH_fn;
P.rI_fn = rI_fn;
P.rJ_fn = rJ_fn;
P.rK_fn = rK_fn;
P.rL_fn = rL_fn;
P.rM_fn = rM_fn;
P.rN_fn = rN_fn;
P.rO_fn = rO_fn;

save('Pre_Load_Structure.mat','P');
save('Symbolic_Structure.mat','Q');

end
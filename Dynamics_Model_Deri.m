function Dynamics_Model_Deri()

% This function is used to generate the equation of motion for the 11-link
% humanoid robot with a different point of view to treat the 
% The item sequence is [foot, shank, thigh, body ,head, forearm, arm]
p = Humanoid_Physical_Parameter();
g = 9.81;

P.mg = p.m_tot * g;

syms rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 real
syms rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot real
syms rIxddot rIyddot thetaddot q1ddot q2ddot q3ddot q4ddot q5ddot q6ddot q7ddot q8ddot real
syms u1 u2 u3 u4 u5 u6 u7 u8 real

AngxIK = theta; 
AngxIJ = AngxIK;
AngxIE = -(pi - theta + q1);
AngxEA = AngxIE - q2;
AngxIF = -(pi - theta - q3);
AngxFB = AngxIF - q4;
AngxJG = -(pi - theta - q5);
AngxGC = AngxJG +q6;
AngxJH = -(pi - theta + q7);
AngxHD = AngxJH + q8;

l_shank = p.l(1);       l_thigh = p.l(2);       l_body = p.l(3);        l_arm = p.l(4);         l_forearm = p.l(5);     
    
KJ2JI = p.KJ2JI;

rI = [rIx, rIy]';

rIF = l_thigh * lamdadirection(AngxIF);
rF = rI + rIF;
rFB = l_shank * lamdadirection(AngxFB);
rB = rF + rFB;

rIE = l_thigh * lamdadirection(AngxIE);
rE = rI + rIE;
rEA = l_shank * lamdadirection(AngxEA);
rA = rE + rEA;

rIJ = l_body * lamdadirection(AngxIJ);
rJ = rI + rIJ;
rIK = (1 + KJ2JI) * l_body * lamdadirection(AngxIJ);
rK = rI + rIK;

rJH = l_arm * lamdadirection(AngxJH);
rH = rJ + rJH;
rHD = l_forearm * lamdadirection(AngxHD);
rD = rH + rHD;

rJG = l_arm * lamdadirection(AngxJG);
rG = rJ + rJG;
rGC = l_forearm * lamdadirection(AngxGC);
rC = rG + rGC;

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

r_vec = [rA;  rB;  rC;  rD;  rE;  rF;  rG;  rH;  rI;  rJ;  rK];

% This part is the generalized coordinate of the robots
q =     [rIx,     rIy,     theta,     q1,     q2,     q3,     q4,     q5,     q6,     q7,     q8]';
qdot =  [rIxdot,  rIydot,  thetadot,  q1dot,  q2dot,  q3dot,  q4dot,  q5dot,  q6dot,  q7dot,  q8dot]';
qddot = [rIxddot, rIyddot, thetaddot, q1ddot, q2ddot, q3ddot, q4ddot, q5ddot, q6ddot, q7ddot, q8ddot]';

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

v_vec = [vA; vB; vC; vD; vE; vF; vG; vH; vI; vJ];

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

[T_EA, V_EA] = Kinematics_Cal(AngxEA, 'AngxEA', q, qdot, r_vec, v_vec, p);
[T_FB, V_FB] = Kinematics_Cal(AngxFB, 'AngxFB', q, qdot, r_vec, v_vec, p);
[T_IE, V_IE] = Kinematics_Cal(AngxIE, 'AngxIE', q, qdot, r_vec, v_vec, p);
[T_IF, V_IF] = Kinematics_Cal(AngxIF, 'AngxIF', q, qdot, r_vec, v_vec, p);
[T_IJ, V_IJ] = Kinematics_Cal(AngxIJ, 'AngxIJ', q, qdot, r_vec, v_vec, p);
[T_JH, V_JH] = Kinematics_Cal(AngxJH, 'AngxJH', q, qdot, r_vec, v_vec, p);
[T_HD, V_HD] = Kinematics_Cal(AngxHD, 'AngxHD', q, qdot, r_vec, v_vec, p);
[T_JG, V_JG] = Kinematics_Cal(AngxJG, 'AngxJG', q, qdot, r_vec, v_vec, p);
[T_GC, V_GC] = Kinematics_Cal(AngxGC, 'AngxGC', q, qdot, r_vec, v_vec, p);

T = T_EA + T_FB + T_IE + T_IF + T_IJ + T_JH + T_HD + T_JG + T_GC;
V = V_EA + V_FB + V_IE + V_IF + V_IJ + V_JH + V_HD + V_JG + V_GC;

T = simplify(T);
V = simplify(V);
T_fn = matlabFunction(T);
V_fn = matlabFunction(V);

P.T_fn = T_fn;
P.V_fn = V_fn;
Q.T = T;
Q.V = V; 

u = [u1 u2 u3 u4 u5 u6 u7 u8]'; 

L = T - V;
pL_pq = simplify(jacobian(L, q));
pL_pqdot = simplify(jacobian(L,qdot));
d_pL_pqdot_dt = simplify(jacobian(pL_pqdot, [q; qdot]) * [qdot; qddot]);

Eqn = d_pL_pqdot_dt - pL_pq.' - [0; 0; 0; u];
Eqn = simplify(Eqn);
D_q = jacobian(Eqn, qddot);
B_q = -jacobian(Eqn, u);
C_q_qdot = simplify(Eqn - D_q * qddot + B_q * u);

D_q_fn = matlabFunction(D_q);%@(q1,q2,q3,q4,q5,q6,q7,q8,theta)
B_q_fn = matlabFunction(B_q);
C_q_qdot_fn = matlabFunction(C_q_qdot);%@(q1,q2,q3,q4,q5,q6,q7,q8,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,thetadot,theta)

P.D_q_fn = D_q_fn;
P.B_q_fn = B_q_fn;
P.C_q_qdot_fn = C_q_qdot_fn;

Q.D_q = D_q;
Q.B_q = B_q;
Q.C_q_qdot = C_q_qdot;

% Now let us come to the computation of the full dynamics constraint jacobian matrix

syms rAx rAy rBx rBy rCx rCy rDx rDy real
Phi_A = [rAx; rAy] - rA;
Phi_B = [rBx; rBy] - rB;
Phi_C = [rCx; rCy] - rC;
Phi_D = [rDx; rDy] - rD;

Phi_Eqn = [-Phi_A; -Phi_B; -Phi_C; -Phi_D];
Jac_Full = jacobian(Phi_Eqn, q);
P.Jac_Full_fn = matlabFunction(Jac_Full); %@(q1,q2,q3,q4,q5,q6,q7,q8,theta)
Q.Jac_Full = Jac_Full;

Jac_Full_dot_Eqn = simplify(jacobian(Jac_Full * qdot, [q;qdot]) * [qdot;qddot]);
Jacdot_qdot = simplify(Jac_Full_dot_Eqn - Jac_Full * qddot);

P.Jacdot_qdot_fn = matlabFunction(Jacdot_qdot); %@(q1,q2,q3,q4,q5,q6,q7,q8,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,thetadot,theta)

Q.Jac_Full_dot_Eqn = Jac_Full_dot_Eqn; 
Q.Jacdot_qdot = Jacdot_qdot;

P.rA_fn = matlabFunction(rA);%@(q1,q2,rIx,rIy,theta)
P.rB_fn = matlabFunction(rB);%@(q3,q4,rIx,rIy,theta)
P.rC_fn = matlabFunction(rC);%@(q5,q6,rIx,rIy,theta)
P.rD_fn = matlabFunction(rD);%@(q7,q8,rIx,rIy,theta)
P.rE_fn = matlabFunction(rE);%@(q1,rIx,rIy,theta)
P.rF_fn = matlabFunction(rF);%@(q3,rIx,rIy,theta)
P.rG_fn = matlabFunction(rG);%@(q5,rIx,rIy,theta)
P.rH_fn = matlabFunction(rH);%@(q7,rIx,rIy,theta)
P.rI_fn = matlabFunction(rI);%@(rIx,rIy)
P.rJ_fn = matlabFunction(rJ);%@(rIx,rIy,theta)
P.rK_fn = matlabFunction(rK);%@(rIx,rIy,theta)

P.vA_fn = matlabFunction(vA);  %@(q1,q2,q1dot,q2dot,rIxdot,rIydot,thetadot,theta)
P.vB_fn = matlabFunction(vB);  %@(q3,q4,q3dot,q4dot,rIxdot,rIydot,thetadot,theta)
P.vC_fn = matlabFunction(vC);  %@(q5,q6,q5dot,q6dot,rIxdot,rIydot,thetadot,theta)
P.vD_fn = matlabFunction(vD);  %@(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta)
P.vE_fn = matlabFunction(vE);  %@(q1,q1dot,rIxdot,rIydot,thetadot,theta)
P.vF_fn = matlabFunction(vF);  %@(q3,q3dot,rIxdot,rIydot,thetadot,theta)
P.vG_fn = matlabFunction(vG);  %@(q5,q5dot,rIxdot,rIydot,thetadot,theta)
P.vH_fn = matlabFunction(vH);  %@(q7,q7dot,rIxdot,rIydot,thetadot,theta)
P.vI_fn = matlabFunction(vI);  %@(rIxdot,rIydot)
P.vJ_fn = matlabFunction(vJ);  %@(rIxdot,rIydot,thetadot,theta)

save('Pre_Load_Structure.mat','P');
save('Symbolic_Structure.mat','Q');
end

function [T_i, V_i] = Kinematics_Cal(Angxi, Angxi_name, q, qdot, r_vec, v_vec, p)
% This function is used to calculate the kinematics of a given link

m_shank = p.m(1);       m_thigh = p.m(2);       m_body = p.m(3);        m_arm = p.m(4);         m_forearm = p.m(5);     
I_shank = p.I(1);       I_thigh = p.I(2);       I_body = p.I(3);        I_arm = p.I(4);         I_forearm = p.I(5); 

rA = r_vec(1:2,:);
rB = r_vec(3:4,:);
rC = r_vec(5:6,:);
rD = r_vec(7:8,:);
rE = r_vec(9:10,:);
rF = r_vec(11:12,:);
rG = r_vec(13:14,:);
rH = r_vec(15:16,:);
rI = r_vec(17:18,:);
rJ = r_vec(19:20,:);

vA = v_vec(1:2,:); 
vB = v_vec(3:4,:); 
vC = v_vec(5:6,:); 
vD = v_vec(7:8,:); 
vE = v_vec(9:10,:); 
vF = v_vec(11:12,:); 
vG = v_vec(13:14,:); 
vH = v_vec(15:16,:); 
vI = v_vec(17:18,:); 
vJ = v_vec(19:20,:); 

mIJ = m_body;
mAE = m_shank;          mBF = mAE;          mIE = m_thigh;          mIF = mIE;
mJG = m_arm;            mJH = mJG;          mGC = m_forearm;        mHD = mGC;

IIJ = I_body;
IAE = I_shank;          IBF = IAE;          IIE = I_thigh;          IIF = IIE;
IJG = I_arm;            IJH = IJG;          IGC = I_forearm;        IHD = IGC;

AngRatexi = jacobian(Angxi, q) * qdot;
i = 1;
while(i<length(Angxi_name))
    Angxi_name_i = Angxi_name(i);
    if Angxi_name_i=='x'
        r1 = Angxi_name(i + 1);
        r2 = Angxi_name(i + 2);
        break;
    end
    i = i + 1;
end

% Then it is to find the right name for this link
temp_name = strcat(r1,r2);
mass_temp_name = strcat('m',temp_name);
if (exist(mass_temp_name,'var') == 0)
    temp_name = strcat(r2,r1);
end

mass_name = strcat('m',temp_name);
Inertia_name = strcat('I',temp_name);
Edge_pos_1 = strcat('r',r1);
Edge_pos_2 = strcat('r',r2);
Edge_vel_1 = strcat('v',r1);
Edge_vel_2 = strcat('v',r2);


evalc(['link_mass = ' mass_name]);
evalc(['link_inertia = ' Inertia_name]);
evalc(['r_pos_1 = ' Edge_pos_1]);
evalc(['r_pos_2 = ' Edge_pos_2]);

evalc(['r_vel_1 = ' Edge_vel_1]);
evalc(['r_vel_2 = ' Edge_vel_2]);


T_i = 1/2 * link_mass * dot(0.5 * (r_vel_1 + r_vel_2), 0.5 * (r_vel_1 + r_vel_2)) + 1/2 * link_inertia * dot(AngRatexi,AngRatexi);
V_i = link_mass * 9.81 * (r_pos_1(2) + r_pos_2(2))/2;

end

function lamda_vec = lamdadirection(theta)

% This function is used to generate the unit direction vector under a given
% angle

% This angle is computed with respect to the horizontal line and the
% counterclockwise direction is the positive direction

lamda_vec = [cos(theta), sin(theta)].';

end
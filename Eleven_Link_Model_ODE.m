function zdot = Eleven_Link_Model_ODE(t,z,P)
% This function is the right hand side function of the ode integration
global contact_status;     % contact_status is a 6 by 2 vector

% The input state variable
rIx = z(1);
rIy = z(2);
theta = z(3);
q1 = z(4);
q2 = z(5);
q3 = z(6);
q4 = z(7);
q5 = z(8);
q6 = z(9);
q7 = z(10);
q8 = z(11);
q9 = z(12);
q10 = z(13);

rIxdot = z(14);
rIydot = z(15);
thetadot = z(16);
q1dot = z(17);
q2dot = z(18);
q3dot = z(19);
q4dot = z(20);
q5dot = z(21);
q6dot = z(22);
q7dot = z(23);
q8dot = z(24);
q9dot = z(25);
q10dot = z(26);

u = P.u;

D_q_fn = P.D_q_fn;
B_q_fn = P.B_q_fn;
C_q_qdot_fn = P.C_q_qdot_fn;

D_q = D_q_fn(q2,q3,q4,q5,q7,q8,q9,q10,theta);
B_q = B_q_fn();
C_q_qdot = C_q_qdot_fn(q2,q3,q4,q5,q7,q8,q9,q10,q10dot,q2dot,q3dot,q4dot,q5dot,q7dot,q8dot,q9dot,thetadot,theta);

rA_fn = P.rA_fn;
rB_fn = P.rB_fn;
rC_fn = P.rC_fn;
rD_fn = P.rD_fn;
% rE_fn = P.rE_fn;
% rF_fn = P.rF_fn;
% rG_fn = P.rG_fn;
% rH_fn = P.rH_fn;
% rI_fn = P.rI_fn;
% rJ_fn = P.rJ_fn;
% rK_fn = P.rK_fn;
% rL_fn = P.rL_fn;
rM_fn = P.rM_fn;
% rN_fn = P.rN_fn;
rO_fn = P.rO_fn;

rA = rA_fn(q4,q5,q6,rIx,rIy,theta);
rB = rB_fn(q4,q5,q6,rIx,rIy,theta);
rC = rC_fn(q1,q2,q3,rIx,rIy,theta);
rD = rD_fn(q1,q2,q3,rIx,rIy,theta);
% rE = rE_fn(q4,q5,rIx,rIy,theta);
% rF = rF_fn(q4,rIx,rIy,theta);
% rG = rG_fn(q2,q3,rIx,rIy,theta);
% rH = rH_fn(q3,rIx,rIy,theta);
% rI = rI_fn(rIx,rIy);
% rJ = rJ_fn(rIx,rIy,theta);
% rK = rK_fn(rIx,rIy,theta);
% rL = rL_fn(q9,rIx,rIy,theta);
rM = rM_fn(q9,q10,rIx,rIy,theta);
% rN = rN_fn(q7,rIx,rIy,theta);
rO = rO_fn(q7,q8,rIx,rIy,theta);

% vA_fn = P.vA_fn;
% vB_fn = P.vB_fn;
% vC_fn = P.vC_fn;
% vD_fn = P.vD_fn;
% vE_fn = P.E_fn;
% vF_fn = P.vF_fn;
% vG_fn = P.vG_fn;
% vH_fn = P.vH_fn;
% vI_fn = P.vI_fn;
% vJ_fn = P.vJ_fn;
% vK_fn = P.vK_fn;
% vL_fn = P.vL_fn;
% vM_fn = P.vM_fn;
% vN_fn = P.vN_fn;
% vO_fn = P.vO_fn;

vA = P.vA_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vB = P.vB_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vC = P.vC_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vD = P.vD_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
% vE = P.vE_fn(q4,q5,q4dot,q5dot,rIxdot,rIydot,thetadot,theta);
% vF = P.vF_fn(q4,q4dot,rIxdot,rIydot,thetadot,theta);
% vG = P.vG_fn(q2,q3,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
% vH = P.vH_fn(q3,q3dot,rIxdot,rIydot,thetadot,theta);
% vI = P.vI_fn(rIxdot,rIydot);
% vJ = P.vJ_fn(rIxdot,rIydot,thetadot,theta);
% vK = P.vK_fn(rIxdot,rIydot,thetadot,theta);
% vL = P.vL_fn(q9,q9dot,rIxdot,rIydot,thetadot,theta);
vM = P.vM_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
% vN = P.vN_fn(q7,q7dot,rIxdot,rIydot,thetadot,theta);
vO = P.vO_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);

% Jac_A_fn = P.Jac_A_fn;%@(q4,q5,q6,theta)
% Jac_B_fn = P.Jac_B_fn;%@(q4,q5,q6,theta)
% Jac_C_fn = P.Jac_C_fn;%@(q1,q2,q3,theta)
% Jac_D_fn = P.Jac_D_fn;%@(q1,q2,q3,theta)
% Jac_M_fn = P.Jac_M_fn;%@(q9,q10,theta)
% Jac_O_fn = P.Jac_O_fn;%@(q7,q8,theta)
%
% Jac_A = Jac_A_fn(q4,q5,q6,theta);
% Jac_B = Jac_B_fn(q4,q5,q6,theta);
% Jac_C = Jac_C_fn(q1,q2,q3,theta);
% Jac_D = Jac_D_fn(q1,q2,q3,theta);
% Jac_M = Jac_M_fn(q9,q10,theta);
% Jac_O = Jac_O_fn(q7,q8,theta);

Jac_Full_fn = P.Jac_Full_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta)
Jac_Full = Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
Jacdot_qdot_fn = P.Jacdot_qdot_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta)
Jacdot_qdot = Jacdot_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);

% Based on the different mode status, there would be a modification of the
% equation of motion.

% The mode status can be set to three different state: separation: 0, sliding: 0.5 and sticking: 1.
% As a result, the first step to do is to conduct an analysis of how the
% mode status at the current integration time.

% In this problem, there are six contact points:
Jac_t = [];                          % This is the real jacobian involved in the current integration
Jacdot_qdot_t = [];                  % This is the term use for the computation of the contact force
Jac_depend_t = [];                   % This is the independent jacobian matrix
Jac_independ_index_t = [];           % This is the index that the independent force
Jac_independ_index_in_Jac_t = [];    % This is the index of the independent force with respect to the whole lamda vector
Jac_depend_index_t = [];             % This is the index of the dependent force
for i = 1:6
    contact_status_i = contact_status(i,:);   % There are three possibilities    
    % In this case, the current contact point is in "sticking" condition so we add them into our Jac_t
    if (max(contact_status_i) == min(contact_status_i))&&( min(contact_status_i)== 1)
        Jac_t = [Jac_t; Jac_Full(2*i-1:2*i,:)];
        Jacdot_qdot_t = [Jacdot_qdot_t; Jacdot_qdot(2*i-1:2*i,:)];
        Jac_independ_index_t = [Jac_independ_index_t; i;i];
    end
    % In this case, the current contact point is in "separation" condition so we neglect them
    if (max(contact_status_i) == min(contact_status_i))&&( min(contact_status_i)== 0)
        continue;
    end
    % In this case, the current contact point is in "sliding" condition so
    % we add one term and compute the other with the friction coefficient
    if (max(contact_status_i) == 1)&&(min(contact_status_i) == 0.5)
        [~,contact_sticking_index] = max(contact_status_i);  % This is used to find out which direction remains a holonomic constraint
        if  contact_sticking_index == 1  % In this case, the x position remains fixed so the end effector is sliding along the y axis
            Jac_t = [Jac_t; Jac_Full(2*i-1,:)];
            Jacdot_qdot_t = [Jacdot_qdot_t; Jacdot_qdot(2*i-1,:)];
            Jac_independ_index_t = [Jac_independ_index_t; 2*i - 1];
            Jac_depend_index_t = [Jac_depend_index_t; 2*i];
            Jac_depend_t = [Jac_depend_t; Jac_Full(2*i,:)];
            Jac_independ_index_in_Jac_t = [Jac_independ_index_in_Jac_t; length(Jac_independ_index_t)];
        else
            Jac_t = [Jac_t; Jac_Full(2*i,:)];
            Jacdot_qdot_t = [Jacdot_qdot_t; Jacdot_qdot(2*i,:)];
            Jac_independ_index_t = [Jac_independ_index_t; 2*i];
            Jac_depend_index_t = [Jac_depend_index_t; 2*i-1];
            Jac_depend_t = [Jac_depend_t; Jac_Full(2*i-1,:)];
            Jac_independ_index_in_Jac_t = [Jac_independ_index_in_Jac_t; length(Jac_independ_index_t)];
        end
    end
end

[Full_rank_Jac_t, Full_rank_Jacdot_qdot_t,~, ~, Jac_independ_index_t, Jac_independ_index_in_Jac_t]= Over_Constraint_Simplify(Jac_t, Jacdot_qdot_t, Jac_independ_index_t, Jac_independ_index_in_Jac_t);

Jac_t         = Full_rank_Jac_t;
Jacdot_qdot_t = Full_rank_Jacdot_qdot_t;
% Now it is the computation of the contact force
if isempty(Jac_depend_index_t) == 0    % So there exists at least one dependent term
    contact_velocity_name = cellstr(P.contact_velocity_name);
    M_t = zeros(length(Jac_depend_index_t), length(Jacdot_qdot_t));   % In this case, this M matrix would be a wide matrix with rows longer than columns
    for i = 1:length(Jac_depend_index_t) 
        v_independ_t = eval(contact_velocity_name{floor(Jac_independ_index_t(Jac_independ_index_in_Jac_t(i))/2)});
        v_independ_tangential_index_t = mod(Jac_independ_index_t(Jac_independ_index_in_Jac_t(i)),2);
        if v_independ_tangential_index_t == 0
            v_independ_tangential_t = v_independ_t(1);
        else
            v_independ_tangential_t = v_independ_t(2);
        end
        
        M_t(i, Jac_independ_index_in_Jac_t(i)) = - P.mu * sign(v_independ_tangential_t);
    end
    Jac_E = Jac_t + M_t' * Jac_depend_t;
    lamda_I = (Jac_t * D_q^(-1) * Jac_E')\(Jac_t * D_q^(-1) * C_q_qdot - Jacdot_qdot_t - Jac_t * D_q^(-1) * B_q * u);
    J_T_lamda_t = Jac_E' * lamda_I;
else
    if isempty(Jac_t)==1
        J_T_lamda_t = zeros(length(D_q), length(z)/2);
    else
        [Jac_t, Jacdot_qdot_t ]= Over_Constraint_Simplify(Jac_t, Jacdot_qdot_t);
        lamda_t = (Jac_t * D_q^(-1) * Jac_t')\(Jac_t * D_q^(-1) * C_q_qdot - Jacdot_qdot_t - Jac_t * D_q^(-1) * B_q * u);
        J_T_lamda_t = Jac_t' * lamda_t;
    end
    
end
qddot = -D_q\C_q_qdot + D_q\(J_T_lamda_t) + D_q\B_q * u;
zdot = [z(14:26,:); qddot];
end
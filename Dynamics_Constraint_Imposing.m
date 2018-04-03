function [c, ceq] = Dynamics_Constraint_Imposing(Contact_Force_Array, Q_Qdot_Array, Control_Torque_Array, P)

% This function is used to impose the dynamics constraint into the
% optimization problem

% The acceleration is estimated through an numerical differential method

% For each time,there is a dynamic constraint

D_q_fn = P.D_q_fn;
B_q_fn = P.B_q_fn;
C_q_qdot_fn = P.C_q_qdot_fn;

for i = 1:P.N
    
    Contact_Force_t  = Contact_Force_Array(:,i);
    
    Q_Qdot_t         = Q_Qdot_Array(:,i);
    
    Control_Torque_t = Control_Torque_Array(:,i);
    
    % The input state variable
    rIx = Q_Qdot_t(1);
    rIy = Q_Qdot_t(2);
    theta = Q_Qdot_t(3);
    q1 = Q_Qdot_t(4);
    q2 = Q_Qdot_t(5);
    q3 = Q_Qdot_t(6);
    q4 = Q_Qdot_t(7);
    q5 = Q_Qdot_t(8);
    q6 = Q_Qdot_t(9);
    q7 = Q_Qdot_t(10);
    q8 = Q_Qdot_t(11);
    q9 = Q_Qdot_t(12);
    q10 = Q_Qdot_t(13);
    
    rIxdot = Q_Qdot_t(14);
    rIydot = Q_Qdot_t(15);
    thetadot = Q_Qdot_t(16);
    q1dot = Q_Qdot_t(17);
    q2dot = Q_Qdot_t(18);
    q3dot = Q_Qdot_t(19);
    q4dot = Q_Qdot_t(20);
    q5dot = Q_Qdot_t(21);
    q6dot = Q_Qdot_t(22);
    q7dot = Q_Qdot_t(23);
    q8dot = Q_Qdot_t(24);
    q9dot = Q_Qdot_t(25);
    q10dot = Q_Qdot_t(26);
    
    u = Control_Torque_t;
    
    D_q = D_q_fn(q2,q3,q4,q5,q7,q8,q9,q10,theta);
    B_q = B_q_fn();
    C_q_qdot = C_q_qdot_fn(q2,q3,q4,q5,q7,q8,q9,q10,q10dot,q2dot,q3dot,q4dot,q5dot,q7dot,q8dot,q9dot,thetadot,theta);
    
    Jac_Full_fn = P.Jac_Full_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta)
    Jac_Full = Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
    
    
end
end


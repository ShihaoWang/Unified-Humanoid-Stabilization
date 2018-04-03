function [ c,ceq ] = Contact_Constraint_Cal( c,ceq, P, initial_mode_status, initial_state, Trans_States, All_Grids, Contact_Forces, Control_Torque_T_N)
% This function is used to add the equality and inequality constraint to
% the c and ceq where mode_status_ref is the reference mode status to
% compared to. Usually it is the first mode of the segment.

% INPUTS:
% c                                         the original inequality constraints
% ceq:                                      the equality constraints
% P:                                        handle structure
% initial_mode_status:  12 * 1              the mode status at the instant of the push
% initial_state:        26 * 1              the robot state at the instant of the push
% Trans_States:         13 * 3 * (N-1)      the transition states during the whole process
% All_Grids:            39 * n * N          the position, velocity and acceralation of at each grid point for the whole segments
% Contact_Forces:       12 * n * N          the contact forces to be optimized at the end effector

% OUTPUTS: the updated c and ceq

D_q_fn = P.D_q_fn;
B_q_fn = P.B_q_fn;
C_q_qdot_fn = P.C_q_qdot_fn;
%

rA_fn = P.rA_fn;
rB_fn = P.rB_fn;
rC_fn = P.rC_fn;
rD_fn = P.rD_fn;
rE_fn = P.rE_fn;
rF_fn = P.rF_fn;
rG_fn = P.rG_fn;
rH_fn = P.rH_fn;
rI_fn = P.rI_fn;
rJ_fn = P.rJ_fn;
rK_fn = P.rK_fn;
rL_fn = P.rL_fn;
rM_fn = P.rM_fn;
rN_fn = P.rN_fn;
rO_fn = P.rO_fn;

%
% rA = rA_fn(q4,q5,q6,rIx,rIy,theta);
% rB = rB_fn(q4,q5,q6,rIx,rIy,theta);
% rC = rC_fn(q1,q2,q3,rIx,rIy,theta);
% rD = rD_fn(q1,q2,q3,rIx,rIy,theta);
% rM = rM_fn(q9,q10,rIx,rIy,theta);
% rO = rO_fn(q7,q8,rIx,rIy,theta);
%
% vA = P.vA_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
% vB = P.vB_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
% vC = P.vC_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
% vD = P.vD_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
%
% vM = P.vM_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
% vO = P.vO_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);
%
Jac_Full_fn = P.Jac_Full_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta)
% Jac_Full = Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
% Jacdot_qdot_fn = P.Jacdot_qdot_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta)
% Jacdot_qdot = Jacdot_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);
%
% P.rA = rA;
% P.rB = rB;
% P.rC = rC;
% P.rD = rD;
% P.rM = rM;
% P.rO = rO;
%
% P.vA = vA;
% P.vB = vB;
% P.vC = vC;
% P.vD = vD;
% P.vM = vM;
% P.vO = vO;


%% Here is to let the initial condition match with the one calculated from the b-spline
[Init_Const_Val, Init_Const_Index] = Contact_Status2_Constant_Val(initial_mode_status, initial_state, P);

[rIx, rIy, theta,...
    q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,...
    rIxdot, rIydot, thetadot,...
    q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot...
    ,rIxddot, rIyddot, thetaddot,...
    q1ddot,q2ddot,q3ddot,q4ddot,q5ddot,q6ddot,q7ddot,q8ddot,q9ddot,q10ddot] = State_Decomposition(All_Grids(:,1,1));

ceq = [ceq; rIx - initial_state(1)];
ceq = [ceq; rIy - initial_state(2)];
ceq = [ceq; theta - initial_state(3)];
ceq = [ceq; q1 - initial_state(4)];
ceq = [ceq; q2 - initial_state(5)];
ceq = [ceq; q3 - initial_state(6)];
ceq = [ceq; q4 - initial_state(7)];
ceq = [ceq; q5 - initial_state(8)];
ceq = [ceq; q6 - initial_state(9)];
ceq = [ceq; q7 - initial_state(10)];
ceq = [ceq; q8 - initial_state(11)];
ceq = [ceq; q9 - initial_state(12)];
ceq = [ceq; q10 - initial_state(13)];
ceq = [ceq; rIxdot - initial_state(14)];
ceq = [ceq; rIydot - initial_state(15)];
ceq = [ceq; thetadot - initial_state(16)];
ceq = [ceq; q1dot - initial_state(17)];
ceq = [ceq; q2dot - initial_state(18)];
ceq = [ceq; q3dot - initial_state(19)];
ceq = [ceq; q4dot - initial_state(20)];
ceq = [ceq; q5dot - initial_state(21)];
ceq = [ceq; q6dot - initial_state(22)];
ceq = [ceq; q7dot - initial_state(23)];
ceq = [ceq; q8dot - initial_state(24)];
ceq = [ceq; q9dot - initial_state(25)];
ceq = [ceq; q10dot - initial_state(26)];

%% The first task is to make sure that the contact status for each grid inside each segement is the same

% Due to the accessibility of the initial condition, the initial segment
% will be specifically taken care of.

% To know the holonomic constraint, we will find out the initial contact point.

for i = 1:P.N
    % i is the index for segment
    for j = 1:P.n
        % j is the index for which grid point within a segment
        [rIx, rIy, theta, q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,...
            rIxdot, rIydot, thetadot, q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,...
            rIxddot, rIyddot, thetaddot,...
            q1ddot,q2ddot,q3ddot,q4ddot,q5ddot,q6ddot,q7ddot,q8ddot,q9ddot,q10ddot] = State_Decomposition(All_Grids(:,j,i));
        
        Contact_Force_Seg_i_Grid_j =  Contact_Forces(:,j,i);
        
        State_Seg_i_Grid_j = [rIx, rIy, theta, q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,...
            rIxdot, rIydot, thetadot, q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot];
        Stateddot_Seg_i_Grid_j = [rIxddot, rIyddot, thetaddot,...
            q1ddot,q2ddot,q3ddot,q4ddot,q5ddot,q6ddot,q7ddot,q8ddot,q9ddot,q10ddot];
        
        % For each grid point inside a segment, there are four main
        % constraints to be satisfied.
        
        % 1. First is the contact maintaining constraint: Holonomic
        % constraints at the contant points
        
        % 2. The relative heights: Hip point is higher than knee, knee
        % is higher than ankle and feet
        
        % 3. The feasible force constraint D q'' + C + G = J^T lamda +
        % B * u
        
        % 4. The force and position needs to satisfy the
        % complementarity conditions
        
        % 1. Contact constraint
        [Contact_Seg_i_Grid_j_Vals, Contact_Seg_i_Grid_j_Index] = Contact_Status2_Constant_Val(Contact_Force_Seg_i_Grid_j, State_Seg_i_Grid_j, P);
        
        if i == 1;
            Pos_Const_Val = Init_Const_Val;
        end
        
        ceq = [ceq; Pos_Const_Val - Contact_Seg_i_Grid_j_Vals];
        
        % 2. Relative heights
        rA = rA_fn(q4,q5,q6,rIx,rIy,theta);
        
        rB = rB_fn(q4,q5,q6,rIx,rIy,theta);
        
        rC = rC_fn(q1,q2,q3,rIx,rIy,theta);
        
        rD = rD_fn(q1,q2,q3,rIx,rIy,theta);
        
        rE = rE_fn(q4,q5,rIx,rIy,theta);
        
        rF = rF_fn(q4,rIx,rIy,theta);
        
        rG = rG_fn(q2,q3,rIx,rIy,theta);
        
        rH = rH_fn(q3,rIx,rIy,theta);
        
        rI = rI_fn(rIx,rIy);
        
        rJ = rJ_fn(rIx,rIy,theta);
        
        rK = rK_fn(rIx,rIy,theta);
        
        rL = rL_fn(q9,rIx,rIy,theta);
        
        rM = rM_fn(q9,q10,rIx,rIy,theta);
        
        rN = rN_fn(q7,rIx,rIy,theta);
        
        rO = rO_fn(q7,q8,rIx,rIy,theta);
        
        % Basic constraint on heights
        c = [c; -rA(2); -rB(2); -rC(2); -rD(2);...
            -rE(2); -rF(2); -rG(2); -rH(2);...
            -rI(2); -rJ(2); -rK(2); -rL(2);...
            -rM(2);-rN(2);-rO(2);];
        
        % Relative constraint on heights
        
        eps = 0.1;
        
        c = [c; -rF(2) + rA(2) + eps;...
            -rF(2) + rB(2) + eps;...
            -rF(2) + rE(2) + eps;...
            -rI(2) + rF(2) + eps;...
            -rI(2) + rH(2) + eps;...
            -rH(2) + rG(2) + eps;...
            -rH(2) + rC(2) + eps;...
            -rH(2) + rD(2) + eps];
        
        % 3. Dynamics constraint on the maximum torque
        D_q = D_q_fn(q2,q3,q4,q5,q7,q8,q9,q10,theta);
        B_q = B_q_fn();
        C_q_qdot = C_q_qdot_fn(q2,q3,q4,q5,q7,q8,q9,q10,q10dot,q2dot,q3dot,q4dot,q5dot,q7dot,q8dot,q9dot,thetadot,theta);
        
        Jac_Full = Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
        
        Control_Torque_Seg_i_Grid_j = Control_Torque_T_N(:,j,i);
        
        ceq = [ceq; D_q * Stateddot_Seg_i_Grid_j' + C_q_qdot -  Jac_Full' * Contact_Force_Seg_i_Grid_j - B_q * Control_Torque_Seg_i_Grid_j];
        
        u_mag = (Control_Torque_Seg_i_Grid_j.*Control_Torque_Seg_i_Grid_j);
        
        u_max = (P.u_max)^2 * ones(10,1);
        
        c = [c; u_mag - u_max];
        
        % 4. Linear complementarity condition constraint
        rA_Dist = Relative_Dist_Cal(rA, P.Obs_Info);
        rB_Dist = Relative_Dist_Cal(rB, P.Obs_Info);
        rC_Dist = Relative_Dist_Cal(rC, P.Obs_Info);
        rD_Dist = Relative_Dist_Cal(rD, P.Obs_Info);
        rM_Dist = Relative_Dist_Cal(rM, P.Obs_Info);
        rO_Dist = Relative_Dist_Cal(rO, P.Obs_Info);
        
        ceq = [ceq; Contact_Force_Seg_i_Grid_j(1) * rA_Dist;...
            Contact_Force_Seg_i_Grid_j(2) * rA_Dist;...
            Contact_Force_Seg_i_Grid_j(3) * rB_Dist;...
            Contact_Force_Seg_i_Grid_j(4) * rB_Dist;...
            Contact_Force_Seg_i_Grid_j(5) * rC_Dist;...
            Contact_Force_Seg_i_Grid_j(6) * rC_Dist;...
            Contact_Force_Seg_i_Grid_j(7) * rD_Dist;...
            Contact_Force_Seg_i_Grid_j(8) * rD_Dist;...
            Contact_Force_Seg_i_Grid_j(9) * rM_Dist;...
            Contact_Force_Seg_i_Grid_j(10) * rM_Dist;...
            Contact_Force_Seg_i_Grid_j(11) * rO_Dist;...
            Contact_Force_Seg_i_Grid_j(12) * rO_Dist];
        
        
        Seg_i_State_j = All_Grids(:,j,i);
    end
end

% One more equality equation to be added is that the final contact mode
% should be the same as the prescribed one

% This constraint can be added as an inequality constraint
Contact_Force_Final = Contact_Force_Pattern_T_N(:,end);

% The four vertical contact forces must be positive
c = [c; -Contact_Force_Final(2) + eps ];
c = [c; -Contact_Force_Final(4) + eps ];
c = [c; -Contact_Force_Final(6) + eps ];
c = [c; -Contact_Force_Final(8) + eps ];

% While the other four contact forces should be zero
ceq = [ceq;Contact_Force_Final(9) ];
ceq = [ceq;Contact_Force_Final(10) ];
ceq = [ceq;Contact_Force_Final(11) ];
ceq = [ceq;Contact_Force_Final(12) ];
end


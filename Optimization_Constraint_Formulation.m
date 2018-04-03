function [c, ceq] = Optimization_Constraint_Formulation(Contact_Force_Array, Q_Qdot_Array, Control_Torque_Array, P)

% This function is used to impose the constraint into the optimization problem

% The acceleration is estimated through an numerical differential method

c = [];
ceq = [];

D_q_fn = P.D_q_fn;
B_q_fn = P.B_q_fn;
C_q_qdot_fn = P.C_q_qdot_fn;

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


Jac_Full_fn = P.Jac_Full_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta)
% Jacdot_qdot_fn = P.Jacdot_qdot_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta)

vA_fn = P.vA_fn;
vB_fn = P.vB_fn;
vC_fn = P.vC_fn;
vD_fn = P.vD_fn;
vM_fn = P.vM_fn;
vO_fn = P.vO_fn;

Contact_Position_Name = cellstr(P.contact_position_name);
Contact_Velocity_Name = cellstr(P.contact_velocity_name);

for i = 1:P.N
    
    %% 1. The first constraint to be added is the dynamic constraint
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
    
    Qdot_t = Q_Qdot_t(14:26,1);
    
    old_mode_q = Q_Qdot_t;
    
    if i == P.N
        c = [c; dot(Qdot_t, Qdot_t) - (P.eps)];    % To make sure that at the final stage the angular velocity is almost trivial
    end
    
    if i<P.N
        Qdot_tp1 = Q_Qdot_Array(14:26,i+1);
    else
        Qdot_tp1 = zeros(13,1);
    end
    
    u = Control_Torque_t;
    
    D_q = D_q_fn(q2,q3,q4,q5,q7,q8,q9,q10,theta);
    B_q = B_q_fn();
    C_q_qdot = C_q_qdot_fn(q2,q3,q4,q5,q7,q8,q9,q10,q10dot,q2dot,q3dot,q4dot,q5dot,q7dot,q8dot,q9dot,thetadot,theta);
    
    %     Jac_Full_fn = P.Jac_Full_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta)
    Jac_Full = Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
    
    ceq_Dynamics_Constraint = D_q * (Qdot_tp1 - Qdot_t)/P.dt + C_q_qdot - Jac_Full' * Contact_Force_t - B_q * u;
    
    ceq = [ceq; ceq_Dynamics_Constraint];
    
    %% 2. The second constraint is the vertical position should remain over the ground
    
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
    
    vA = vA_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
    vB = vB_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
    vC = vC_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
    vD = vD_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
    vM = vM_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
    vO = vO_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);
    
    c = [c; -rA(2);...
        -rB(2);...
        -rC(2);...
        -rD(2);...
        -rE(2);...
        -rF(2) + rA(2) + P.eps;...
        -rG(2);...
        -rH(2) + rC(2) + P.eps;...
        -rI(2) + rF(2) + P.eps;...
        -rI(2) + rH(2) + P.eps;...
        -rJ(2);...
        -rK(2);...
        -rL(2);...
        -rM(2);...
        -rN(2);...
        -rO(2)];
    
    %% 3. The third constraint is the linear complementarity condition of the contact force and position
    ceq = [ceq; Contact_Force_t(1)  * Relative_Dist_Cal(rA, P.Obs_Info);...
        Contact_Force_t(2)  * Relative_Dist_Cal(rA, P.Obs_Info);...
        Contact_Force_t(3)  * Relative_Dist_Cal(rB, P.Obs_Info);...
        Contact_Force_t(4)  * Relative_Dist_Cal(rB, P.Obs_Info);...
        Contact_Force_t(5)  * Relative_Dist_Cal(rC, P.Obs_Info);...
        Contact_Force_t(6)  * Relative_Dist_Cal(rC, P.Obs_Info);...
        Contact_Force_t(7)  * Relative_Dist_Cal(rD, P.Obs_Info);...
        Contact_Force_t(8)  * Relative_Dist_Cal(rD, P.Obs_Info);...
        Contact_Force_t(9)  * Relative_Dist_Cal(rM, P.Obs_Info);...
        Contact_Force_t(10) * Relative_Dist_Cal(rM, P.Obs_Info);...
        Contact_Force_t(11) * Relative_Dist_Cal(rO, P.Obs_Info);...
        Contact_Force_t(12) * Relative_Dist_Cal(rO, P.Obs_Info);...
        ];
    %% 4. The consistency constraint checking
    % The first trick is to make sure that the initial force equals to the first contact force element
    if i == 1
        Init_Contact_Force = P.Contact_Force_Init;
        ceq = [ceq; Contact_Force_t(1)  - Init_Contact_Force(1);...
            Contact_Force_t(2)  - Init_Contact_Force(2);...
            Contact_Force_t(3)  - Init_Contact_Force(3);...
            Contact_Force_t(4)  - Init_Contact_Force(4);...
            Contact_Force_t(5)  - Init_Contact_Force(5);...
            Contact_Force_t(6)  - Init_Contact_Force(6);...
            Contact_Force_t(7)  - Init_Contact_Force(7);...
            Contact_Force_t(8)  - Init_Contact_Force(8);...
            Contact_Force_t(9)  - Init_Contact_Force(9);...
            Contact_Force_t(10) - Init_Contact_Force(10);...
            Contact_Force_t(11) - Init_Contact_Force(11);...
            Contact_Force_t(12) - Init_Contact_Force(12);...
            ];
    end
    % While the last force follows the force pattern that we would like to
    % force to have
    if i == P.N
        ceq = [ceq; Contact_Force_t(1)  + Contact_Force_t(3) + Contact_Force_t(5) + Contact_Force_t(7);...
            Contact_Force_t(9)  ;...
            Contact_Force_t(10) ;...
            Contact_Force_t(11) ;...
            Contact_Force_t(12) ;...
            Contact_Force_t(2) + Contact_Force_t(4) + Contact_Force_t(6) + Contact_Force_t(8) - P.mg];
        
        c = [c;    -Contact_Force_t(2) + P.eps;...
            -Contact_Force_t(4) + P.eps;...
            -Contact_Force_t(6) + P.eps;...
            -Contact_Force_t(8) + P.eps];
    end
    
    % Then is to carry on the checking of contact consistency.
    % The basic assumption:
    % 1. The product of the elementarywise vector for sequential frame should be nonnegative.
    % 2. If a contact point is active during the sequential frame, then the
    % resulting constraint has to be satisfied.
    
    if i<P.N-1
        Contact_Force_tp1 = Contact_Force_Array(:,i+1);
        for m = 1:4
            c = [c; -Contact_Force_t(2*m) * Contact_Force_tp1(2*m)];
        end
        
        [Flag_t, Contact_Status_t] = Contact_Force2Holonomic_Constraint(Contact_Force_t);
        
        % Compute the reference Contact_Status_Val
        
        if Flag_t == 1
            Contact_Nonzero_Index_t = find(Contact_Status_t);
            Holo_t = zeros(6,2);      % The holonomic constraint for the position is saved into a lengthy matrix with the row indicating a coordinate
            for Holo_Index = 1:length(Contact_Nonzero_Index_t)
                Holo_t(Contact_Nonzero_Index_t(Holo_Index),:) = eval(Contact_Position_Name{Holo_Index})';
                ceq = [ceq; eval(Contact_Velocity_Name{Holo_Index})];
            end
        end
        
        [Flag_tp1, Contact_Status_tp1] = Contact_Force2Holonomic_Constraint(Contact_Force_tp1);
        
        if Flag_t * Flag_tp1==1   % This means that the sequential frames are all in contact with the environmental obstacle
            Q_Qdot_tp1 = Q_Qdot_Array(:,i+1);
            rIx = Q_Qdot_tp1(1);
            rIy = Q_Qdot_tp1(2);
            theta = Q_Qdot_tp1(3);
            q1 = Q_Qdot_tp1(4);
            q2 = Q_Qdot_tp1(5);
            q3 = Q_Qdot_tp1(6);
            q4 = Q_Qdot_tp1(7);
            q5 = Q_Qdot_tp1(8);
            q6 = Q_Qdot_tp1(9);
            q7 = Q_Qdot_tp1(10);
            q8 = Q_Qdot_tp1(11);
            q9 = Q_Qdot_tp1(12);
            q10 = Q_Qdot_tp1(13);
            
            rIxdot = Q_Qdot_tp1(14);
            rIydot = Q_Qdot_tp1(15);
            thetadot = Q_Qdot_tp1(16);
            q1dot = Q_Qdot_tp1(17);
            q2dot = Q_Qdot_tp1(18);
            q3dot = Q_Qdot_tp1(19);
            q4dot = Q_Qdot_tp1(20);
            q5dot = Q_Qdot_tp1(21);
            q6dot = Q_Qdot_tp1(22);
            q7dot = Q_Qdot_tp1(23);
            q8dot = Q_Qdot_tp1(24);
            q9dot = Q_Qdot_tp1(25);
            q10dot = Q_Qdot_tp1(26);
            
            % Then is to find the consistent constraint
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
            
            vA = vA_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
            vB = vB_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
            vC = vC_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
            vD = vD_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
            vM = vM_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
            vO = vO_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);
            
            Contact_Status_Comparison = Contact_Status_t .*Contact_Status_tp1;
            
            Contact_Status_Change = Contact_Status_tp1 - Contact_Status_t;
            
            if isempty(find(Contact_Status_Comparison, 1))==0  % So there exists at least one consistent contact constraint
                
                Contact_Consistent_Status = find(Contact_Status_Comparison);
                
                for l = 1:length(Contact_Consistent_Status)
                    
                    ceq = [ceq; eval(Contact_Position_Name{l}) - Holo_t(Contact_Consistent_Status(l),:)'];
                    
                    ceq = [ceq; eval(Contact_Velocity_Name{l})];              
                end
                
                if max(Contact_Status_Change) == 1 % In this case, there is an impact mapping happening to change the robot state                
                    % Then the idea is to make use of the consistent contact and the new added contact
                    [~, Max_Ind] = max(Contact_Status_Change);
                    % Here only one new added contact point will be considered
                             
                    Event_Index = 4 * (Max_Ind(1) -1) + 1;
                    
                    Contact_Index_Array = [Contact_Consistent_Status', Max_Ind(1)];
                    Contact_Status = Contact_Status_Formulation(Contact_Index_Array);
                    new_mode_q = Robot_State_Mode_Shaping(old_mode_q, Event_Index, Contact_Status, P);           
                    ceq = [ceq; new_mode_q - Q_Qdot_tp1];                 
                end  
                
            else
%                 
%                 % In this case, the contact transition is sharp. Then the
%                 % idea is to make sure that the contact status at the next
%                 % step is satisfied.
%                 
%                 Contact_Status_Change = Contact_Status_tp1 - Contact_Status_t;
%                 
%                 if max(Contact_Status_Change) == 1 % In this case, there is an impact mapping happening to change the robot state
%                     % Then the idea is to make use of the consistent contact and the new added contact
%                     [~, Max_Ind] = max(Contact_Status_Change);
%                     % Here only one new added contact point will be considered
%                     
%                     Event_Index = 4 * (Max_Ind -1) + 1;
%                     
%                     Contact_Index_Array = [Contact_Consistent_Status, Max_Ind];
%                     Contact_Status = Contact_Status_Formulation(Contact_Index_Array);
%                     new_mode_q = Robot_State_Mode_Shaping(old_mode_q, Event_Index, Contact_Status, P);           
%                     ceq = [ceq; new_mode_q - Q_Qdot_tp1];                 
%                 end  
%                 
                

            end
        else
            % In this case, there are two possibilities:
            % 1. First is that two frames are all in the air
            % 2. Second is that one frame is in contact while the other is in the air
            
            Q_Qdot_tp1 = Q_Qdot_Array(:,i+1);
            rIx = Q_Qdot_tp1(1);
            rIy = Q_Qdot_tp1(2);
            theta = Q_Qdot_tp1(3);
            q1 = Q_Qdot_tp1(4);
            q2 = Q_Qdot_tp1(5);
            q3 = Q_Qdot_tp1(6);
            q4 = Q_Qdot_tp1(7);
            q5 = Q_Qdot_tp1(8);
            q6 = Q_Qdot_tp1(9);
            q7 = Q_Qdot_tp1(10);
            q8 = Q_Qdot_tp1(11);
            q9 = Q_Qdot_tp1(12);
            q10 = Q_Qdot_tp1(13);
            
            rIxdot = Q_Qdot_tp1(14);
            rIydot = Q_Qdot_tp1(15);
            thetadot = Q_Qdot_tp1(16);
            q1dot = Q_Qdot_tp1(17);
            q2dot = Q_Qdot_tp1(18);
            q3dot = Q_Qdot_tp1(19);
            q4dot = Q_Qdot_tp1(20);
            q5dot = Q_Qdot_tp1(21);
            q6dot = Q_Qdot_tp1(22);
            q7dot = Q_Qdot_tp1(23);
            q8dot = Q_Qdot_tp1(24);
            q9dot = Q_Qdot_tp1(25);
            q10dot = Q_Qdot_tp1(26);
            
            % Then is to find the consistent constraint
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
            
            vA = vA_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
            vB = vB_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
            vC = vC_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
            vD = vD_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
            vM = vM_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
            vO = vO_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);
            
            if max(Contact_Status_t)==0   % Which frame has no contact?
                
                % So in this case, it is an impact!
                
                % In this case, the first frame has no contact with the
                % environment
                Contact_Status_Change = Contact_Status_tp1 - Contact_Status_t;
                
                % Then the idea is to make use of the consistent contact and the new added contact
                [~, Max_Ind] = max(Contact_Status_Change);
                % Here only one new added contact point will be considered
                
                Event_Index = 4 * (Max_Ind(1) -1) + 1;
                
                Contact_Index_Array = Max_Ind(1);
                Contact_Status = Contact_Status_Formulation(Contact_Index_Array);
                new_mode_q = Robot_State_Mode_Shaping(old_mode_q, Event_Index, Contact_Status, P);
                ceq = [ceq; new_mode_q - Q_Qdot_tp1];
                
            else
                % In this case, the second frame has no contact with the
                % ground Then it is okay to not compute anything for it
                
                
            end
            
            
            
        end
        
    end
end
end

function [Flag, Contact_Status] = Contact_Force2Holonomic_Constraint(Contact_Force_t)

% This function is used to judge which point that the current vector is in contact with
% The first part is for the foot contact

Contact_Status = zeros(6,1);
Feet_Even_Index_Array = 2:2:8;
Feet_Contact_Force = Contact_Force_t(Feet_Even_Index_Array);

for i = 1:4
    Contact_Status(i) = Contact_Status(i)||Feet_Contact_Force(i);
end

% The second part is for the hand contact.

Contact_Status(5) = Contact_Force_t(9)||Contact_Force_t(10);
Contact_Status(6) = Contact_Force_t(11)||Contact_Force_t(12);

if max(Contact_Status)==1
    Flag = 1;
else
    Flag = 0;
end
end

function Contact_Status = Contact_Status_Formulation(Contact_Index_Array)
% This function is used to formulate the contact status given the Contact_Index_Array
Contact_Status = zeros(6,2);
for i = 1:length(Contact_Index_Array)   
    Contact_Status(Contact_Index_Array(i),:) = [ 1 1];
end
end


function [new_mode_q, contact_status] = Robot_State_Mode_Shaping(old_mode_q, event_index, contact_status, P)

% This function is used to take care of the change of the robot state after the change of the mode

new_mode_q = old_mode_q;

rIx = old_mode_q(1);
rIy = old_mode_q(2);
theta = old_mode_q(3);
q1 = old_mode_q(4);
q2 = old_mode_q(5);
q3 = old_mode_q(6);
q4 = old_mode_q(7);
q5 = old_mode_q(8);
q6 = old_mode_q(9);
q7 = old_mode_q(10);
q8 = old_mode_q(11);
q9 = old_mode_q(12);
q10 = old_mode_q(13);

q_state = [rIx, rIy, theta, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10]';

rIxdot = old_mode_q(14);
rIydot = old_mode_q(15);
thetadot = old_mode_q(16);
q1dot = old_mode_q(17);
q2dot = old_mode_q(18);
q3dot = old_mode_q(19);
q4dot = old_mode_q(20);
q5dot = old_mode_q(21);
q6dot = old_mode_q(22);
q7dot = old_mode_q(23);
q8dot = old_mode_q(24);
q9dot = old_mode_q(25);
q10dot = old_mode_q(26);

qdot_old = [rIxdot, rIydot, thetadot, q1dot, q2dot, q3dot, q4dot, q5dot, q6dot, q7dot, q8dot, q9dot, q10dot]'; 

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
rM_fn = P.rM_fn;
rO_fn = P.rO_fn;

rA = rA_fn(q4,q5,q6,rIx,rIy,theta);
rB = rB_fn(q4,q5,q6,rIx,rIy,theta);
rC = rC_fn(q1,q2,q3,rIx,rIy,theta);
rD = rD_fn(q1,q2,q3,rIx,rIy,theta);
rM = rM_fn(q9,q10,rIx,rIy,theta);
rO = rO_fn(q7,q8,rIx,rIy,theta);

vA = P.vA_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vB = P.vB_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vC = P.vC_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vD = P.vD_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);

vM = P.vM_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
vO = P.vO_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);

T_fn = P.T_fn;
T_old = T_fn(q2,q3,q4,q5,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);

minorclass_index = mod(event_index, 4);
majorclass_index = (event_index - minorclass_index)/4;

if majorclass_index == 0
    majorclass_index = 1;
end

contact_position_name = cellstr(P.contact_position_name);

if minorclass_index == 1  % This is the only case that the robot makes impact contact with the environmental features
    % The main consideration is that the holonomic constraints are
    % satisfied at this time while the others are okay to be violated but
    % the kinetic energy has to decrease through this impact process.
    Jac_Full_fn = P.Jac_Full_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta)
    Jac_Full = Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
    %     Jacdot_qdot_fn = P.Jacdot_qdot_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta)
    %     Jacdot_qdot = Jacdot_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);
    Jac_t = [];
    Jac_index = [];
    for i = 1:6   
        r_i = eval(contact_position_name{i});
        flag = In_Contact_Or_Not(r_i,P);
        if flag ==1  % Means that the current constraint equation is satisfied
            if (mod(i,2) == 0)&&(min(contact_status(i-1,:)==1))&&(i<5)
                Jac_t = [Jac_t; Jac_Full(2*i,:)];
                Jac_index = [Jac_index; 2*i];
                contact_status(i, 1)=0.5;
                contact_status(i, 2)=1;
            else
                Jac_t = [Jac_t; Jac_Full(2*i-1,:)];
                Jac_t = [Jac_t; Jac_Full(2*i,:)];
                Jac_index = [Jac_index; 2*i-1];
                Jac_index = [Jac_index; 2*i];
                contact_status(i, 1)=1;
                contact_status(i, 2)=1;
            end
        end
    end
    A = Jac_t';
    [~,colind] = rref(A);
    B = A(:, colind);
    Full_rank_Jac_t = B';  
    A = [D_q, -Full_rank_Jac_t'; Full_rank_Jac_t, zeros(length(colind),length(colind))];
    B = [D_q * qdot_old; zeros(length(colind),1)];
    X = A\B;
    qdot_new = X(1:length(qdot_old));    
%     rIxdot = qdot_new(1);
%     rIydot = qdot_new(2);
%     thetadot = qdot_new(3);
%     q1dot = qdot_new(4);
%     q2dot = qdot_new(5);
%     q3dot = qdot_new(6);
%     q4dot = qdot_new(7);
%     q5dot = qdot_new(8);
%     q6dot = qdot_new(9);
%     q7dot = qdot_new(10);
%     q8dot = qdot_new(11);
%     q9dot = qdot_new(12);
%     q10dot = qdot_new(13);
%     T_new = T_fn(q2,q3,q4,q5,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);
%     if T_new<T_old
        new_mode_q = [q_state; qdot_new];
%     else        
%         % An optimization method should be adopted to solve for the proper
%         % new qdot
%         x0 = qdot_new;                  
%         fmincon_opt = optimoptions('fmincon','Algorithm','sqp');
%         
%         P.T_old = T_old;
%         P.Jac_index = Jac_index;
%         P.q_state = q_state;
%         
%         x = fmincon(@mode_switch_objective,x0,[],[],[],[],[],[],@mode_switch_constraints,fmincon_opt, P);
%         new_mode_q = [q_state; x];            
%     end    
end

if minorclass_index == 2  
    contact_status_pre = contact_status(majorclass_index, :);
    if (max(contact_status_pre) == min(contact_status_pre))&&(min(contact_status_pre)==1)
        contact_status(majorclass_index, :) = [0.5, 1];
    else
         contact_status(majorclass_index, :) = [1, 1];  
    end 
end

if minorclass_index == 3
    contact_status_pre = contact_status(majorclass_index, :);
    if (max(contact_status_pre) == min(contact_status_pre))&&(min(contact_status_pre)==1)
        contact_status(majorclass_index, :) = [1, 0.5];
    else
        contact_status(majorclass_index, :) = [1, 1];
    end  
end

if minorclass_index == 0
     contact_status(majorclass_index, :) = [0,0];    
end

end
% 
% function obj_val = mode_switch_objective(z, P)
% obj_val = 1;  % The most important result from this optimization is the satisfaction of the constraints
% end
% 
% function [c,ceq] = mode_switch_constraints(z, P)
% 
% old_mode_q = P.q_state;
% 
% rIx     = old_mode_q(1);
% rIy     = old_mode_q(2);
% theta   = old_mode_q(3);
% q1      = old_mode_q(4);
% q2      = old_mode_q(5);
% q3      = old_mode_q(6);
% q4      = old_mode_q(7);
% q5      = old_mode_q(8);
% q6      = old_mode_q(9);
% q7      = old_mode_q(10);
% q8      = old_mode_q(11);
% q9      = old_mode_q(12);
% q10     = old_mode_q(13);
% 
% rIxdot  = z(1);
% rIydot  = z(2);
% thetadot = z(3);
% q1dot   = z(4);
% q2dot   = z(5);
% q3dot   = z(6);
% q4dot   = z(7);
% q5dot   = z(8);
% q6dot   = z(9);
% q7dot   = z(10);
% q8dot   = z(11);
% q9dot   = z(12);
% q10dot  = z(13);
% 
% Jac_Full_fn = P.Jac_Full_fn; %@(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta)
% Jac_Full = Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
% 
% Jac_Test = Jac_Full * z;
% 
% ceq = Jac_Test(P.Jac_index)';
% 
% T_fn = P.T_fn;
% T_new = T_fn(q2,q3,q4,q5,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);
% 
% c = P.T_old - 0.9 * T_new;  % Here 0.8 is an arbitrary estimation
% 
% end

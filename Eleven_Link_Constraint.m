function [c, ceq] = Eleven_Link_Constraint(z,P)
% This is the most important part of the proposed method
% c(x)<=0
c = [];                     % Inequality constraints
ceq = [];                   % Equality constraints

[Contact_Force_Array, Q_Qdot_Array, Control_Torque_Array] = Optimal_Variables_Unzip(z, P);

% Constraints!
[c, ceq] = Optimization_Constraint_Formulation(Contact_Force_Array, Q_Qdot_Array, Control_Torque_Array, P);

% One more equality equation to be added is that the initial contact mode
% should be the same as the initial one. So this requires the following
% modes within that segment to be exactly the same as the initial one.

% [c,ceq ] = Contact_Constraint_Cal( c,ceq, P, P.Init_Contact_Status, P.init0, Trans_State_T_N, All_Grids, Contact_Force_Pattern_T_N, Control_Torque_T_N);
end
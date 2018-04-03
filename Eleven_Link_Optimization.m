function [Contact_Sequence,x0 ]= Eleven_Link_Optimization(contact_status_ref, q_ref, handles)
% This function is used to optimization the trajectory to stabilize a
% humanoid robot given arbitrary disturbance and environmental features
P = handles.P;
global contact_status
contact_status  = contact_status_reformulate(contact_status_ref);

Contact_Sequence = [1, 11, 111, 1111];     % This is where the  contact sequence is defined
P.Contact_Sequence = Contact_Sequence;
% The first element has to match the real robot contact status

rIx       = q_ref(1);
rIy       = q_ref(2);
theta     = q_ref(3);                % Checked
q1        = q_ref(4);                % Checked
q2        = q_ref(5);                % Checked
q3        = q_ref(6);                % Checked
q4        = q_ref(7);                % Checked
q5        = q_ref(8);                % Checked
q6        = q_ref(9);                % Checked
q7        = q_ref(10);               % Checked
q8        = q_ref(11);               % Checked
q9        = q_ref(12);               % Checked
q10       = q_ref(13);               % Checked
q = [rIx, rIy, theta, q1,q2,q3,q4,q5,q6,q7,q8,q9,q10]';
rIxdot    = q_ref(14);
rIydot    = q_ref(15);
thetadot  = q_ref(16);
q1dot     = q_ref(17);
q2dot     = q_ref(18);
q3dot     = q_ref(19);
q4dot     = q_ref(20);
q5dot     = q_ref(21);
q6dot     = q_ref(22);
q7dot     = q_ref(23);
q8dot     = q_ref(24);
q9dot     = q_ref(25);
q10dot    = q_ref(26);
qdot = [rIxdot, rIydot, thetadot, q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot]';

P.mu = 0.5;        % Initialization of the friction coefficient

P.u_max = 2;       % Maximum torque is default to be 1 Nm
P.qdot_max = 10;   % Maximum angular velocity at the joint motor
P.eps = 0.05;

P.init0 = [q;qdot];

Contact_Force_Init = Contact_Force_Computation(P.init0, contact_status, P, 'init');

% The basic assumption is that the contact force can always satisfy the
% friction force constraint

% tf = 5;           P.tf = tf;                                  % Total running time
% dt = 0.1;         P.dt = dt;                                  % Period of is the sampling rate
% N = tf/dt;        P.N = N;                                    % Total discretization points

P.grids_per_segment = 10;
P.N = P.grids_per_segment * length(Contact_Sequence); N= P.N; 
P.dt = 1/9;
P.Contact_Force_Init = Contact_Force_Init;  % This is the initial contact status
% Variables to be optimized

% 1. Contact Force
[Contact_Force_Array, Contact_Status_Array, Avg_Grid_In_Each_Mode]= Contact_Force_Initialization(N, P.Contact_Force_Init, contact_status);

% 2. Joint Angles and Angular velocity
Q_Qdot_Array = State_Angle_Angular_Initialization_Constraint_Optimization(P.init0, Contact_Status_Array, N, Avg_Grid_In_Each_Mode, P);

% 3. Joint Torques 
Control_Torque_Array = P.u_max * rand(10, N);

% Concatenate all the variables to be in a single column vector
x0 = Optimization_Variable_Concatenation(Contact_Force_Array, Q_Qdot_Array, Control_Torque_Array, P);

% fmincon_opt = optimoptions('fmincon','Display','iter','Algorithm','interior-point', 'MaxFunEvals',Inf,'MaxIter',5000);
% x = fmincon(@Eleven_Link_Stabilization_Obj,x0,[],[],[],[],option_lb, option_ub,@Eleven_Link_Constraint,fmincon_opt, P);
end

function Obj_val = Eleven_Link_Stabilization_Obj(z,P)

Obj_val = 1;

end
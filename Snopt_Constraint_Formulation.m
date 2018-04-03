function Snopt_Constraint_Formulation(handles,mode_sequence)

% This function is used to formulate the constraint for the optimization in snopt

load('Symbolic_Structure.mat');  % Here we have the symbolic expression

if nargin ==2
    rIx        = handles.rIx;
    rIy        = handles.rIy;
    theta      = handles.theta * pi/180;
    q1         = handles.q1 * pi/180;
    q2         = handles.q2 * pi/180;
    q3         = handles.q3 * pi/180;
    q4         = handles.q4 * pi/180;
    q5         = handles.q5 * pi/180;
    q6         = handles.q6 * pi/180;
    q7         = handles.q7 * pi/180;
    q8         = handles.q8 * pi/180;
    q9         = handles.q9 * pi/180;
    q10        = handles.q10 * pi/180;
    
    rIxdot     = handles.rIxdot;
    rIydot     = handles.rIydot;
    thetadot   = handles.thetadot;
    q1dot      = handles.q1dot;
    q2dot      = handles.q2dot;
    q3dot      = handles.q3dot;
    q4dot      = handles.q4dot;
    q5dot      = handles.q5dot;
    q6dot      = handles.q6dot;
    q7dot      = handles.q7dot;
    q8dot      = handles.q8dot;
    q9dot      = handles.q9dot;
    q10dot     = handles.q10dot;
    
else
    rIx = -0.0000;
    rIy = 0.5009;
    theta = 1.3708;
    q1 =  1.5626;
    q2 =  2.0940;
    q3 =  4.1883;
    q4 =   2.7429;
    q5 =  2.6908;
    q6 =  1.5888;
    q7 =   1.0472;
    q8 =   0.7854;
    q9 =   1.0472;
    q10 =     0.5236;
    rIxdot =     1.2518;
    rIydot =    -0.7585;
    thetadot =    -0.9611;
    q1dot =    -1.0000;
    q2dot =    -1.0000;
    q3dot =     1.0000;
    q4dot =    -0.9611;
    q5dot =   -0.9906;
    q6dot =   -1.0341;
    q7dot =    1.0000;
    q8dot =   -1.0000;
    q9dot =    1.0000;
    q10dot =   1.0000;
end

% Here we would like to define the whole body stabilization process
mode_sequence = [1 11 111 1111];
grids_per_segment = 10;

q_init = [rIx, rIy, theta, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, rIxdot, rIydot, thetadot, q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot]';

Snopt_Constraint_Concatenation(mode_sequence, grids_per_segment, q_init, Q);

end

function Snopt_Constraint_Concatenation(mode_sequence, grids_per_segment, q_init, Q)

% Be sure to have the C++ code also have the same distance for the vertical wall

Vert_wall_dist = 5;

syms rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 real
syms rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot real
syms rIxddot rIyddot thetaddot thetaddot q1ddot q2ddot q3ddot q4ddot q5ddot q6ddot q7ddot q8ddot q9ddot q10ddot real
syms u1 u2 u3 u4 u5 u6 u7 u8 u9 u10 real

% This function is used to formulate the whole body stablization problem
segment_number = length(mode_sequence);
period_per_segment = 1;    % This value can be optimized
period_per_grid = period_per_segment/(grids_per_segment - 1);

% for i = 1:segment_number
% 
%     for j = 1:grids_per_segment
% 
%         % Initialization of the state
%         eval(['syms ','rIx','_',num2str(i),'_',num2str(j),' rIy','_',num2str(i),'_',num2str(j),' theta','_',...
%            num2str(i),'_',num2str(j),' q1','_',num2str(i),'_',num2str(j),' q2','_',num2str(i),'_',num2str(j)...
%                      ,' q3','_',num2str(i),'_',num2str(j),' q4','_',num2str(i),'_',num2str(j),' q5','_',...
%                      num2str(i),'_',num2str(j),' q6','_',num2str(i),'_',num2str(j),' q7','_',num2str(i),...
%                      '_',num2str(j),' q8','_',num2str(i),'_',num2str(j),' q9','_',num2str(i),'_',num2str(j),...
%                      ' q10','_',num2str(i),'_',num2str(j),' real']);
% 
%         % Initialization of the statedot
%         eval(['syms ',' rIxdot','_',num2str(i),'_',num2str(j),' rIydot','_',num2str(i),'_',num2str(j),...
%                      ' thetadot','_',num2str(i),'_',num2str(j),' q1dot','_',num2str(i),'_',num2str(j),...
%                      ' q2dot','_',num2str(i),'_',num2str(j) ,' q3dot','_',num2str(i),'_',num2str(j),....
%                      ' q4dot','_',num2str(i),'_',num2str(j),' q5dot','_',num2str(i),'_',num2str(j),...
%                      ' q6dot','_',num2str(i),'_',num2str(j),' q7dot','_',num2str(i),'_',num2str(j)...
%                      ,' q8dot','_',num2str(i),'_',num2str(j),' q9dot','_',num2str(i),'_',num2str(j),...
%                      ' q10dot','_',num2str(i),'_',num2str(j),' real']);
% 
%         % Initialization of the lamda
%         eval(['syms ',' lamda_Ax','_',num2str(i),'_',num2str(j),' lamda_Ay','_',num2str(i),'_',num2str(j),...
%                       ' lamda_Bx','_',num2str(i),'_',num2str(j),' lamda_By','_',num2str(i),'_',num2str(j),...
%                       ' lamda_Cx','_',num2str(i),'_',num2str(j),' lamda_Cy','_',num2str(i),'_',num2str(j),...
%                       ' lamda_Dx','_',num2str(i),'_',num2str(j),' lamda_Dy','_',num2str(i),'_',num2str(j),...
%                       ' lamda_Mx','_',num2str(i),'_',num2str(j),' lamda_My','_',num2str(i),'_',num2str(j),...
%                       ' lamda_Ox','_',num2str(i),'_',num2str(j),' lamda_Oy','_',num2str(i),'_',num2str(j),...
%                      ' real']);
%         % Initialization of the control
%         eval(['syms ',' u1','_',num2str(i),'_',num2str(j),' u2','_',num2str(i),'_',num2str(j),...
%                       ' u3','_',num2str(i),'_',num2str(j),' u4','_',num2str(i),'_',num2str(j),...
%                       ' u5','_',num2str(i),'_',num2str(j),' u6','_',num2str(i),'_',num2str(j),...
%                       ' u7','_',num2str(i),'_',num2str(j),' u8','_',num2str(i),'_',num2str(j),...
%                       ' u9','_',num2str(i),'_',num2str(j),' u10','_',num2str(i),'_',num2str(j),' real']);
%     end
%     if i<segment_number
%         eval(['syms ',' lambar_Ax','_',num2str(i),' lambar_Ay','_',num2str(i),...
%             ' lambar_Bx','_',num2str(i),' lambar_By','_',num2str(i),...
%             ' lambar_Cx','_',num2str(i),' lambar_Cy','_',num2str(i),...
%             ' lambar_Dx','_',num2str(i),' lambar_Dy','_',num2str(i),...
%             ' lambar_Mx','_',num2str(i),' lambar_My','_',num2str(i),...
%             ' lambar_Ox','_',num2str(i),' lambar_Oy','_',num2str(i),...
%             ' real']);
%     end
% end
% 
% save('1_11_111_1111.mat');

load('1_11_111_1111.mat');

% 1. The first part is the formulation of the variables to be optimized

Snopt_State = [];
Snopt_State_Bound = [];
Snopt_State_Raw = [];

% To make sure that an obvious pattern can be found, we would prefer to
% generate the state one by one

%% First is the generation of the joint angles and angular velocities
for i = 1: segment_number
    for j = 1:grids_per_segment
        % 1. State and Statedot
        eval(['rIx_new = rIx_',num2str(i),'_',num2str(j),';']);
        eval(['rIy_new = rIy_',num2str(i),'_',num2str(j),';']);
        eval(['theta_new = theta_',num2str(i),'_',num2str(j),';']);
        eval(['q1_new = q1_',num2str(i),'_',num2str(j),';']);
        eval(['q2_new = q2_',num2str(i),'_',num2str(j),';']);
        eval(['q3_new = q3_',num2str(i),'_',num2str(j),';']);
        eval(['q4_new = q4_',num2str(i),'_',num2str(j),';']);
        eval(['q5_new = q5_',num2str(i),'_',num2str(j),';']);
        eval(['q6_new = q6_',num2str(i),'_',num2str(j),';']);
        eval(['q7_new = q7_',num2str(i),'_',num2str(j),';']);
        eval(['q8_new = q8_',num2str(i),'_',num2str(j),';']);
        eval(['q9_new = q9_',num2str(i),'_',num2str(j),';']);
        eval(['q10_new = q10_',num2str(i),'_',num2str(j),';']);
        
        eval(['rIxdot_new = rIxdot_',num2str(i),'_',num2str(j),';']);
        eval(['rIydot_new = rIydot_',num2str(i),'_',num2str(j),';']);
        eval(['thetadot_new = thetadot_',num2str(i),'_',num2str(j),';']);
        eval(['q1dot_new = q1dot_',num2str(i),'_',num2str(j),';']);
        eval(['q2dot_new = q2dot_',num2str(i),'_',num2str(j),';']);
        eval(['q3dot_new = q3dot_',num2str(i),'_',num2str(j),';']);
        eval(['q4dot_new = q4dot_',num2str(i),'_',num2str(j),';']);
        eval(['q5dot_new = q5dot_',num2str(i),'_',num2str(j),';']);
        eval(['q6dot_new = q6dot_',num2str(i),'_',num2str(j),';']);
        eval(['q7dot_new = q7dot_',num2str(i),'_',num2str(j),';']);
        eval(['q8dot_new = q8dot_',num2str(i),'_',num2str(j),';']);
        eval(['q9dot_new = q9dot_',num2str(i),'_',num2str(j),';']);
        eval(['q10dot_new = q10dot_',num2str(i),'_',num2str(j),';']);
        
        Snopt_State = [Snopt_State  rIx_new, rIy_new, theta_new, q1_new, q2_new, q3_new, q4_new, q5_new, q6_new, q7_new, q8_new, q9_new, q10_new,...
            rIxdot_new, rIydot_new, thetadot_new, q1dot_new, q2dot_new, q3dot_new, q4dot_new, q5dot_new, q6dot_new, q7dot_new, q8dot_new, q9dot_new, q10dot_new];
        
        [ Snopt_State_Bound, Snopt_State_Raw ] = state_bound_formulation(Snopt_State_Bound, Snopt_State_Raw, 1);
        
        % 2. Control variables
        eval(['u1_new = u1_',num2str(i),'_',num2str(j),';']);
        eval(['u2_new = u2_',num2str(i),'_',num2str(j),';']);
        eval(['u3_new = u3_',num2str(i),'_',num2str(j),';']);
        eval(['u4_new = u4_',num2str(i),'_',num2str(j),';']);
        eval(['u5_new = u5_',num2str(i),'_',num2str(j),';']);
        eval(['u6_new = u6_',num2str(i),'_',num2str(j),';']);
        eval(['u7_new = u7_',num2str(i),'_',num2str(j),';']);
        eval(['u8_new = u8_',num2str(i),'_',num2str(j),';']);
        eval(['u9_new = u9_',num2str(i),'_',num2str(j),';']);
        eval(['u10_new = u10_',num2str(i),'_',num2str(j),';']);
        
        Snopt_State = [Snopt_State  u1_new u2_new u3_new u4_new u5_new u6_new u7_new u8_new u9_new u10_new];
        [ Snopt_State_Bound, Snopt_State_Raw ]= state_bound_formulation(Snopt_State_Bound, Snopt_State_Raw, 2);
        
        % 3. Contact force variables
        eval(['lamda_Ax_new = lamda_Ax_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Ay_new = lamda_Ay_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Bx_new = lamda_Bx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_By_new = lamda_By_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Cx_new = lamda_Cx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Cy_new = lamda_Cy_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Dx_new = lamda_Dx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Dy_new = lamda_Dy_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Mx_new = lamda_Mx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_My_new = lamda_My_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Ox_new = lamda_Ox_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Oy_new = lamda_Oy_',num2str(i),'_',num2str(j),';']);
        
        Snopt_State = [Snopt_State  lamda_Ax_new lamda_Ay_new lamda_Bx_new lamda_By_new lamda_Cx_new lamda_Cy_new...
            lamda_Dx_new lamda_Dy_new lamda_Mx_new lamda_My_new lamda_Ox_new lamda_Oy_new];
        [ Snopt_State_Bound, Snopt_State_Raw ] = state_bound_formulation(Snopt_State_Bound, Snopt_State_Raw, 3);
    end
end

for i = 1:segment_number-1
    % Here we come to the ending time of the current segments and it is time to add the impact mapping constraint into the new state
    eval(['lambar_i = [',' lambar_Ax_',num2str(i),' lambar_Ay_',num2str(i),...
        ' lambar_Bx_',num2str(i),' lambar_By_',num2str(i),...
        ' lambar_Cx_',num2str(i),' lambar_Cy_',num2str(i),...
        ' lambar_Dx_',num2str(i),' lambar_Dy_',num2str(i),...
        ' lambar_Mx_',num2str(i),' lambar_My_',num2str(i),...
        ' lambar_Ox_',num2str(i),' lambar_Oy_',num2str(i),'];']);
    Snopt_State = [Snopt_State  lambar_i];
    [ Snopt_State_Bound, Snopt_State_Raw ]= state_bound_formulation(Snopt_State_Bound, Snopt_State_Raw, 4);
    
end
Snopt_State = Snopt_State.';

Snopt_State_Bound = state_bound_formulation_loop();

% The second part constraint is the dynamics contraints

% Now it is the most important part: formulation of the optimized variable

D_q = real(Q.D_q);
B_q = real(Q.B_q);
C_q_qdot = real(Q.C_q_qdot);
Jac_Full = real(Q.Jac_Full);

rA = Q.rA;
rB = Q.rB;
rC = Q.rC;
rD = Q.rD;
rM = Q.rM;
rO = Q.rO;

Snopt_Constraint_Bound = [];
Snopt_Constraint_Raw = [];

Snopt_Constraint_tot = [];
Snopt_Full_Constraint_Jac_tot = [];

% The first one is for the constraint equations
Snopt_Constraint_CppCode = [];

% The second two for the jacobians
Snopt_iGfunjGvar_CppCode = [];
Snopt_Jacobian_CppCode = [];

eps = 0.01;   % This is used to maintain the positivity of a certain constraint

%% 1. Initial constraints

Snopt_Constraint_CppCode = [Snopt_Constraint_CppCode '*neF = 0; \n for(int i = 0; i<26; i++) \n { \n   F[*neG] = x[i] - ru[i]; \n   *neF = *neF + 1; \n} \n'];
Snopt_Jacobian_CppCode = [Snopt_Jacobian_CppCode '*neG = 0; \n for( int j = 0; j<26; j++) \n { G[*neG] = 1.0; \n *neG = *neG + 1; \n}'];
Snopt_iGfunjGvar_CppCode = [Snopt_iGfunjGvar_CppCode 'neG = 0; \n for(int j = 0; j<26; j++) \n { iGfun[neG] = j; \n jGvar[neG] = j;\n  neG++; \n} \n'];
[Snopt_Constraint_Bound, Snopt_Constraint_Raw]= constraint_bound_formulation(Snopt_Constraint_Bound, Snopt_Constraint_Raw, 26, 0);

%% 2. Dynamics constraints
% for i = 1:segment_number
%     for j = 1:grids_per_segment-1
i = 1; j = 1;
eval(['rIx_new = rIx_',num2str(i),'_',num2str(j),';']);
eval(['rIy_new = rIy_',num2str(i),'_',num2str(j),';']);
eval(['theta_new = theta_',num2str(i),'_',num2str(j),';']);
eval(['q1_new = q1_',num2str(i),'_',num2str(j),';']);
eval(['q2_new = q2_',num2str(i),'_',num2str(j),';']);
eval(['q3_new = q3_',num2str(i),'_',num2str(j),';']);
eval(['q4_new = q4_',num2str(i),'_',num2str(j),';']);
eval(['q5_new = q5_',num2str(i),'_',num2str(j),';']);
eval(['q6_new = q6_',num2str(i),'_',num2str(j),';']);
eval(['q7_new = q7_',num2str(i),'_',num2str(j),';']);
eval(['q8_new = q8_',num2str(i),'_',num2str(j),';']);
eval(['q9_new = q9_',num2str(i),'_',num2str(j),';']);
eval(['q10_new = q10_',num2str(i),'_',num2str(j),';']);

eval(['rIxdot_new = rIxdot_',num2str(i),'_',num2str(j),';']);
eval(['rIydot_new = rIydot_',num2str(i),'_',num2str(j),';']);
eval(['thetadot_new = thetadot_',num2str(i),'_',num2str(j),';']);
eval(['q1dot_new = q1dot_',num2str(i),'_',num2str(j),';']);
eval(['q2dot_new = q2dot_',num2str(i),'_',num2str(j),';']);
eval(['q3dot_new = q3dot_',num2str(i),'_',num2str(j),';']);
eval(['q4dot_new = q4dot_',num2str(i),'_',num2str(j),';']);
eval(['q5dot_new = q5dot_',num2str(i),'_',num2str(j),';']);
eval(['q6dot_new = q6dot_',num2str(i),'_',num2str(j),';']);
eval(['q7dot_new = q7dot_',num2str(i),'_',num2str(j),';']);
eval(['q8dot_new = q8dot_',num2str(i),'_',num2str(j),';']);
eval(['q9dot_new = q9dot_',num2str(i),'_',num2str(j),';']);
eval(['q10dot_new = q10dot_',num2str(i),'_',num2str(j),';']);

D_q_i_j = subs(D_q,[q2,q3,q4,q5,q7,q8,q9,q10,theta], [q2_new,q3_new,q4_new,q5_new,q7_new,q8_new,q9_new,q10_new,theta_new]);
C_q_qdot_i_j = subs(C_q_qdot,[q2,q3,q4,q5,q7,q8,q9,q10,q10dot,q2dot,q3dot,q4dot,q5dot,q7dot,q8dot,q9dot,thetadot,theta],...
    [q2_new,q3_new,q4_new,q5_new,q7_new,q8_new,q9_new,q10_new,...
    q10dot_new,q2dot_new,q3dot_new,q4dot_new,q5dot_new,q7dot_new,q8dot_new,q9dot_new,thetadot_new,theta_new]);
eval(['rIxdot_new_p = rIxdot_',num2str(i),'_',num2str(j+1),';']);
eval(['rIydot_new_p = rIydot_',num2str(i),'_',num2str(j+1),';']);
eval(['thetadot_new_p = thetadot_',num2str(i),'_',num2str(j+1),';']);
eval(['q1dot_new_p = q1dot_',num2str(i),'_',num2str(j+1),';']);
eval(['q2dot_new_p = q2dot_',num2str(i),'_',num2str(j+1),';']);
eval(['q3dot_new_p = q3dot_',num2str(i),'_',num2str(j+1),';']);
eval(['q4dot_new_p = q4dot_',num2str(i),'_',num2str(j+1),';']);
eval(['q5dot_new_p = q5dot_',num2str(i),'_',num2str(j+1),';']);
eval(['q6dot_new_p = q6dot_',num2str(i),'_',num2str(j+1),';']);
eval(['q7dot_new_p = q7dot_',num2str(i),'_',num2str(j+1),';']);
eval(['q8dot_new_p = q8dot_',num2str(i),'_',num2str(j+1),';']);
eval(['q9dot_new_p = q9dot_',num2str(i),'_',num2str(j+1),';']);
eval(['q10dot_new_p = q10dot_',num2str(i),'_',num2str(j+1),';']);

qddot_i_j_statedot_p =[rIxdot_new_p rIydot_new_p thetadot_new_p q1dot_new_p q2dot_new_p q3dot_new_p q4dot_new_p q5dot_new_p q6dot_new_p q7dot_new_p q8dot_new_p q9dot_new_p q10dot_new_p].';
qddot_i_j_statedot =[rIxdot_new rIydot_new thetadot_new q1dot_new q2dot_new q3dot_new q4dot_new q5dot_new q6dot_new q7dot_new q8dot_new q9dot_new q10dot_new].';
qddot_i_j = ([qddot_i_j_statedot_p] - [qddot_i_j_statedot])/period_per_grid;

eval(['u1_new = u1_',num2str(i),'_',num2str(j),';']);
eval(['u2_new = u2_',num2str(i),'_',num2str(j),';']);
eval(['u3_new = u3_',num2str(i),'_',num2str(j),';']);
eval(['u4_new = u4_',num2str(i),'_',num2str(j),';']);
eval(['u5_new = u5_',num2str(i),'_',num2str(j),';']);
eval(['u6_new = u6_',num2str(i),'_',num2str(j),';']);
eval(['u7_new = u7_',num2str(i),'_',num2str(j),';']);
eval(['u8_new = u8_',num2str(i),'_',num2str(j),';']);
eval(['u9_new = u9_',num2str(i),'_',num2str(j),';']);
eval(['u10_new = u10_',num2str(i),'_',num2str(j),';']);

eval(['lamda_Ax_new = lamda_Ax_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Ay_new = lamda_Ay_',num2str(i),'_',num2str(j),';']);

eval(['lamda_Bx_new = lamda_Bx_',num2str(i),'_',num2str(j),';']);
eval(['lamda_By_new = lamda_By_',num2str(i),'_',num2str(j),';']);

eval(['lamda_Cx_new = lamda_Cx_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Cy_new = lamda_Cy_',num2str(i),'_',num2str(j),';']);

eval(['lamda_Dx_new = lamda_Dx_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Dy_new = lamda_Dy_',num2str(i),'_',num2str(j),';']);

eval(['lamda_Mx_new = lamda_Mx_',num2str(i),'_',num2str(j),';']);
eval(['lamda_My_new = lamda_My_',num2str(i),'_',num2str(j),';']);

eval(['lamda_Ox_new = lamda_Ox_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Oy_new = lamda_Oy_',num2str(i),'_',num2str(j),';']);

Jac_Full_i_j = subs(Jac_Full,[q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta],[q1_new,q2_new,q3_new,q4_new,q5_new,q6_new,q7_new,q8_new,q9_new,q10_new,theta_new]);

Dynamics_Constraint_i_j = D_q_i_j * qddot_i_j + C_q_qdot_i_j - Jac_Full_i_j.' * [lamda_Ax_new; lamda_Ay_new;lamda_Bx_new;...
    lamda_By_new;lamda_Cx_new; lamda_Cy_new;lamda_Dx_new; lamda_Dy_new;...
    lamda_Mx_new; lamda_My_new;...
    lamda_Ox_new; lamda_Oy_new] - B_q() * [u1_new;u2_new;u3_new;u4_new;u5_new;u6_new;u7_new;u8_new;u9_new;u10_new];

Snopt_Full_Constraint_Jac_tot = [Snopt_Full_Constraint_Jac_tot; jacobian(Dynamics_Constraint_i_j, Snopt_State)];

Snopt_Jacobian_Raw = 26;  % This one is used to save the number of useful jacobian components

Fsym_Dyn_Total = Sym2C_Index_Converter_Loop_Dynamic(ccode(Dynamics_Constraint_i_j), length(Snopt_Constraint_Raw));

[iGfunjGvar_Dyn_Total, Gsym_Dyn_Total, row_Dyn_no] = Sym2C_Index_Converter_Loop_Dynamic_Jac(ccode(Snopt_Full_Constraint_Jac_tot), length(Snopt_Constraint_Raw), Snopt_Jacobian_Raw);

[Snopt_Constraint_Bound, Snopt_Constraint_Raw]= constraint_bound_formulation(Snopt_Constraint_Bound, Snopt_Constraint_Raw, length(Dynamics_Constraint_i_j) *segment_number * (grids_per_segment-1), 0);

Snopt_Constraint_CppCode = [Snopt_Constraint_CppCode Fsym_Dyn_Total];
Snopt_iGfunjGvar_CppCode = [Snopt_iGfunjGvar_CppCode iGfunjGvar_Dyn_Total];
Snopt_Jacobian_CppCode = [Snopt_Jacobian_CppCode Gsym_Dyn_Total];

Snopt_Jacobian_Raw = Snopt_Jacobian_Raw + 9 * 4 * row_Dyn_no;


%% 3. Complementarity constraints
eval(['rIx_new = rIx_',num2str(i),'_',num2str(j),';']);
eval(['rIy_new = rIy_',num2str(i),'_',num2str(j),';']);
eval(['theta_new = theta_',num2str(i),'_',num2str(j),';']);
eval(['q1_new = q1_',num2str(i),'_',num2str(j),';']);
eval(['q2_new = q2_',num2str(i),'_',num2str(j),';']);
eval(['q3_new = q3_',num2str(i),'_',num2str(j),';']);
eval(['q4_new = q4_',num2str(i),'_',num2str(j),';']);
eval(['q5_new = q5_',num2str(i),'_',num2str(j),';']);
eval(['q6_new = q6_',num2str(i),'_',num2str(j),';']);
eval(['q7_new = q7_',num2str(i),'_',num2str(j),';']);
eval(['q8_new = q8_',num2str(i),'_',num2str(j),';']);
eval(['q9_new = q9_',num2str(i),'_',num2str(j),';']);
eval(['q10_new = q10_',num2str(i),'_',num2str(j),';']);

eval(['lamda_Ax_new = lamda_Ax_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Ay_new = lamda_Ay_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Bx_new = lamda_Bx_',num2str(i),'_',num2str(j),';']);
eval(['lamda_By_new = lamda_By_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Cx_new = lamda_Cx_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Cy_new = lamda_Cy_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Dx_new = lamda_Dx_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Dy_new = lamda_Dy_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Mx_new = lamda_Mx_',num2str(i),'_',num2str(j),';']);
eval(['lamda_My_new = lamda_My_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Ox_new = lamda_Ox_',num2str(i),'_',num2str(j),';']);
eval(['lamda_Oy_new = lamda_Oy_',num2str(i),'_',num2str(j),';']);

rA_new = subs(rA, [q4,q5,q6,rIx,rIy,theta], [q4_new,q5_new,q6_new,rIx_new,rIy_new,theta_new]);
rB_new = subs(rB, [q4,q5,q6,rIx,rIy,theta], [q4_new,q5_new,q6_new,rIx_new,rIy_new,theta_new]);
rC_new = subs(rC, [q1,q2,q3,rIx,rIy,theta], [q1_new,q2_new,q3_new,rIx_new,rIy_new,theta_new]);
rD_new = subs(rD, [q1,q2,q3,rIx,rIy,theta], [q1_new,q2_new,q3_new,rIx_new,rIy_new,theta_new]);
rM_new = subs(rM, [q9,q10,rIx,rIy,theta],   [q9_new,q10_new,rIx_new,rIy_new,theta_new]);
rO_new = subs(rO, [q7,q8,rIx,rIy,theta],    [q7_new,q8_new,rIx_new,rIy_new,theta_new]);

Contact_Complementarity_Constraint_i_j  =[lamda_Ax_new * rA_new(2);...
    lamda_Ay_new * rA_new(2);...
    lamda_Bx_new * rB_new(2);...
    lamda_By_new * rB_new(2);...
    lamda_Cx_new * rC_new(2);...
    lamda_Cy_new * rC_new(2);...
    lamda_Dx_new * rD_new(2);...
    lamda_Dy_new * rD_new(2);...
    lamda_Mx_new * (Vert_wall_dist - rM_new(1));...
    lamda_My_new * (Vert_wall_dist - rM_new(1));...
    lamda_Ox_new * (Vert_wall_dist - rO_new(1));...
    lamda_Oy_new * (Vert_wall_dist - rO_new(1))];

Fsym_Cmpmtrt_Total = Sym2C_Index_Converter_Loop_Complementarity(ccode(Contact_Complementarity_Constraint_i_j), length(Snopt_Constraint_Raw));

[iGfunjGvar_Cmpmtrt_Total, Gsym_Cmpmtrt_Total, row_Cmpmtrt_no] = Sym2C_Index_Converter_Loop_Complementarity_Jac(ccode(jacobian(Contact_Complementarity_Constraint_i_j, Snopt_State)), length(Snopt_Constraint_Raw), Snopt_Jacobian_Raw);
 
[Snopt_Constraint_Bound, Snopt_Constraint_Raw]= constraint_bound_formulation(Snopt_Constraint_Bound, Snopt_Constraint_Raw, length(Contact_Complementarity_Constraint_i_j) *segment_number * (grids_per_segment-1), 0);

Snopt_Constraint_CppCode = [Snopt_Constraint_CppCode Fsym_Cmpmtrt_Total];
Snopt_iGfunjGvar_CppCode = [Snopt_iGfunjGvar_CppCode iGfunjGvar_Cmpmtrt_Total];
Snopt_Jacobian_CppCode = [Snopt_Jacobian_CppCode Gsym_Cmpmtrt_Total];

Snopt_Jacobian_Raw = Snopt_Jacobian_Raw + 9 * 4 * row_Cmpmtrt_no;

%% 4. Strict contact force requirements
% for i = 1: segment_number    
 i = 1; j = 1;   
 [coef_A, coef_B, coef_C, coef_D, coef_M, coef_O] = Contact_Force_Constraint_Coef(mode_sequence(i));
 coef_A_val = coef_A;
 coef_B_val = coef_B;
 coef_C_val = coef_C;
 coef_D_val = coef_D;
 coef_M_val = coef_M;
 coef_O_val = coef_O;
 syms coef_A coef_B coef_C coef_D coef_M coef_O real

    
%     for j = 1:grids_per_segment-1
        % Contact constraints: one constraint is the certain force has to
        % be positive, while the other constraint is that certain distance
        % has to be zero or not
        
        eval(['rIx_new = rIx_',num2str(i),'_',num2str(j),';']);
        eval(['rIy_new = rIy_',num2str(i),'_',num2str(j),';']);
        eval(['theta_new = theta_',num2str(i),'_',num2str(j),';']);
        eval(['q1_new = q1_',num2str(i),'_',num2str(j),';']);
        eval(['q2_new = q2_',num2str(i),'_',num2str(j),';']);
        eval(['q3_new = q3_',num2str(i),'_',num2str(j),';']);
        eval(['q4_new = q4_',num2str(i),'_',num2str(j),';']);
        eval(['q5_new = q5_',num2str(i),'_',num2str(j),';']);
        eval(['q6_new = q6_',num2str(i),'_',num2str(j),';']);
        eval(['q7_new = q7_',num2str(i),'_',num2str(j),';']);
        eval(['q8_new = q8_',num2str(i),'_',num2str(j),';']);
        eval(['q9_new = q9_',num2str(i),'_',num2str(j),';']);
        eval(['q10_new = q10_',num2str(i),'_',num2str(j),';']);
        
        eval(['lamda_Ax_new = lamda_Ax_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Ay_new = lamda_Ay_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Bx_new = lamda_Bx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_By_new = lamda_By_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Cx_new = lamda_Cx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Cy_new = lamda_Cy_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Dx_new = lamda_Dx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Dy_new = lamda_Dy_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Mx_new = lamda_Mx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_My_new = lamda_My_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Ox_new = lamda_Ox_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Oy_new = lamda_Oy_',num2str(i),'_',num2str(j),';']);
        
        
        Contact_Force_Constraint_i_j = [lamda_Ay_new - coef_A * eps;...
            lamda_By_new - coef_B * eps;...
            lamda_Cy_new - coef_C * eps;...
            lamda_Dy_new - coef_D * eps;...
            -(lamda_Mx_new + coef_M * eps);...
            -(lamda_Ox_new + coef_O * eps)];
        
        Fsym_StrictF_Total = Sym2C_Index_Converter_Loop_StrictF(ccode(Contact_Force_Constraint_i_j), length(Snopt_Constraint_Raw));
        
        [iGfunjGvar_StrictF_Total, Gsym_StrictF_Total, row_StrictF_no] = Sym2C_Index_Converter_Loop_StrictF_Jac(ccode(jacobian(Contact_Force_Constraint_i_j, Snopt_State)), length(Snopt_Constraint_Raw), Snopt_Jacobian_Raw);
        
        [Snopt_Constraint_Bound, Snopt_Constraint_Raw]= constraint_bound_formulation(Snopt_Constraint_Bound, Snopt_Constraint_Raw, length(Contact_Force_Constraint_i_j) *segment_number * (grids_per_segment-1), 1);
        
        Snopt_Constraint_CppCode = [Snopt_Constraint_CppCode Fsym_StrictF_Total];
        Snopt_iGfunjGvar_CppCode = [Snopt_iGfunjGvar_CppCode iGfunjGvar_StrictF_Total];
        Snopt_Jacobian_CppCode = [Snopt_Jacobian_CppCode Gsym_StrictF_Total];
        
        Snopt_Jacobian_Raw = Snopt_Jacobian_Raw + 9 * 4 * row_StrictF_no;
        
%     end
    
% end

%% 5. Fixed distance constraint

% for i = 1: segment_number     
    i = 1;
    [coef_A, coef_B, coef_C, coef_D, coef_M, coef_O] = Contact_Force_Constraint_Coef(mode_sequence(i));  
    syms coef_A coef_B coef_C coef_D coef_M coef_O real
    
%     for j = 1:grids_per_segment        
    for j = 1:2        
        
        % Contact constraints: one constraint is the certain force has to
        % be positive, while the other constraint is that certain distance
        % has to be zero or not
        
        eval(['rIx_new = rIx_',num2str(i),'_',num2str(j),';']);
        eval(['rIy_new = rIy_',num2str(i),'_',num2str(j),';']);
        eval(['theta_new = theta_',num2str(i),'_',num2str(j),';']);
        eval(['q1_new = q1_',num2str(i),'_',num2str(j),';']);
        eval(['q2_new = q2_',num2str(i),'_',num2str(j),';']);
        eval(['q3_new = q3_',num2str(i),'_',num2str(j),';']);
        eval(['q4_new = q4_',num2str(i),'_',num2str(j),';']);
        eval(['q5_new = q5_',num2str(i),'_',num2str(j),';']);
        eval(['q6_new = q6_',num2str(i),'_',num2str(j),';']);
        eval(['q7_new = q7_',num2str(i),'_',num2str(j),';']);
        eval(['q8_new = q8_',num2str(i),'_',num2str(j),';']);
        eval(['q9_new = q9_',num2str(i),'_',num2str(j),';']);
        eval(['q10_new = q10_',num2str(i),'_',num2str(j),';']);
        
        eval(['lamda_Ax_new = lamda_Ax_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Ay_new = lamda_Ay_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Bx_new = lamda_Bx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_By_new = lamda_By_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Cx_new = lamda_Cx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Cy_new = lamda_Cy_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Dx_new = lamda_Dx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Dy_new = lamda_Dy_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Mx_new = lamda_Mx_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_My_new = lamda_My_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Ox_new = lamda_Ox_',num2str(i),'_',num2str(j),';']);
        eval(['lamda_Oy_new = lamda_Oy_',num2str(i),'_',num2str(j),';']);
        
        rA_new = subs(rA, [q4,q5,q6,rIx,rIy,theta], [q4_new,q5_new,q6_new,rIx_new,rIy_new,theta_new]);
        rB_new = subs(rB, [q4,q5,q6,rIx,rIy,theta], [q4_new,q5_new,q6_new,rIx_new,rIy_new,theta_new]);
        rC_new = subs(rC, [q1,q2,q3,rIx,rIy,theta], [q1_new,q2_new,q3_new,rIx_new,rIy_new,theta_new]);
        rD_new = subs(rD, [q1,q2,q3,rIx,rIy,theta], [q1_new,q2_new,q3_new,rIx_new,rIy_new,theta_new]);
        rM_new = subs(rM, [q9,q10,rIx,rIy,theta],   [q9_new,q10_new,rIx_new,rIy_new,theta_new]);
        rO_new = subs(rO, [q7,q8,rIx,rIy,theta],    [q7_new,q8_new,rIx_new,rIy_new,theta_new]);     
        %%
        if j == 1          
            % At the start, the fixed horizontal distance will be used as a reference
            
            Hori_A_ref = rA_new(1);
            
            Hori_B_ref = rB_new(2);
            
            Hori_C_ref = rC_new(1);
            
            Hori_D_ref = rD_new(2);
            
            Hori_M_ref = rM_new(2);
            
            Hori_O_ref = rO_new(2);
            
            Fixed_Distance_Constraint_i_j = [];
        else          
            Fixed_Distance_Constraint_i_j =[Fixed_Distance_Constraint_i_j; coef_A * (Hori_A_ref - rA_new(1))];
            
            Fixed_Distance_Constraint_i_j =[Fixed_Distance_Constraint_i_j; coef_B * (Hori_B_ref - rB_new(2))];
            
            Fixed_Distance_Constraint_i_j =[Fixed_Distance_Constraint_i_j; coef_C * (Hori_C_ref - rC_new(1))];
            
            Fixed_Distance_Constraint_i_j =[Fixed_Distance_Constraint_i_j; coef_D * (Hori_D_ref - rD_new(2))];
            
            Fixed_Distance_Constraint_i_j =[Fixed_Distance_Constraint_i_j; coef_M * (Hori_M_ref - rM_new(2))];  
            
            Fixed_Distance_Constraint_i_j =[Fixed_Distance_Constraint_i_j; coef_O * (Hori_O_ref - rO_new(2))];
            
        end
    end
% end

Fsym_FixedDis_Total = Sym2C_Index_Converter_Loop_FixedDis(ccode(Fixed_Distance_Constraint_i_j), length(Snopt_Constraint_Raw));

[iGfunjGvar_FixedDis_Total, Gsym_FixedDis_Total, row_FixedDis_no] = Sym2C_Index_Converter_Loop_FixedDis_Jac(ccode(jacobian(Fixed_Distance_Constraint_i_j, Snopt_State)), length(Snopt_Constraint_Raw), Snopt_Jacobian_Raw);

[Snopt_Constraint_Bound, Snopt_Constraint_Raw]= constraint_bound_formulation(Snopt_Constraint_Bound, Snopt_Constraint_Raw, length(Fixed_Distance_Constraint_i_j) *segment_number * (grids_per_segment-1), 0);

Snopt_Constraint_CppCode = [Snopt_Constraint_CppCode Fsym_FixedDis_Total];
Snopt_iGfunjGvar_CppCode = [Snopt_iGfunjGvar_CppCode iGfunjGvar_FixedDis_Total];
Snopt_Jacobian_CppCode = [Snopt_Jacobian_CppCode Gsym_FixedDis_Total];

Snopt_Jacobian_Raw = Snopt_Jacobian_Raw + 9 * 4 * row_FixedDis_no;

%% The last constraint is the impact mapping constraint

Snopt_Impact_Constraint_tot = [];
Snopt_Impact_Constraint_Jac_tot = [];

% for i = 1:segment_number-1
    i = 1;
    j = grids_per_segment - 1;
    
    eval(['rIx_old = rIx_',num2str(i),'_',num2str(j+1),';']);
    eval(['rIy_old = rIy_',num2str(i),'_',num2str(j+1),';']);
    eval(['theta_old = theta_',num2str(i),'_',num2str(j+1),';']);
    eval(['q1_old = q1_',num2str(i),'_',num2str(j+1),';']);
    eval(['q2_old = q2_',num2str(i),'_',num2str(j+1),';']);
    eval(['q3_old = q3_',num2str(i),'_',num2str(j+1),';']);
    eval(['q4_old = q4_',num2str(i),'_',num2str(j+1),';']);
    eval(['q5_old = q5_',num2str(i),'_',num2str(j+1),';']);
    eval(['q6_old = q6_',num2str(i),'_',num2str(j+1),';']);
    eval(['q7_old = q7_',num2str(i),'_',num2str(j+1),';']);
    eval(['q8_old = q8_',num2str(i),'_',num2str(j+1),';']);
    eval(['q9_old = q9_',num2str(i),'_',num2str(j+1),';']);
    eval(['q10_old = q10_',num2str(i),'_',num2str(j+1),';']);
    
    eval(['rIxdot_old = rIxdot_',num2str(i),'_',num2str(j+1),';']);
    eval(['rIydot_old = rIydot_',num2str(i),'_',num2str(j+1),';']);
    eval(['thetadot_old = thetadot_',num2str(i),'_',num2str(j+1),';']);
    eval(['q1dot_old = q1dot_',num2str(i),'_',num2str(j+1),';']);
    eval(['q2dot_old = q2dot_',num2str(i),'_',num2str(j+1),';']);
    eval(['q3dot_old = q3dot_',num2str(i),'_',num2str(j+1),';']);
    eval(['q4dot_old = q4dot_',num2str(i),'_',num2str(j+1),';']);
    eval(['q5dot_old = q5dot_',num2str(i),'_',num2str(j+1),';']);
    eval(['q6dot_old = q6dot_',num2str(i),'_',num2str(j+1),';']);
    eval(['q7dot_old = q7dot_',num2str(i),'_',num2str(j+1),';']);
    eval(['q8dot_old = q8dot_',num2str(i),'_',num2str(j+1),';']);
    eval(['q9dot_old = q9dot_',num2str(i),'_',num2str(j+1),';']);
    eval(['q10dot_old = q10dot_',num2str(i),'_',num2str(j+1),';']);  
    
    
    eval(['rIx_new = rIx_',num2str(i+1),'_',num2str(1),';']);
    eval(['rIy_new = rIy_',num2str(i+1),'_',num2str(1),';']);
    eval(['theta_new = theta_',num2str(i+1),'_',num2str(1),';']);
    eval(['q1_new = q1_',num2str(i+1),'_',num2str(1),';']);
    eval(['q2_new = q2_',num2str(i+1),'_',num2str(1),';']);
    eval(['q3_new = q3_',num2str(i+1),'_',num2str(1),';']);
    eval(['q4_new = q4_',num2str(i+1),'_',num2str(1),';']);
    eval(['q5_new = q5_',num2str(i+1),'_',num2str(1),';']);
    eval(['q6_new = q6_',num2str(i+1),'_',num2str(1),';']);
    eval(['q7_new = q7_',num2str(i+1),'_',num2str(1),';']);
    eval(['q8_new = q8_',num2str(i+1),'_',num2str(1),';']);
    eval(['q9_new = q9_',num2str(i+1),'_',num2str(1),';']);
    eval(['q10_new = q10_',num2str(i+1),'_',num2str(1),';']);
    
    eval(['rIxdot_new = rIxdot_',num2str(i+1),'_',num2str(1),';']);
    eval(['rIydot_new = rIydot_',num2str(i+1),'_',num2str(1),';']);
    eval(['thetadot_new = thetadot_',num2str(i+1),'_',num2str(1),';']);
    eval(['q1dot_new = q1dot_',num2str(i+1),'_',num2str(1),';']);
    eval(['q2dot_new = q2dot_',num2str(i+1),'_',num2str(1),';']);
    eval(['q3dot_new = q3dot_',num2str(i+1),'_',num2str(1),';']);
    eval(['q4dot_new = q4dot_',num2str(i+1),'_',num2str(1),';']);
    eval(['q5dot_new = q5dot_',num2str(i+1),'_',num2str(1),';']);
    eval(['q6dot_new = q6dot_',num2str(i+1),'_',num2str(1),';']);
    eval(['q7dot_new = q7dot_',num2str(i+1),'_',num2str(1),';']);
    eval(['q8dot_new = q8dot_',num2str(i+1),'_',num2str(1),';']);
    eval(['q9dot_new = q9dot_',num2str(i+1),'_',num2str(1),';']);
    eval(['q10dot_new = q10dot_',num2str(i+1),'_',num2str(1),';']);
    
    Impact_Mapping_State_Constraint = [rIx_new - rIx_old;...
        rIy_new - rIy_old;...
        theta_new - theta_old;...
        q1_new - q1_old;...
        q2_new - q2_old;...
        q3_new - q3_old;...
        q4_new - q4_old;...
        q5_new - q5_old;...
        q6_new - q6_old;...
        q7_new - q7_old;...
        q8_new - q8_old;...
        q9_new - q9_old;...
        q10_new - q10_old];
    
    % Here is the choice of the Jacobian matrix
    
    Jac_Full_New = subs(Jac_Full,[q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta],[q1_new,q2_new,q3_new,q4_new,q5_new,q6_new,q7_new,q8_new,q9_new,q10_new,theta_new]);
    
    Jac_t_index = Selection_Matrix_Cal(mode_sequence(i+1));
    
    Full_rank_Jac_t = Jac_Full_New(Jac_t_index(1),:);
    
    for m = 1:length(Jac_t_index)-1
        Full_rank_Jac_temp = [Full_rank_Jac_t; Jac_Full_New(Jac_t_index(m+1),:)];
        [m_rank,~] = size(Full_rank_Jac_temp);
        if rank(Full_rank_Jac_temp) == m_rank
            Full_rank_Jac_t = Full_rank_Jac_temp;
        end
    end
    
    Rank_Jac_Full = rank(Full_rank_Jac_temp);
    
    qdot_old = [rIxdot_old; rIydot_old; thetadot_old; q1dot_old; q2dot_old; q3dot_old; q4dot_old; q5dot_old; q6dot_old; q7dot_old; q8dot_old; q9dot_old; q10dot_old];
    qdot_new = [rIxdot_new; rIydot_new; thetadot_new; q1dot_new; q2dot_new; q3dot_new; q4dot_new; q5dot_new; q6dot_new; q7dot_new; q8dot_new; q9dot_new; q10dot_new];
    
    D_q_i_j = subs(D_q,[q2,q3,q4,q5,q7,q8,q9,q10,theta], [q2_new,q3_new,q4_new,q5_new,q7_new,q8_new,q9_new,q10_new,theta_new]);
    
    % To avoid the extraordinary computational burden, we would rely on
    % the optimization solver to find a solution instead of specifying
    % it directly
    
    eval(['lambar_i = [',' lambar_Ax_',num2str(i),' lambar_Ay_',num2str(i),...
        ' lambar_Bx_',num2str(i),' lambar_By_',num2str(i),...
        ' lambar_Cx_',num2str(i),' lambar_Cy_',num2str(i),...
        ' lambar_Dx_',num2str(i),' lambar_Dy_',num2str(i),...
        ' lambar_Mx_',num2str(i),' lambar_My_',num2str(i),...
        ' lambar_Ox_',num2str(i),' lambar_Oy_',num2str(i),'];']);
    
    Impact_Mapping_Statedot_Constraint = D_q_i_j * (qdot_new - qdot_old) - Jac_Full_New.'* lambar_i.';
    Impact_Mapping_Statedot_Jac_Constraint = D_q_i_j * qdot_new;
        
    Snopt_Impact_Constraint_tot = [Snopt_Impact_Constraint_tot; Impact_Mapping_State_Constraint];
    Snopt_Impact_Constraint_Jac_tot = [Snopt_Impact_Constraint_Jac_tot; jacobian(Impact_Mapping_State_Constraint, Snopt_State)];
       
    
    Snopt_Impact_Constraint_tot = [Snopt_Impact_Constraint_tot; Impact_Mapping_Statedot_Constraint];
    Snopt_Impact_Constraint_tot = [Snopt_Impact_Constraint_tot; Impact_Mapping_Statedot_Jac_Constraint];
    
    Snopt_Impact_Constraint_Jac_tot = [Snopt_Impact_Constraint_Jac_tot; jacobian(Impact_Mapping_Statedot_Constraint, Snopt_State)];
    Snopt_Impact_Constraint_Jac_tot = [Snopt_Impact_Constraint_Jac_tot; jacobian(Impact_Mapping_Statedot_Jac_Constraint, Snopt_State)];
    
        
    Fsym_ImpactM_Total = Sym2C_Index_Converter_Loop_ImpactM(ccode(Snopt_Impact_Constraint_tot), length(Snopt_Constraint_Raw));
    
    [iGfunjGvar_ImpactM_Total, Gsym_ImpactM_Total, row_ImpactM_no] = Sym2C_Index_Converter_Loop_ImpactM_Jac(ccode(jacobian(Snopt_Impact_Constraint_tot, Snopt_State)), length(Snopt_Constraint_Raw), Snopt_Jacobian_Raw);
    
    [Snopt_Constraint_Bound, Snopt_Constraint_Raw]= constraint_bound_formulation(Snopt_Constraint_Bound, Snopt_Constraint_Raw, length(Snopt_Impact_Constraint_tot) * (segment_number -1), 0);
    
    Snopt_Constraint_CppCode = [Snopt_Constraint_CppCode Fsym_ImpactM_Total];
    Snopt_iGfunjGvar_CppCode = [Snopt_iGfunjGvar_CppCode iGfunjGvar_ImpactM_Total];
    Snopt_Jacobian_CppCode = [Snopt_Jacobian_CppCode Gsym_ImpactM_Total];
    
    Snopt_Jacobian_Raw = Snopt_Jacobian_Raw +  3 * row_ImpactM_no;
    
% end

% The final step is to add the objective function at the ending time

eval(['rIx_new = rIx_',num2str(4),'_',num2str(10),';']);
eval(['rIy_new = rIy_',num2str(4),'_',num2str(10),';']);
eval(['theta_new = theta_',num2str(4),'_',num2str(10),';']);
eval(['q1_new = q1_',num2str(4),'_',num2str(10),';']);
eval(['q2_new = q2_',num2str(4),'_',num2str(10),';']);
eval(['q3_new = q3_',num2str(4),'_',num2str(10),';']);
eval(['q4_new = q4_',num2str(4),'_',num2str(10),';']);
eval(['q5_new = q5_',num2str(4),'_',num2str(10),';']);
eval(['q6_new = q6_',num2str(4),'_',num2str(10),';']);
eval(['q7_new = q7_',num2str(4),'_',num2str(10),';']);
eval(['q8_new = q8_',num2str(4),'_',num2str(10),';']);
eval(['q9_new = q9_',num2str(4),'_',num2str(10),';']);
eval(['q10_new = q10_',num2str(4),'_',num2str(10),';']);

eval(['rIxdot_new = rIxdot_',num2str(4),'_',num2str(10),';']);
eval(['rIydot_new = rIydot_',num2str(4),'_',num2str(10),';']);
eval(['thetadot_new = thetadot_',num2str(4),'_',num2str(10),';']);
eval(['q1dot_new = q1dot_',num2str(4),'_',num2str(10),';']);
eval(['q2dot_new = q2dot_',num2str(4),'_',num2str(10),';']);
eval(['q3dot_new = q3dot_',num2str(4),'_',num2str(10),';']);
eval(['q4dot_new = q4dot_',num2str(4),'_',num2str(10),';']);
eval(['q5dot_new = q5dot_',num2str(4),'_',num2str(10),';']);
eval(['q6dot_new = q6dot_',num2str(4),'_',num2str(10),';']);
eval(['q7dot_new = q7dot_',num2str(4),'_',num2str(10),';']);
eval(['q8dot_new = q8dot_',num2str(4),'_',num2str(10),';']);
eval(['q9dot_new = q9dot_',num2str(4),'_',num2str(10),';']);
eval(['q10dot_new = q10dot_',num2str(4),'_',num2str(10),';']);

T_i_j = subs(Q.T,[q2,q3,q4,q5,q7,q8,q9,q10,...
                 q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,...
                 rIxdot,rIydot,thetadot,theta],...
                 [q2_new,q3_new,q4_new,q5_new,q7_new,q8_new,q9_new,q10_new,...
                 q10dot_new,q1dot_new,q2dot_new,q3dot_new,q4dot_new,q5dot_new,q6dot_new,q7dot_new,q8dot_new,q9dot_new,...
                 rIxdot_new,rIydot_new,thetadot_new,theta_new]);
% Here we come to the ending time of the current segments and it is time to add the impact mapping constraint into the new state

Fsym_Obj = Sym2C_Index_Converter_Obj(ccode(T_i_j), length(Snopt_Constraint_Raw));

[iGfunjGvar_Obj, Gsym_Obj, row_Obj_no] = Sym2C_Index_Converter_Obj_Jac(ccode(jacobian(T_i_j, Snopt_State)), length(Snopt_Constraint_Raw), Snopt_Jacobian_Raw);

[Snopt_Constraint_Bound, Snopt_Constraint_Raw]= constraint_bound_formulation(Snopt_Constraint_Bound, Snopt_Constraint_Raw, 1, 1);

Snopt_Constraint_CppCode = [Snopt_Constraint_CppCode Fsym_Obj];
Snopt_iGfunjGvar_CppCode = [Snopt_iGfunjGvar_CppCode iGfunjGvar_Obj];
Snopt_Jacobian_CppCode = [Snopt_Jacobian_CppCode Gsym_Obj];

Snopt_Jacobian_Raw = Snopt_Jacobian_Raw +   row_Obj_no;




% Snopt_Full_Constraint_Jac = jacobian(Snopt_Constraint_tot, Snopt_State);
%
% [iGfunjGvar_start_Total, Gsym_start_Total ] = Sym2C_Index_Converter(ccode(Snopt_Full_Constraint_Jac));
%
% sprintf(iGfunjGvar_start_Total);
%
% sprintf(Gsym_start_Total);

contact_sequence_char = '1_11_111_111_';

time_char = datestr(now,'yyyy-mm-dd HH_MM_SS');

mat_char = '.mat';

final_file_name = [contact_sequence_char time_char mat_char];

save(final_file_name)

% Get em to work in Snopt c++

% This is the script for the definition of variables from x
% Extrac_tot = Snopt_DoubleFromX( Snopt_State );
% sprintf(Extrac_tot)

% The bounds for x
sprintf(Snopt_State_Bound)

% The bounds for constraint equations
sprintf(Snopt_Constraint_Bound)

sprintf(Snopt_Constraint_CppCode)

sprintf(Snopt_iGfunjGvar_CppCode)

sprintf(Snopt_Jacobian_CppCode)

end

function Jac_t_index = Selection_Matrix_Cal(contact_status)

% This function is used to select the proper row of the Jac_Full matrix
Jac_t_index = [];
mode_seq_str = num2str(contact_status);
for i = 1:length(mode_seq_str)
    mode_seq_str_i = mode_seq_str(i);
    if mode_seq_str_i == '1'
        Jac_t_index = [Jac_t_index 2*i-1 2*i];
    end
end
end
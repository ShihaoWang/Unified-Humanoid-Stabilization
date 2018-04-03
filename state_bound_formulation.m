function [ Snopt_State_Bound, Snopt_State_Raw ]= state_bound_formulation(Snopt_State_Bound, Snopt_State_Raw, State_Type)

% 1. State and Statedot bounds!

% xlow[0]
rIx_low = '-infBnd'; 
rIx_upp = 'infBnd'; 

% xlow[1]
rIy_low = '0.0'; 
rIy_upp = 'infBnd'; 

% xlow[2]
theta_low = '-PI/2.0'; 
theta_upp = 'PI*1.0';

% xlow[3]
q1_low = '0.0';
q1_upp = 'PI';

% xlow[4]
q2_low = '0.0';
q2_upp = 'PI';

% xlow[5]
q3_low = '0.0';
q3_upp = '3.0 * PI/2.0';

% xlow[6]
q4_low = '0.0';
q4_upp = '3.0 * PI/2.0';

% xlow[7]
q5_low = '0.0';
q5_upp = 'PI';

% xlow[8]
q6_low = '0.0';
q6_upp = 'PI';

% xlow[9]
q7_low = '-PI';
q7_upp = 'PI';

% xlow[10]
q8_low = '0.0';
q8_upp = 'PI';

% xlow[11]
q9_low = '-PI';
q9_upp = 'PI';

% xlow[12]
q10_low = '0.0';
q10_upp = 'PI';

% xlow[13]
rIxdot_low = '-Trans_Max';
rIxdot_upp = 'Trans_Max';

% xlow[14]
rIydot_low = '-Trans_Max';
rIydot_upp = 'Trans_Max';

% xlow[15]
thetadot_low = '-AngRate_Max';
thetadot_upp = 'AngRate_Max';

% xlow[16]
q1dot_low = thetadot_low;
q1dot_upp = thetadot_upp;

% xlow[17]
q2dot_low = q1dot_low;
q2dot_upp = q1dot_upp;

% xlow[17]
q3dot_low = q2dot_low;
q3dot_upp = q2dot_upp;

% xlow[17]
q4dot_low = q3dot_low;
q4dot_upp = q3dot_upp;

% xlow[17]
q5dot_low = q4dot_low;
q5dot_upp = q4dot_upp;

% xlow[17]
q6dot_low = q5dot_low;
q6dot_upp = q5dot_upp;

% xlow[17]
q7dot_low = q6dot_low;
q7dot_upp = q6dot_upp;

% xlow[17]
q8dot_low = q7dot_low;
q8dot_upp = q7dot_upp;

% xlow[17]
q9dot_low = q8dot_low;
q9dot_upp = q8dot_upp;

% xlow[17]
q10dot_low = q9dot_low;
q10dot_upp = q9dot_upp;

% 2. The contact force bound

lamda_Ax_low = '-infBnd';
lamda_Ax_upp = 'infBnd';

lamda_Ay_low = '0.0';
lamda_Ay_upp = 'infBnd';

lamda_Bx_low = '-infBnd';
lamda_Bx_upp = 'infBnd';

lamda_By_low = '0.0';
lamda_By_upp = 'infBnd';

lamda_Cx_low = '-infBnd';
lamda_Cx_upp = 'infBnd';

lamda_Cy_low = '0.0';
lamda_Cy_upp = 'infBnd';

lamda_Dx_low = '-infBnd';
lamda_Dx_upp = 'infBnd';

lamda_Dy_low = '0.0';
lamda_Dy_upp = 'infBnd';

lamda_Mx_low = '-infBnd';
lamda_Mx_upp = '0.0';

lamda_My_low = '-infBnd';
lamda_My_upp = 'infBnd';

lamda_Ox_low = '-infBnd';
lamda_Ox_upp = '0.0';

lamda_Oy_low = '-infBnd';
lamda_Oy_upp = 'infBnd';

% 3. Control torque bound

u1_low = '-Torque_Max';
u1_upp = 'Torque_Max';

u2_low = '-Torque_Max';
u2_upp = 'Torque_Max';

u3_low = '-Torque_Max';
u3_upp = 'Torque_Max';

u4_low = '-Torque_Max';
u4_upp = 'Torque_Max';

u5_low = '-Torque_Max';
u5_upp = 'Torque_Max';

u6_low = '-Torque_Max';
u6_upp = 'Torque_Max';

u7_low = '-Torque_Max';
u7_upp = 'Torque_Max';

u8_low = '-Torque_Max';
u8_upp = 'Torque_Max';

u9_low = '-Torque_Max';
u9_upp = 'Torque_Max';

u10_low = '-Torque_Max';
u10_upp = 'Torque_Max';

% 4. Impulse bounds

lambar_Ax_low = '-infBnd';
lambar_Ax_upp = 'infBnd';

lambar_Ay_low = '0.0';
lambar_Ay_upp = 'infBnd';

lambar_Bx_low = '-infBnd';
lambar_Bx_upp = 'infBnd';

lambar_By_low = '0.0';
lambar_By_upp = 'infBnd';

lambar_Cx_low = '-infBnd';
lambar_Cx_upp = 'infBnd';

lambar_Cy_low = '0.0';
lambar_Cy_upp = 'infBnd';

lambar_Dx_low = '-infBnd';
lambar_Dx_upp = 'infBnd';

lambar_Dy_low = '0.0';
lambar_Dy_upp = 'infBnd';

lambar_Mx_low = '-infBnd';
lambar_Mx_upp = '0.0';

lambar_My_low = '-infBnd';
lambar_My_upp = 'infBnd';

lambar_Ox_low = '-infBnd';
lambar_Ox_upp = '0.0';

lambar_Oy_low = '-infBnd';
lambar_Oy_upp = 'infBnd';

current_index = length(Snopt_State_Raw);

State_bound_char = char('rIx', 'rIy', 'theta', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7', 'q8', 'q9', 'q10');
Cntrl_bound_char = char('u1', 'u2', 'u3', 'u4', 'u5', 'u6', 'u7', 'u8', 'u9', 'u10');
Lamda_bound_char = char('lamda_Ax', 'lamda_Ay', 'lamda_Bx', 'lamda_By', 'lamda_Cx', 'lamda_Cy', ...
                        'lamda_Dx', 'lamda_Dy', 'lamda_Mx', 'lamda_My', 'lamda_Ox', 'lamda_Oy');
Lambar_bound_char = char('lambar_Ax', 'lambar_Ay', 'lambar_Bx', 'lambar_By', 'lambar_Cx', 'lambar_Cy',...
                         'lambar_Dx', 'lambar_Dy', 'lambar_Mx', 'lambar_My', 'lambar_Ox', 'lambar_Oy');

if State_Type == 1
    
    for i = 1:13
        variable_i_low = genvarname(strcat(' ', State_bound_char(i,:),'_low')); 
        variable_i_upp = genvarname(strcat(' ', State_bound_char(i,:),'_upp'));
        evalc(['variable_i_low', ' = ', variable_i_low]);
        evalc(['variable_i_upp', ' = ', variable_i_upp]);
        Snopt_State_Bound = Snopt_State_Bound_Sub(Snopt_State_Bound, current_index, variable_i_low, variable_i_upp);
        current_index = current_index + 1;
    end
    
    for i = 1:13
        variable_i_low = genvarname(strcat(' ', State_bound_char(i,:),'dot_low')); 
        variable_i_upp = genvarname(strcat(' ', State_bound_char(i,:),'dot_upp'));
        evalc(['variable_i_low', ' = ', variable_i_low]);
        evalc(['variable_i_upp', ' = ', variable_i_upp]);
        Snopt_State_Bound = Snopt_State_Bound_Sub(Snopt_State_Bound, current_index, variable_i_low, variable_i_upp);
        current_index = current_index + 1;
    end
   
    Snopt_State_Raw = [Snopt_State_Raw; ones(26,1)];
    
else
    if State_Type == 2
        % In this case, it is the control variable       
        for i = 1:10
            variable_i_low = genvarname(strcat(' ', Cntrl_bound_char(i,:),'_low'));
            variable_i_upp = genvarname(strcat(' ', Cntrl_bound_char(i,:),'_upp'));
            evalc(['variable_i_low', ' = ', variable_i_low]);
            evalc(['variable_i_upp', ' = ', variable_i_upp]);
            Snopt_State_Bound = Snopt_State_Bound_Sub(Snopt_State_Bound, current_index, variable_i_low, variable_i_upp);
            current_index = current_index + 1;
        end
        Snopt_State_Raw = [Snopt_State_Raw; ones(10,1)];    
    else
        if State_Type == 3
            % This is the contact force
            for i = 1:12
                variable_i_low = genvarname(strcat(' ', Lamda_bound_char(i,:),'_low'));
                variable_i_upp = genvarname(strcat(' ', Lamda_bound_char(i,:),'_upp'));
                evalc(['variable_i_low', ' = ', variable_i_low]);
                evalc(['variable_i_upp', ' = ', variable_i_upp]);
                Snopt_State_Bound = Snopt_State_Bound_Sub(Snopt_State_Bound, current_index, variable_i_low, variable_i_upp);
                current_index = current_index + 1;
            end        
            Snopt_State_Raw = [Snopt_State_Raw; ones(12,1)];
            
        else
            % This is the impulse
            for i = 1:12
                variable_i_low = genvarname(strcat(' ', Lambar_bound_char(i,:),'_low'));
                variable_i_upp = genvarname(strcat(' ', Lambar_bound_char(i,:),'_upp'));
                evalc(['variable_i_low', ' = ', variable_i_low]);
                evalc(['variable_i_upp', ' = ', variable_i_upp]);
                Snopt_State_Bound = Snopt_State_Bound_Sub(Snopt_State_Bound, current_index, variable_i_low, variable_i_upp);
                current_index = current_index + 1;
            end         
            Snopt_State_Raw = [Snopt_State_Raw; ones(12,1)];
        end
    end
    
end
end
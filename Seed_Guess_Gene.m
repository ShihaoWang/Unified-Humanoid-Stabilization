function [Opt_Seed, Opt_Lowbd, Opt_Uppbd, P]= Seed_Guess_Gene(sigma_i, x_i, sigma_i_child, P)

% This function is used to generate the seed used for the optimization

%% The main idea is to generate a sequence of transition configurations then reversely compute the control torques
% Since sigma_i and x_i are selected from the Frontier set, the
% coresponding constraints have already been satisfied.
% The problem becomes how to address the constraint of sigma_child

%% The first step is to generate a feasible robot state that satisfies the sigma_child
RobotState_LowBd = P.RobotState_LowBd;
RobotState_UppBd = P.RobotState_UppBd;
Seed_Opt = optimoptions(@fmincon,'Display','off','Algorithm','sqp','MaxIterations',2500,'OptimalityTolerance',1e-8,'MaxFunctionEvaluations',10000);
P.Refer0 = x_i;

% Here an iterative optimization idea is adopted to minimize the difference
% between the new optimized translation to the initial translation position
% P.Iter_Offset = 0.5;
exitflag = 0;  % Set the default value to be 0
P.sigma_i_child = sigma_i_child;
while (exitflag~=1)&&(exitflag~=2)
%% exitflag
%               1:          First-order optimality measure was less than options.OptimalityTolerance, 
%                           and maximum constraint violation was less than options.ConstraintTolerance.
%               0:          Number of iterations exceeded options.MaxIterations or number of function 
%                           evaluations exceeded options.MaxFunctionEvaluations.
%               -1:         Stopped by an output function or plot function.
%               -2:         No feasible point was found.
    [x_i_child_Opt,fval,exitflag,~]= fmincon(@Seed_Conf_Obj,x_i,[],[],[],[],RobotState_LowBd,RobotState_UppBd,@Seed_Conf_Constraint,Seed_Opt, P);
%     P.Iter_Offset = P.Iter_Offset + 0.1;
    x_i = x_i_child_Opt;
%     Single_Frame_Plot(x_i_child_Opt, P)
end
P.Opt0 = x_i;   % This is the goal configuratioin at this step
% Single_Frame_Plot(P.Opt0, P);

%% Here the desired configuration has been satisfied, then a seed control needs to be formulated
% Then a linear interpolation can be calculated based on the sigma_i
Tme_Seed = P.Tme_Seed;
Ctrl_No = P.Ctrl_No;

% Initial state of the robot
rIx_0 = P.Refer0(1);      rIy_0 = P.Refer0(2);      theta_0 = P.Refer0(3);
q1_0 = P.Refer0(4);       q2_0 = P.Refer0(5);       q3_0 = P.Refer0(6);
q4_0 = P.Refer0(7);       q5_0 = P.Refer0(8);       q6_0 = P.Refer0(9);
q7_0 = P.Refer0(10);      q8_0 = P.Refer0(11);      q9_0 = P.Refer0(12);     q10_0 = P.Refer0(13);  

rIxdot_0 = P.Refer0(1+13);      rIydot_0 = P.Refer0(2+13);      thetadot_0 = P.Refer0(3+13);
q1dot_0 = P.Refer0(4+13);       q2dot_0 = P.Refer0(5+13);       q3dot_0 = P.Refer0(6+13);
q4dot_0 = P.Refer0(7+13);       q5dot_0 = P.Refer0(8+13);       q6dot_0 = P.Refer0(9+13);
q7dot_0 = P.Refer0(10+13);      q8dot_0 = P.Refer0(11+13);      q9dot_0 = P.Refer0(12+13);     q10dot_0 = P.Refer0(13+13);  

x_0 = [rIx_0 rIy_0 theta_0 q1_0 q2_0 q3_0 q4_0 q5_0 q6_0 q7_0 q8_0 q9_0 q10_0...
       rIxdot_0 rIydot_0 thetadot_0 q1dot_0 q2dot_0 q3dot_0 q4dot_0 q5dot_0 q6dot_0 q7dot_0 q8dot_0 q9dot_0 q10dot_0]';

% Desired state of the robot
x0 = P.Opt0;
rIx = x0(1);             rIy = x0(2);             theta = x0(3);
q1 = x0(4);              q2 = x0(5);              q3 = x0(6);
q4 = x0(7);              q5 = x0(8);              q6 = x0(9);
q7 = x0(10);             q8 = x0(11);             q9 = x0(12);
q10 = x0(13);

rIxdot = x0(1+13);          rIydot = x0(2+13);          thetadot = x0(3+13);
q1dot = x0(4+13);           q2dot = x0(5+13);           q3dot = x0(6+13);
q4dot = x0(7+13);           q5dot = x0(8+13);           q6dot = x0(9+13);
q7dot = x0(10+13);          q8dot = x0(11+13);          q9dot = x0(12+13);
q10dot = x0(13+13);

x_des = [rIx rIy theta q1 q2 q3 q4 q5 q6 q7 q8 q9 q10...
         rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot]';

%% The following optimization is conducted with a transcription approach     
t_span = [0 Tme_Seed];

x_span = [x_0 x_des];
Holo_Ind = Holo_Ind_fn(P.sigma_i);

t_intp_span = linspace(t_span(1),t_span(2),Ctrl_No);

x_intp = spline(t_span, x_span, t_intp_span);

u_array = [];

for i = 1:Ctrl_No
    % Control formulation at each time
    x0 = x_intp(:,i);   
    rIx = x0(1);             rIy = x0(2);             theta = x0(3);
    q1 = x0(4);              q2 = x0(5);              q3 = x0(6);
    q4 = x0(7);              q5 = x0(8);              q6 = x0(9);
    q7 = x0(10);             q8 = x0(11);             q9 = x0(12);
    q10 = x0(13);    
    x_state = x0(1:13,:);    
    rIxdot = x0(1+13);          rIydot = x0(2+13);          thetadot = x0(3+13);
    q1dot = x0(4+13);           q2dot = x0(5+13);           q3dot = x0(6+13);
    q4dot = x0(7+13);           q5dot = x0(8+13);           q6dot = x0(9+13);
    q7dot = x0(10+13);          q8dot = x0(11+13);          q9dot = x0(12+13);
    q10dot = x0(13+13);    
    if i == Ctrl_No
        xdotp1 = xdot;
        x_statep1 = x_state; 
    else
        x0p1 = x_intp(:,i+1);
        x_statep1 = x0p1(1:13,:);
        rIxdotp1 = x0p1(1+13);          rIydotp1 = x0p1(2+13);          thetadotp1 = x0p1(3+13);
        q1dotp1 = x0p1(4+13);           q2dotp1 = x0p1(5+13);           q3dotp1 = x0p1(6+13);
        q4dotp1 = x0p1(7+13);           q5dotp1 = x0p1(8+13);           q6dotp1 = x0p1(9+13);
        q7dotp1 = x0p1(10+13);          q8dotp1 = x0p1(11+13);          q9dotp1 = x0p1(12+13);
        q10dotp1 = x0p1(13+13);
        xdotp1 = [rIxdotp1 rIydotp1 thetadotp1 q1dotp1 q2dotp1 q3dotp1 q4dotp1 q5dotp1 q6dotp1 q7dotp1 q8dotp1 q9dotp1 q10dotp1]';       
    end
    xdot = [rIxdot rIydot thetadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot]';
    delta_t = t_intp_span(2) - t_intp_span(1);    
    qddot = (x_statep1 - x_state - xdot * delta_t)/(1/2 * delta_t^2) ;
    D_q = P.D_q_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
    B_q = P.B_q_fn();
    C_q_qdot = P.C_q_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);     
    if sum(Holo_Ind)==0
        u_i = B_q\(D_q * qddot + C_q_qdot);       
    else
        % Full rank distillation
        Jac_Full = P.Jac_Full_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,theta);
        Jacdot_qdot = P.Jacdot_qdot_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,thetadot,theta);
        
        Active_Rows = Jac_Ind_Fn(Holo_Ind);
        Jac_Full = Jac_Full(Active_Rows,:);
        Jacdot_qdot_Full = Jacdot_qdot(Active_Rows,:);
        
        Jac_Temp = Jac_Full';
        [~,colind] = rref(Jac_Temp);
        P.Active_In = Active_Rows(colind);
        Jac = Jac_Temp(:, colind)';
        Jacdot_qdot_temp = Jacdot_qdot_Full';
        Jacdot = Jacdot_qdot_temp(:, colind)';       
        P.D_q = D_q;
        P.B_q = B_q;
        P.C_q = C_q_qdot;
        P.qddot = qddot;
        P.Jac = Jac;
        P.Jacdot = Jacdot;      
        u_seed_i = B_q\(D_q * qddot + C_q_qdot);   

        Seed_Sub_Opt = optimoptions(@fmincon,'Display','off','Algorithm','sqp','MaxIterations',2500,'OptimalityTolerance',1e-8,'MaxFunctionEvaluations',10000);
        
        [u_i_opt,~,~,~]= fmincon(@Seed_Ctrl_Obj,u_seed_i,[],[],[],[],P.Opt_Var_LowBd,P.Opt_Var_UppBd,@Seed_Ctrl_Constraint,Seed_Sub_Opt, P);
        u_i = u_i_opt;
    end  
    u_array = [u_array u_i];
end
StateNDot_Seed = reshape(x_intp, length(C_q_qdot)*2*Ctrl_No, 1);
Ctrl_Seed = reshape(u_array, length(u_i) *Ctrl_No,1 );
Opt_Seed = [Tme_Seed; StateNDot_Seed; Ctrl_Seed];

Ctrl_LowBd = P.Opt_Var_LowBd;
Ctrl_UppBd = P.Opt_Var_UppBd;
Opt_Lowbd = 0.105;
Opt_Uppbd = inf; 

Opt_Lowbd = [Opt_Lowbd; repmat(RobotState_LowBd', [Ctrl_No,1]); repmat(Ctrl_LowBd, [Ctrl_No,1])];
Opt_Uppbd = [Opt_Uppbd; repmat(RobotState_UppBd', [Ctrl_No,1]); repmat(Ctrl_UppBd, [Ctrl_No,1])];

end
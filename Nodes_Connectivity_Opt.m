function Nodes_Connectivity_Opt(sigma_i, x_i, sigma_i_child, P)

% This function test the connectivity between a certain node and its child
% node

% The current idea is to make sure of the multiple shooting method since in
% this case the dynamics constraints are satisfied automatically

% The main idea to reach the sigma_child at the end step while minimizing the kinetic energy

P.sigma_i = sigma_i;
P.sigma_i_child = sigma_i_child; 
P.x_i = x_i;
P.Ctrl_No = 20;
P.eps = 0.001;

P.Tme_Seed = 2;

tau1_max = 100;             tau2_max = 100;             tau3_max = 100;
tau4_max = 100;             tau5_max = 100;             tau6_max = 100;
tau7_max = 60;              tau8_max = 50;              tau9_max = 60;             tau10_max = 50;

Opt_Var_LowBd_i = -[tau1_max tau2_max tau3_max tau4_max tau5_max tau6_max tau7_max tau8_max tau9_max tau10_max ]';
Opt_Var_UppBd_i = - Opt_Var_LowBd_i;

P.Opt_Var_LowBd = Opt_Var_LowBd_i;
P.Opt_Var_UppBd = Opt_Var_UppBd_i;

[Opt_Seed, Opt_Lowbd, Opt_Uppbd, P]= Seed_Guess_Gene(sigma_i, x_i, sigma_i_child, P);

Nodes_Connectivity_Opt = optimoptions(@fmincon,'Display','iter','Algorithm','sqp','MaxIterations',2500,'MaxFunctionEvaluations',inf);

Var_Opt = fmincon(@Nodes_Connectivity_Obj,Opt_Seed,[],[],[],[],Opt_Lowbd,Opt_Uppbd,@Nodes_Connectivity_Constraint,Nodes_Connectivity_Opt, P);

end
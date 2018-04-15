function Simu_Main()
% This is the main program to run the whole simulation
%% First is to validate the initial condition
% [P, sigma0, x0] = Default_Init('show');
[P, sigma0, x0] = Default_Init();
sigma_i = sigma0;
x_i = x0;
sigma_i_child = [1 1 0 0];
Nodes_Connectivity_Opt(sigma_i, x_i, sigma_i_child, P)
end
function [c,ceq, Dc, Dceq] = Nodes_Connectivity_Constraint_Grad(z, P)
[c,ceq] = Nodes_Connectivity_Constraint(z, P);
Jac = admDiffFor(@f, 1, A, c);



end


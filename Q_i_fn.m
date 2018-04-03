function  Q_i = Q_i_fn(knots, i, p, P_ip1, P_i)

% This function is used to calculate the derivative of a given spline
% Inputs: knots is the division of the domain
%         i is the index
%         p is the order of the basis function
%         P_ip1 is the coefficient at i+1
%         P_i   is the coefficient at i

u_ip1 = knots(i+1);

u_ippp1 = knots(i+p+1);  % i + p + 1

Q_i = p/(u_ippp1 - u_ip1) * (P_ip1 - P_i);

end

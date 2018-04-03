function [x, info] = ipopt_example_trial()

% This function is used to test the basic idea to make use of the ipopt
% solver.

x0         = [-3  -1];   % The starting point.
options.cl = [ -inf -inf];             % Lower bounds on constraints.
options.cu = [ 3 0  ];             % Upper bounds on constraints.

options.ipopt.hessian_approximation = 'limited-memory';
options.ipopt.limited_memory_update_type = 'bfgs'; % {bfgs}, sr1
options.ipopt.limited_memory_max_history = 6; % {6}

% % Hessian; exact
% options.ipopt.hessian_approximation = 'exact';
% % Derivative test (set max_iter = 0 if ONLY the derivative check is to be performed)
% options.ipopt.derivative_test = 'only-second-order'; % {none}, first-order, ...
% % second-order, only-second-order
% options.ipopt.derivative_test_perturbation = 3e-6; % {1e-8}, [3e-6]

options.ipopt.mu_strategy = 'adaptive';
options.ipopt.print_level = 0;
options.ipopt.tol         = 1e-7;
options.ipopt.max_iter    = 200;


% The callback functions.
funcs.objective         = @objective;
funcs.gradient          = @gradient;
funcs.constraints        = @constraint;
funcs.jacobian          = @jacobian;
funcs.jacobianstructure = @jacobian;
funcs.hessian           = @hessian;
funcs.hessianstructure  = @hessianstructure;

[x info] = ipopt(x0,funcs,options)

end

function val = objective(x)

val = (x(1) - 1)^2 + 2 * (x(2) - 1)^2;

end

function grad = gradient(x)

grad = [2*(x(1) -2);...
    4*(x(2)-1)];
end

function const = constraint(x)

const = [x(1) + 4*x(2);...
    -x(1) + x(2)];
end

function jac = jacobian(x)

jac = [1 4;...
    -1 1];

jac = sparse(jac);

end

function hess = hessian(x, sigma, lambda)

hess_f = sigma * [2 0; 0 4];

hess_c = lambda(1) * [1 0; 0 4] + lambda(2) * [ -1 0; 0 1];

hess = hess_f + hess_c;

hess = sparse(hess);
end

function hess_structure = hessianstructure()

hess_structure = sparse([2 0; 0 4]);

hess_structure = tril(hess_structure);

end


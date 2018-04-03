function N_i_p_array = N_i_p_Cal(knots, u, n)

% There are two special cases:
% When u == knots(1) (u_0) let the first N_0_P = 1 others to be zero
% When u == knots(end) (u_m) let the lst N_n_P = 1 others to be zero

% if (u ==knots(1))||(u == knots(end))
% %     if u ==knots(1)
% %         N_i_p_array = [1;zeros(n-1,1)];
% %     else
%         N_i_p_array = [zeros(n-1,1);1];
% %     end
% else
    
    % This function is used to compute the N_i_p vector
    m = length(knots);
    N_i_p_array = [];
    p = m - 1 - n;  % This is the order of the basis function
    N_i_p_name = strcat('@N_i_',num2str(p),'_fn');
    %     N_i_p_fun = str2func(N_i_p_name);
    N_i_p = N_i_p_name;
    
    for i = 1:n
        N_i_p_fun = str2func(N_i_p);
        N_i_p_val = feval(N_i_p_fun, knots, i, u);
        N_i_p_array = [N_i_p_array; N_i_p_val];
    end
% end

end
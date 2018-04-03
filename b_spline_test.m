function b_spline_test()
u = [ 0, 0.25, 0.5, 0.75, 1];
m = 5;
n = 1;
p = m - 1 - (n);  % This is the order of the basis function
N_i_p_name = strcat('@N_i_',num2str(p),'_fn');
N_i_p = N_i_p_name;
N_i_p_fun = str2func(N_i_p);

test_length = 1000;

u_test = linspace(0,1,test_length);

N_i_p_val_array = [];

for i = 1:test_length
    
    u_test_i = u_test(i);
    
    N_i_p_val = feval(N_i_p_fun, u, 1, u_test_i);
 
    N_i_p_val_array = [N_i_p_val_array; N_i_p_val];
    
end

plot(u_test, N_i_p_val_array)


% N_i_p_ref_array = [];

% for i = 1:test_length
%     u_test_i = u_test(i);
%     
%     if (u_test_i>=0)&&(u_test_i<0.25)
%         N_i_p_val = 8 * u_test_i * u_test_i;
%     end
%     
%     if (u_test_i>=0.25)&&(u_test_i<0.5)
%          N_i_p_val = -1.5 + 12 * u_test_i - 16 * u_test_i * u_test_i;       
%     end
%     
%     if (u_test_i>=0.5)&&(u_test_i<0.75)
%           N_i_p_val = 4.5 - 12 * u_test_i + 8 * u_test_i * u_test_i;              
%     end
%     
%     N_i_p_ref_array = [N_i_p_ref_array; N_i_p_val];
% end
% hold on
% plot(u_test, N_i_p_ref_array)
% 
% legend('Mine','Ref');

end


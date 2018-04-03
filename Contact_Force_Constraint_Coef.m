function [coef_A, coef_B, coef_C, coef_D, coef_M, coef_O] = Contact_Force_Constraint_Coef(mode_sequence_i)

ceof_tot = [ 0 0 0 0 0 0 ]';

mode_seq_str = num2str(mode_sequence_i);

for i = 1:length(mode_seq_str)
    mode_seq_str_i = mode_seq_str(i);
    if mode_seq_str_i =='1'
        ceof_tot(i) = 1;
    end
end

coef_A = ceof_tot(1);
coef_B = ceof_tot(2);
coef_C = ceof_tot(3);
coef_D = ceof_tot(4);
coef_M = ceof_tot(5);
coef_O = ceof_tot(6);

end
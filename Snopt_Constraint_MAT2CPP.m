function Snopt_Constraint_MAT2CPP(Snopt_Full_Constraint_Jac)

% The main problem is that since the structure of Snopt_Full_Constraint_Jac
% is so complicated so it is not efficient to convert all the code to C++
% all at once so it is better to do it row by row

[m,n] = size(Snopt_Full_Constraint_Jac);
iGfunjGvar = [];
Gsym = [];
equation_num = 0;
% for i = 1:m
for i = 743:m
        
    if i == 1
        Snopt_Full_Constraint_Jac_i = ccode(Snopt_Full_Constraint_Jac(i,:));
        [iGfunjGvar_i, Gsym_i ] = Sym2C_Index_Converter(Snopt_Full_Constraint_Jac_i);
        data = Snopt_Full_Constraint_Jac(i,:);
        equation_num_i = length(data) - (numel(data) - nnz(data));
        equation_num = equation_num +  equation_num_i;
    else
        Snopt_Full_Constraint_Jac_i_add = zeros(i-1,n);
        Snopt_Full_Constraint_Jac_i = Snopt_Full_Constraint_Jac(i,:);
        Snopt_Full_Constraint_Jac2B = ccode([Snopt_Full_Constraint_Jac_i_add;Snopt_Full_Constraint_Jac_i]);
        [iGfunjGvar_i, Gsym_i ] = Sym2C_Index_Converter(Snopt_Full_Constraint_Jac2B, equation_num);       
        data = Snopt_Full_Constraint_Jac(i,:);
        equation_num_i = length(data) - (numel(data) - nnz(data));
        equation_num = equation_num +  equation_num_i;
    end 
    iGfunjGvar = [iGfunjGvar iGfunjGvar_i];
    Gsym = [Gsym Gsym_i];
end

fid = fopen('iGfunjGvar.txt','wt');
fprintf(fid, '%s', iGfunjGvar);
fclose(fid);

Gsym = Gsym_Reduction(Gsym);

fid = fopen('Gsym.txt','wt');
fprintf(fid, '%s', Gsym);
fclose(fid);

save('743to1956.mat');

% This function is used to convert the matlab code to CPP script
[iGfunjGvar_start_Total, Gsym_start_Total ] = Sym2C_Index_Converter(ccode(Snopt_Full_Constraint_Jac));

sprintf(iGfunjGvar_start_Total);

sprintf(Gsym_start_Total);

end
function [iGfunjGvar_Total, Gsym_Total, row_no] = Sym2C_Index_Converter_Loop_ImpactM_Jac(Symbol_Exp, Current_Constraint_Number, Current_Jacobian_Number)

% This function is used to generate the proper index from the matlab
% symbolic expression to the C++ language

% Here the Symbol_Exp is a long char expression so the first step is to
% find how many row are there

Tot_Len = length(Symbol_Exp);
row_i = [];
row_no = 0;

row_index_array = [];
col_index_array = [];
sym_array = [];

for i = 1:Tot_Len
    
    if Symbol_Exp(i)~=';'
        row_i = [row_i Symbol_Exp(i)];
    else
        [ row_index_i, col_index_j, sym_ij] = single_row_reform(row_i);
        
        row_index_array = [row_index_array (row_index_i) '\n'];
        col_index_array = [col_index_array (col_index_j) '\n'];
        sym_array = [sym_array sym_ij '\n'];
        
        row_no = row_no + 1;
        row_i = [];
    end
end

if nargin == 1
    Current_Constraint_Number = [];
    Current_Jacobian_Number = [];
end

% The next job is to formulate this information into a snopt-friendly format
iGfunjGvar_Total = iGfunjGvar_fn(row_index_array, col_index_array, row_no, Current_Constraint_Number, Current_Jacobian_Number);

Gsym_Total = Gsym_neG_fn(sym_array, row_no, Current_Constraint_Number, Current_Jacobian_Number);
Fsym_Total = Gsym_Total;
Fsym_Total = strrep(Fsym_Total,'rIx_1_10','x[i * 48 * 10 + 9 * 48]');
Fsym_Total = strrep(Fsym_Total,'rIy_1_10','x[i * 48 * 10 + 9 * 48 + 1]');
Fsym_Total = strrep(Fsym_Total,'theta_1_10','x[i * 48 * 10 + 9 * 48 + 2]');
Fsym_Total = strrep(Fsym_Total,'q1_1_10','x[i * 48 * 10 + 9 * 48 + 3]');
Fsym_Total = strrep(Fsym_Total,'q2_1_10','x[i * 48 * 10 + 9 * 48 + 4]');
Fsym_Total = strrep(Fsym_Total,'q3_1_10','x[i * 48 * 10 + 9 * 48 + 5]');
Fsym_Total = strrep(Fsym_Total,'q4_1_10','x[i * 48 * 10 + 9 * 48 + 6]');
Fsym_Total = strrep(Fsym_Total,'q5_1_10','x[i * 48 * 10 + 9 * 48 + 7]');
Fsym_Total = strrep(Fsym_Total,'q6_1_10','x[i * 48 * 10 + 9 * 48 + 8]');
Fsym_Total = strrep(Fsym_Total,'q7_1_10','x[i * 48 * 10 + 9 * 48 + 9]');
Fsym_Total = strrep(Fsym_Total,'q8_1_10','x[i * 48 * 10 + 9 * 48 + 10]');
Fsym_Total = strrep(Fsym_Total,'q9_1_10','x[i * 48 * 10 + 9 * 48 + 11]');
Fsym_Total = strrep(Fsym_Total,'q10_1_10','x[i * 48 * 10 + 9 * 48 + 12]');

Fsym_Total = strrep(Fsym_Total,'rIxdot_1_10','x[i * 48 * 10 + 9 * 48 + 13]');
Fsym_Total = strrep(Fsym_Total,'rIydot_1_10','x[i * 48 * 10 + 9 * 48 + 14]');
Fsym_Total = strrep(Fsym_Total,'thetadot_1_10','x[i * 48 * 10 + 9 * 48 + 15]');
Fsym_Total = strrep(Fsym_Total,'q1dot_1_10','x[i * 48 * 10 + 9 * 48 + 16]');
Fsym_Total = strrep(Fsym_Total,'q2dot_1_10','x[i * 48 * 10 + 9 * 48 + 17]');
Fsym_Total = strrep(Fsym_Total,'q3dot_1_10','x[i * 48 * 10 + 9 * 48 + 18]');
Fsym_Total = strrep(Fsym_Total,'q4dot_1_10','x[i * 48 * 10 + 9 * 48 + 19]');
Fsym_Total = strrep(Fsym_Total,'q5dot_1_10','x[i * 48 * 10 + 9 * 48 + 20]');
Fsym_Total = strrep(Fsym_Total,'q6dot_1_10','x[i * 48 * 10 + 9 * 48 + 21]');
Fsym_Total = strrep(Fsym_Total,'q7dot_1_10','x[i * 48 * 10 + 9 * 48 + 22]');
Fsym_Total = strrep(Fsym_Total,'q8dot_1_10','x[i * 48 * 10 + 9 * 48 + 23]');
Fsym_Total = strrep(Fsym_Total,'q9dot_1_10','x[i * 48 * 10 + 9 * 48 + 24]');
Fsym_Total = strrep(Fsym_Total,'q10dot_1_10','x[i * 48 * 10 + 9 * 48 + 25]');

Fsym_Total = strrep(Fsym_Total,'rIx_2_1','x[(i+1) * 48 * 10 + 48]');
Fsym_Total = strrep(Fsym_Total,'rIy_2_1','x[(i+1) * 48 * 10 + 48 + 1]');
Fsym_Total = strrep(Fsym_Total,'theta_2_1','x[(i+1) * 48 * 10 + 48 + 2]');
Fsym_Total = strrep(Fsym_Total,'q1_2_1','x[(i+1) * 48 * 10 + 48 + 3]');
Fsym_Total = strrep(Fsym_Total,'q2_2_1','x[(i+1) * 48 * 10 + 48 + 4]');
Fsym_Total = strrep(Fsym_Total,'q3_2_1','x[(i+1) * 48 * 10 + 48 + 5]');
Fsym_Total = strrep(Fsym_Total,'q4_2_1','x[(i+1) * 48 * 10 + 48 + 6]');
Fsym_Total = strrep(Fsym_Total,'q5_2_1','x[(i+1) * 48 * 10 + 48 + 7]');
Fsym_Total = strrep(Fsym_Total,'q6_2_1','x[(i+1) * 48 * 10 + 48 + 8]');
Fsym_Total = strrep(Fsym_Total,'q7_2_1','x[(i+1) * 48 * 10 + 48 + 9]');
Fsym_Total = strrep(Fsym_Total,'q8_2_1','x[(i+1) * 48 * 10 + 48 + 10]');
Fsym_Total = strrep(Fsym_Total,'q9_2_1','x[(i+1) * 48 * 10 + 48 + 11]');
Fsym_Total = strrep(Fsym_Total,'q10_2_1','x[(i+1) * 48 * 10 + 48 + 12]');


Fsym_Total = strrep(Fsym_Total,'rIxdot_2_1','x[(i+1) * 48 * 10 + 48 + 13]');
Fsym_Total = strrep(Fsym_Total,'rIydot_2_1','x[(i+1) * 48 * 10 + 48 + 14]');
Fsym_Total = strrep(Fsym_Total,'thetadot_2_1','x[(i+1) * 48 * 10 + 48 + 15]');
Fsym_Total = strrep(Fsym_Total,'q1dot_2_1','x[(i+1) * 48 * 10 + 48 + 16]');
Fsym_Total = strrep(Fsym_Total,'q2dot_2_1','x[(i+1) * 48 * 10 + 48 + 17]');
Fsym_Total = strrep(Fsym_Total,'q3dot_2_1','x[(i+1) * 48 * 10 + 48 + 18]');
Fsym_Total = strrep(Fsym_Total,'q4dot_2_1','x[(i+1) * 48 * 10 + 48 + 19]');
Fsym_Total = strrep(Fsym_Total,'q5dot_2_1','x[(i+1) * 48 * 10 + 48 + 20]');
Fsym_Total = strrep(Fsym_Total,'q6dot_2_1','x[(i+1) * 48 * 10 + 48 + 21]');
Fsym_Total = strrep(Fsym_Total,'q7dot_2_1','x[(i+1) * 48 * 10 + 48 + 22]');
Fsym_Total = strrep(Fsym_Total,'q8dot_2_1','x[(i+1) * 48 * 10 + 48 + 23]');
Fsym_Total = strrep(Fsym_Total,'q9dot_2_1','x[(i+1) * 48 * 10 + 48 + 24]');
Fsym_Total = strrep(Fsym_Total,'q10dot_2_1','x[(i+1) * 48 * 10 + 48 + 25]');

Fsym_Total = strrep(Fsym_Total,'lambar_Ax_1','x[1920 + i * 12 ]');
Fsym_Total = strrep(Fsym_Total,'lambar_Ay_1','x[1920 + i * 12 + 1]');
Fsym_Total = strrep(Fsym_Total,'lambar_Bx_1','x[1920 + i * 12 + 2]');
Fsym_Total = strrep(Fsym_Total,'lambar_By_1','x[1920 + i * 12 + 3]');
Fsym_Total = strrep(Fsym_Total,'lambar_Cx_1','x[1920 + i * 12 + 4]');
Fsym_Total = strrep(Fsym_Total,'lambar_Cy_1','x[1920 + i * 12 + 5]');
Fsym_Total = strrep(Fsym_Total,'lambar_Dx_1','x[1920 + i * 12 + 6]');
Fsym_Total = strrep(Fsym_Total,'lambar_Dy_1','x[1920 + i * 12 + 7]');
Fsym_Total = strrep(Fsym_Total,'lambar_Mx_1','x[1920 + i * 12 + 8]');
Fsym_Total = strrep(Fsym_Total,'lambar_My_1','x[1920 + i * 12 + 9]');
Fsym_Total = strrep(Fsym_Total,'lambar_Ox_1','x[1920 + i * 12 + 10]');
Fsym_Total = strrep(Fsym_Total,'lambar_Oy_1','x[1920 + i * 12 + 11]');

Gsym_Total = Fsym_Total;
end

function Gsym_Total = Gsym_neG_fn(sym_array, row_no, Current_Index, Current_Jacobian_Number)

if isempty(Current_Index)==1
    neG_Start = ' *neG = 0;\n ';
%     G_neG = 'G[*neG] = ';
%     ij_end = ';\n ';
%     row_end = '*neG = *neG + 1;\n ';
else
    neG_Start1 = ' *neG = ';
    neG_Start2 = num2str(Current_Jacobian_Number);
    neG_Start3 = ';\n ';
    neG_Start = [neG_Start1 neG_Start2 neG_Start3]; 
end

% neG_Start = ' *neG = 0;\n ';
G_neG = 'G[*neG] = ';
ij_end = ';\n ';
row_end = '*neG = *neG + 1;\n ';

Gsym_Total = neG_Start;

Gsym_Total = [Gsym_Total 'for(int i = 0; i<segment_number-1; i++)\n { \n'];

for i = 1:row_no
    G_neG_i = char_select(sym_array, i);
    Gsym_Total = [Gsym_Total G_neG G_neG_i   ij_end ];
    Gsym_Total = [Gsym_Total row_end];
end

Gsym_Total = [Gsym_Total '} \n '];

end

function iGfunjGvar_Total = iGfunjGvar_fn(row_index_array, col_index_array, row_no, Current_Index, Current_Jacobian_Number)

% This function is used to formulate the constraint into a snopt-friendly
% format

if isempty(Current_Index)==1
    neG_Start = ' *neG = 0;\n ';
%     row_end = 'neG++;\n ';
else
    neG_Start1 = ' *neG = ';
    neG_Start2 = num2str(Current_Jacobian_Number);
    neG_Start3 = ';\n ';
    neG_Start = [neG_Start1 neG_Start2 neG_Start3]; 
%     row_end = '';
end

iGfun = 'iGfun[neG] = ';
jGvar = 'jGvar[neG] = ';
ij_end = ';\n ';
row_end = 'neG++;\n ';

iGfunjGvar_Total = neG_Start;

iGfunjGvar_Total = [iGfunjGvar_Total 'for(int i = 0; i<segment_number-1; i++)\n { \n'];

for i = 1:row_no
    iGfn_index = num2str(str2double(char_select(row_index_array, i)) + Current_Index);
    jGvar_index = char_select(col_index_array, i);
    iGfunjGvar_Total = [iGfunjGvar_Total iGfun iGfn_index ' + i * (grids_per_segment - 1) * 12 + 39 * j ' ij_end ];
    iGfunjGvar_Total = [iGfunjGvar_Total jGvar jGvar_index ' + i * (grids_per_segment - 1) * 48 + 12 * j ' ij_end ];
    iGfunjGvar_Total = [iGfunjGvar_Total row_end];
    
end
iGfunjGvar_Total = [iGfunjGvar_Total '} \n '];
end

function temp_array = char_select(char_i, index)

% This function is used to select the char_i between the (index -1)th \n
% and (index) \n
count = 0;
i = 1;
while(count~=index)
    temp_array = [];
    if index == 1
        while(i<=length(char_i))
            temp_i = char_i(i);
            if temp_i =='\'
                return
            else
                temp_array = [temp_array temp_i];
            end
            i = i + 1;
            
        end
    else
        while(i<length(char_i)&&(count<index))
            temp_i = char_i(i);
            if temp_i =='\'
                i = i + 2;
                count = count + 1;
                continue;
            end       
            if count == index - 1
                temp_array = [temp_array temp_i];   
            end
            i = i + 1;
        end
    end
end
end

function [ row_i, col_j, sym_ij] = single_row_reform(sym_i)
row_i = [];
col_j = [];
sym_ij = [];
for i = 1:length(sym_i)
    if sym_i(i)=='['   % This is the starting of the comparison
        i = i + 1;
        temp_i = sym_i(i);
        
        % This part is for the row index
        while(temp_i~=']')
            row_i = [row_i temp_i];
            i = i + 1;
            temp_i = sym_i(i);
        end
        
        % This part is for the column index
        i = i + 2;
        temp_i = sym_i(i);
        while(temp_i~=']')
            col_j = [col_j temp_i];
            i = i + 1;
            temp_i = sym_i(i);
        end
        i = i + 4;
        sym_ij = sym_i(i:end);
        break
    end
end
end
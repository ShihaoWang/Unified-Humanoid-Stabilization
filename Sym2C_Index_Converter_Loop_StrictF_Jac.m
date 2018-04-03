function [iGfunjGvar_Total, Gsym_Total, row_no] = Sym2C_Index_Converter_Loop_StrictF_Jac(Symbol_Exp, Current_Constraint_Number, Current_Jacobian_Number)

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

Gsym_Total = strrep(Gsym_Total,'rIx_1_1','x[i * 48 * 10 + j * 48]');
Gsym_Total = strrep(Gsym_Total,'rIy_1_1','x[i * 48 * 10 + j * 48 + 1]');
Gsym_Total = strrep(Gsym_Total,'theta_1_1','x[i * 48 * 10 + j * 48 + 2]');
Gsym_Total = strrep(Gsym_Total,'q1_1_1','x[i * 48 * 10 + j * 48 + 3]');
Gsym_Total = strrep(Gsym_Total,'q2_1_1','x[i * 48 * 10 + j * 48 + 4]');
Gsym_Total = strrep(Gsym_Total,'q3_1_1','x[i * 48 * 10 + j * 48 + 5]');
Gsym_Total = strrep(Gsym_Total,'q4_1_1','x[i * 48 * 10 + j * 48 + 6]');
Gsym_Total = strrep(Gsym_Total,'q5_1_1','x[i * 48 * 10 + j * 48 + 7]');
Gsym_Total = strrep(Gsym_Total,'q6_1_1','x[i * 48 * 10 + j * 48 + 8]');
Gsym_Total = strrep(Gsym_Total,'q7_1_1','x[i * 48 * 10 + j * 48 + 9]');
Gsym_Total = strrep(Gsym_Total,'q8_1_1','x[i * 48 * 10 + j * 48 + 10]');
Gsym_Total = strrep(Gsym_Total,'q9_1_1','x[i * 48 * 10 + j * 48 + 11]');
Gsym_Total = strrep(Gsym_Total,'q10_1_1','x[i * 48 * 10 + j * 48 + 12]');

Gsym_Total = strrep(Gsym_Total,'rIxdot_1_1','x[i * 48 * 10 + j * 48 + 13]');
Gsym_Total = strrep(Gsym_Total,'rIydot_1_1','x[i * 48 * 10 + j * 48 + 14]');
Gsym_Total = strrep(Gsym_Total,'thetadot_1_1','x[i * 48 * 10 + j * 48 + 15]');
Gsym_Total = strrep(Gsym_Total,'q1dot_1_1','x[i * 48 * 10 + j * 48 + 16]');
Gsym_Total = strrep(Gsym_Total,'q2dot_1_1','x[i * 48 * 10 + j * 48 + 17]');
Gsym_Total = strrep(Gsym_Total,'q3dot_1_1','x[i * 48 * 10 + j * 48 + 18]');
Gsym_Total = strrep(Gsym_Total,'q4dot_1_1','x[i * 48 * 10 + j * 48 + 19]');
Gsym_Total = strrep(Gsym_Total,'q5dot_1_1','x[i * 48 * 10 + j * 48 + 20]');
Gsym_Total = strrep(Gsym_Total,'q6dot_1_1','x[i * 48 * 10 + j * 48 + 21]');
Gsym_Total = strrep(Gsym_Total,'q7dot_1_1','x[i * 48 * 10 + j * 48 + 22]');
Gsym_Total = strrep(Gsym_Total,'q8dot_1_1','x[i * 48 * 10 + j * 48 + 23]');
Gsym_Total = strrep(Gsym_Total,'q9dot_1_1','x[i * 48 * 10 + j * 48 + 24]');
Gsym_Total = strrep(Gsym_Total,'q10dot_1_1','x[i * 48 * 10 + j * 48 + 25]');

Gsym_Total = strrep(Gsym_Total,'u1_1_1','x[i * 48 * 10 + j * 48 + 26]');
Gsym_Total = strrep(Gsym_Total,'u2_1_1','x[i * 48 * 10 + j * 48 + 27]');
Gsym_Total = strrep(Gsym_Total,'u3_1_1','x[i * 48 * 10 + j * 48 + 28]');
Gsym_Total = strrep(Gsym_Total,'u4_1_1','x[i * 48 * 10 + j * 48 + 29]');
Gsym_Total = strrep(Gsym_Total,'u5_1_1','x[i * 48 * 10 + j * 48 + 30]');
Gsym_Total = strrep(Gsym_Total,'u6_1_1','x[i * 48 * 10 + j * 48 + 31]');
Gsym_Total = strrep(Gsym_Total,'u7_1_1','x[i * 48 * 10 + j * 48 + 32]');
Gsym_Total = strrep(Gsym_Total,'u8_1_1','x[i * 48 * 10 + j * 48 + 33]');
Gsym_Total = strrep(Gsym_Total,'u9_1_1','x[i * 48 * 10 + j * 48 + 34]');
Gsym_Total = strrep(Gsym_Total,'u10_1_1','x[i * 48 * 10 + j * 48 + 35]');

Gsym_Total = strrep(Gsym_Total,'lamda_Ax_1_1','x[i * 48 * 10 + j * 48 + 36]');
Gsym_Total = strrep(Gsym_Total,'lamda_Ay_1_1','x[i * 48 * 10 + j * 48 + 37]');
Gsym_Total = strrep(Gsym_Total,'lamda_Bx_1_1','x[i * 48 * 10 + j * 48 + 38]');
Gsym_Total = strrep(Gsym_Total,'lamda_By_1_1','x[i * 48 * 10 + j * 48 + 39]');
Gsym_Total = strrep(Gsym_Total,'lamda_Cx_1_1','x[i * 48 * 10 + j * 48 + 40]');
Gsym_Total = strrep(Gsym_Total,'lamda_Cy_1_1','x[i * 48 * 10 + j * 48 + 41]');
Gsym_Total = strrep(Gsym_Total,'lamda_Dx_1_1','x[i * 48 * 10 + j * 48 + 42]');
Gsym_Total = strrep(Gsym_Total,'lamda_Dy_1_1','x[i * 48 * 10 + j * 48 + 43]');
Gsym_Total = strrep(Gsym_Total,'lamda_Mx_1_1','x[i * 48 * 10 + j * 48 + 44]');
Gsym_Total = strrep(Gsym_Total,'lamda_My_1_1','x[i * 48 * 10 + j * 48 + 45]');
Gsym_Total = strrep(Gsym_Total,'lamda_Ox_1_1','x[i * 48 * 10 + j * 48 + 46]');
Gsym_Total = strrep(Gsym_Total,'lamda_Oy_1_1','x[i * 48 * 10 + j * 48 + 47]');

Gsym_Total = strrep(Gsym_Total,'rIx_1_2','x[i * 48 * 10 + j * 48 + 48]');
Gsym_Total = strrep(Gsym_Total,'rIy_1_2','x[i * 48 * 10 + j * 48 + 49]');
Gsym_Total = strrep(Gsym_Total,'theta_1_2','x[i * 48 * 10 + j * 48 + 50]');
Gsym_Total = strrep(Gsym_Total,'q1_1_2','x[i * 48 * 10 + j * 48 + 51]');
Gsym_Total = strrep(Gsym_Total,'q2_1_2','x[i * 48 * 10 + j * 48 + 52]');
Gsym_Total = strrep(Gsym_Total,'q3_1_2','x[i * 48 * 10 + j * 48 + 53]');
Gsym_Total = strrep(Gsym_Total,'q4_1_2','x[i * 48 * 10 + j * 48 + 54]');
Gsym_Total = strrep(Gsym_Total,'q5_1_2','x[i * 48 * 10 + j * 48 + 55]');
Gsym_Total = strrep(Gsym_Total,'q6_1_2','x[i * 48 * 10 + j * 48 + 56]');
Gsym_Total = strrep(Gsym_Total,'q7_1_2','x[i * 48 * 10 + j * 48 + 57]');
Gsym_Total = strrep(Gsym_Total,'q8_1_2','x[i * 48 * 10 + j * 48 + 58]');
Gsym_Total = strrep(Gsym_Total,'q9_1_2','x[i * 48 * 10 + j * 48 + 59]');
Gsym_Total = strrep(Gsym_Total,'q10_1_2','x[i * 48 * 10 + j * 48 + 60]');

Gsym_Total = strrep(Gsym_Total,'rIxdot_1_2','x[i * 48 * 10 + j * 48 + 61]');
Gsym_Total = strrep(Gsym_Total,'rIydot_1_2','x[i * 48 * 10 + j * 48 + 62]');
Gsym_Total = strrep(Gsym_Total,'thetadot_1_2','x[i * 48 * 10 + j * 48 + 63]');
Gsym_Total = strrep(Gsym_Total,'q1dot_1_2','x[i * 48 * 10 + j * 48 + 64]');
Gsym_Total = strrep(Gsym_Total,'q2dot_1_2','x[i * 48 * 10 + j * 48 + 65]');
Gsym_Total = strrep(Gsym_Total,'q3dot_1_2','x[i * 48 * 10 + j * 48 + 66]');
Gsym_Total = strrep(Gsym_Total,'q4dot_1_2','x[i * 48 * 10 + j * 48 + 67]');
Gsym_Total = strrep(Gsym_Total,'q5dot_1_2','x[i * 48 * 10 + j * 48 + 68]');
Gsym_Total = strrep(Gsym_Total,'q6dot_1_2','x[i * 48 * 10 + j * 48 + 69]');
Gsym_Total = strrep(Gsym_Total,'q7dot_1_2','x[i * 48 * 10 + j * 48 + 70]');
Gsym_Total = strrep(Gsym_Total,'q8dot_1_2','x[i * 48 * 10 + j * 48 + 71]');
Gsym_Total = strrep(Gsym_Total,'q9dot_1_2','x[i * 48 * 10 + j * 48 + 72]');
Gsym_Total = strrep(Gsym_Total,'q10dot_1_2','x[i * 48 * 10 + j * 48 + 73]');
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

Gsym_Total = [Gsym_Total 'for(int i = 0; i<segment_number; i++)\n { for(int j = 0; j<grids_per_segment - 1; j++)\n {'];

for i = 1:row_no
    G_neG_i = char_select(sym_array, i);
    Gsym_Total = [Gsym_Total G_neG G_neG_i   ij_end ];
    Gsym_Total = [Gsym_Total row_end];
end

Gsym_Total = [Gsym_Total '} \n }\n'];

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

iGfunjGvar_Total = [iGfunjGvar_Total 'for(int i = 0; i<segment_number; i++)\n { for(int j = 0; j<grids_per_segment - 1; j++)\n {'];

for i = 1:row_no
    iGfn_index = num2str(str2double(char_select(row_index_array, i)) + Current_Index);
    jGvar_index = char_select(col_index_array, i);
    iGfunjGvar_Total = [iGfunjGvar_Total iGfun iGfn_index ' + i * (grids_per_segment - 1) * 12 + 6 * j ' ij_end ];
    iGfunjGvar_Total = [iGfunjGvar_Total jGvar jGvar_index ' + i * (grids_per_segment - 1) * 48 + 48 * j ' ij_end ];
    iGfunjGvar_Total = [iGfunjGvar_Total row_end];
    
end
iGfunjGvar_Total = [iGfunjGvar_Total '} \n } \n'];
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
            if temp_i =='\';
                return
            else
                temp_array = [temp_array temp_i];
            end
            i = i + 1;
            
        end
    else
        while(i<length(char_i)&&(count<index))
            temp_i = char_i(i);
            if temp_i =='\';
                i = i + 2;
                count = count + 1;
                continue;
            end       
            if count == index - 1;
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
function Fsym_Total = Sym2C_Index_Converter_Obj(Symbol_Exp, Current_Constraint_Number)

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
        %         [ row_index_i, col_index_j, sym_ij] = single_row_reform(row_i);
        %         row_index_array = [row_index_array (row_index_i) '\n'];
        %         col_index_array = [col_index_array (col_index_j) '\n'];
        %         sym_array = [sym_array sym_ij '\n'];
        %
        %         row_no = row_no + 1;
        %         row_i = [];
        break;
    end
end

% The next job is to formulate this information into a snopt-friendly format
% Fsym_Total = Gsym_neF_fn(sym_array, row_no, Current_Constraint_Number);
% 
Fsym_Total = row_i;

Fsym_Total = strrep(Fsym_Total,'rIx_4_10','x[3*48*10+9*48]');
Fsym_Total = strrep(Fsym_Total,'rIy_4_10','x[3*48*10+9*48+1]');
Fsym_Total = strrep(Fsym_Total,'theta_4_10','x[3*48*10+9*48+2]');
Fsym_Total = strrep(Fsym_Total,'q1_4_10','x[3*48*10+9*48+3]');
Fsym_Total = strrep(Fsym_Total,'q2_4_10','x[3*48*10+9*48+4]');
Fsym_Total = strrep(Fsym_Total,'q3_4_10','x[3*48*10+9*48+5]');
Fsym_Total = strrep(Fsym_Total,'q4_4_10','x[3*48*10+9*48+6]');
Fsym_Total = strrep(Fsym_Total,'q5_4_10','x[3*48*10+9*48+7]');
Fsym_Total = strrep(Fsym_Total,'q6_4_10','x[3*48*10+9*48+8]');
Fsym_Total = strrep(Fsym_Total,'q7_4_10','x[3*48*10+9*48+9]');
Fsym_Total = strrep(Fsym_Total,'q8_4_10','x[3*48*10+9*48+10]');
Fsym_Total = strrep(Fsym_Total,'q9_4_10','x[3*48*10+9*48+11]');
Fsym_Total = strrep(Fsym_Total,'q10_4_10','x[3*48*10+9*48+12]');

Fsym_Total = strrep(Fsym_Total,'rIxdot_4_10','x[3*48*10+9*48+13]');
Fsym_Total = strrep(Fsym_Total,'rIydot_4_10','x[3*48*10+9*48+14]');
Fsym_Total = strrep(Fsym_Total,'thetadot_4_10','x[3*48*10+9*48+15]');
Fsym_Total = strrep(Fsym_Total,'q1dot_4_10','x[3*48*10+9*48+16]');
Fsym_Total = strrep(Fsym_Total,'q2dot_4_10','x[3*48*10+9*48+17]');
Fsym_Total = strrep(Fsym_Total,'q3dot_4_10','x[3*48*10+9*48+18]');
Fsym_Total = strrep(Fsym_Total,'q4dot_4_10','x[3*48*10+9*48+19]');
Fsym_Total = strrep(Fsym_Total,'q5dot_4_10','x[3*48*10+9*48+20]');
Fsym_Total = strrep(Fsym_Total,'q6dot_4_10','x[3*48*10+9*48+21]');
Fsym_Total = strrep(Fsym_Total,'q7dot_4_10','x[3*48*10+9*48+22]');
Fsym_Total = strrep(Fsym_Total,'q8dot_4_10','x[3*48*10+9*48+23]');
Fsym_Total = strrep(Fsym_Total,'q9dot_4_10','x[3*48*10+9*48+24]');
Fsym_Total = strrep(Fsym_Total,'q10dot_4_10','x[3*48*10+9*48+25]');

end

function Gsym_Total = Gsym_neF_fn(sym_array, row_no, Current_Constraint_Number)


neF_Start1 = ' *neF = ';
neF_Start2 = num2str(Current_Constraint_Number);
neF_Start3 = ';\n ';
neF_Start = [neF_Start1 neF_Start2 neF_Start3];


% neF_Start = ' *neF = 0;\n ';
G_neF = 'F[*neF] = ';
ij_end = ';\n ';
row_end = '*neF = *neF + 1;\n ';

Gsym_Total = neF_Start;

% Gsym_Total = [Gsym_Total 'for(int i = 0; i<segment_number-1; i++)\n { \n'];
for i = 1:row_no
    G_neF_i = char_select(sym_array, i);
    Gsym_Total = [Gsym_Total G_neF G_neF_i   ij_end ];
    Gsym_Total = [Gsym_Total row_end];
end
% Gsym_Total = [Gsym_Total '} \n '];

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
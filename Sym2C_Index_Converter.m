function [iGfunjGvar_Total, Gsym_Total ] = Sym2C_Index_Converter(Symbol_Exp, Current_Constraint_Number, Current_Jacobian_Number)

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

for i = 1:row_no
    G_neG_i = char_select(sym_array, i);
    Gsym_Total = [Gsym_Total G_neG G_neG_i   ij_end ];
    Gsym_Total = [Gsym_Total row_end];
    
end

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

for i = 1:row_no
    iGfn_index = num2str(str2double(char_select(row_index_array, i)) + Current_Index);
    jGvar_index = char_select(col_index_array, i);
    iGfunjGvar_Total = [iGfunjGvar_Total iGfun iGfn_index  ij_end ];
    iGfunjGvar_Total = [iGfunjGvar_Total jGvar jGvar_index ij_end ];
    iGfunjGvar_Total = [iGfunjGvar_Total row_end];
    
end

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
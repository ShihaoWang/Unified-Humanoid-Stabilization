function String_Exp = Mat2Mathe(Sym_Exp)

% This function is used to geneate the necessary form of the symbolic
% expression used for Mathematica
%
String_Exp_Raw = char(Sym_Exp);
String_Exp = '';   % Initialize to be a null string

Keyword_Flag = 0;
Keyword_Len = 0;
Left_Brack_Count = 0;
Right_Brack_Count = 0;
for i = 1:length(String_Exp_Raw)
    Sym_Exp_i = String_Exp_Raw(i);
    % This is used to rewrite the analytic expression from sin() to sin[]
    if(Sym_Exp_i == 's')||(Sym_Exp_i == 'c')||(Keyword_Flag == 1)
        if Keyword_Flag == 0  % This means that this is the first time that the keyword has been identified
            Keyword_Flag = 1;
            Keyword_Len = 1;
        else
            % In this case, the first keyword has been identified
            if Keyword_Len == 1  % Then we are expecting a second word
                Keyword_Len = Keyword_Len + 1;  % Keyword_Len = 2
            end
            if Keyword_Len == 2
                Keyword_Len = Keyword_Len + 1;  % Keyword_Len = 3
            end
            
        end
    end
    if Keyword_Len == 3
        if Sym_Exp_i == '(';
            Left_Brack_Count = Left_Brack_Count + 1;
            Sym_Exp_i = '[';
            
        end
        if Sym_Exp_i == ')';
            Right_Brack_Count = Right_Brack_Count + 1;
            Sym_Exp_i = ']';
        end
        
        if (Left_Brack_Count == 1)&&(Right_Brack_Count == 1)
            Keyword_Flag = 0;
            Keyword_Len = 0;
            Left_Brack_Count = 0;
            Right_Brack_Count = 0;
        end
        
    end
    String_Exp = strcat(String_Exp,Sym_Exp_i);
end
end


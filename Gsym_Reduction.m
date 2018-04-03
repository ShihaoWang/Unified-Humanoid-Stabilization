function Gsym_tot = Gsym_Reduction(Gsym_char)
% This function is used to reduce the necessary row of G[*neG]
Gsym_tot = [];
i = 1;
Gsym_char_len = length(Gsym_char);
while (i<=Gsym_char_len)
    Gsym_i = Gsym_char(i);
    if(Gsym_i =='*')
        if (i + 6<Gsym_char_len)&&(sum(Gsym_char(i+1:i+6) =='neG = ')==6)
            if Gsym_char(i+7)~='*'
                j = 1;
                flag = 0;
                while (flag ==0)                 
                    Gsym_j = Gsym_char(i+j);
                    if Gsym_j == ';'
                        flag = 1;
                    else
                        j = j + 1;        
                    end                 
                end
                i = i + j;
                Gsym_i = [];
            end   
        end       
    end
    Gsym_tot = [Gsym_tot Gsym_i];
    i = i + 1;
end

end
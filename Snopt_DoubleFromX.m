function Extrac_tot = Snopt_DoubleFromX( Snopt_State )
% Then it is the official formulation of the script for Snopt

% The first is to extract the variable names into a x
%%
Extrac_tot = [];
Extrac_Start = 'double ';

for i = 1:length(Snopt_State)
    
    Extrac_tot = [Extrac_tot Extrac_Start char(Snopt_State(i)) ' = x[' num2str(i-1) ']; \n'];
 
end


end


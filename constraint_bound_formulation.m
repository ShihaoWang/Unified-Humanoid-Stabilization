function [Snopt_Constraint_Bound, Snopt_Constraint_Raw]= constraint_bound_formulation(Snopt_Constraint_Bound, Snopt_Constraint_Raw, Constraint_Eqn_Length, Constraint_Type)

Eq_Flow_start = 'Flow[';
Eq_Flow_end = '] = 0.0;';
Eq_Fupp_start = '   Fupp[';
Eq_Fupp_end = '] = 0.0; \n';

Ineq_Flow_start = 'Flow[';
Ineq_Flow_end = '] = 0.0';
Ineq_Fupp_start = ';   Fupp[';
Ineq_Fupp_end = '] = infBnd; \n';

current_index = length(Snopt_Constraint_Raw);

current_indexpConstraint_Eqn_Length = current_index + Constraint_Eqn_Length;

for_loop_start = ['for( int i = ' num2str(current_index) '; i<' num2str(current_indexpConstraint_Eqn_Length) '; i++)\n {  '];
for_loop_end = ' } \n ';

if length(Constraint_Type) == 1
    % This is the indicator of the same kinds of constraints
    if Constraint_Type == 1         % This means that all the constraints are inequality constraints     
        Snopt_Constraint_Bound = [Snopt_Constraint_Bound for_loop_start];
        Snopt_Constraint_Bound = [Snopt_Constraint_Bound Ineq_Flow_start 'i' Ineq_Flow_end];
        Snopt_Constraint_Bound = [Snopt_Constraint_Bound Ineq_Fupp_start 'i' Ineq_Fupp_end];
        Snopt_Constraint_Bound = [Snopt_Constraint_Bound for_loop_end];
    else
        if Constraint_Type == 0     % This means that all the constraints are equality constraints
            Snopt_Constraint_Bound = [Snopt_Constraint_Bound for_loop_start];
            Snopt_Constraint_Bound = [Snopt_Constraint_Bound Eq_Flow_start 'i' Eq_Flow_end];
            Snopt_Constraint_Bound = [Snopt_Constraint_Bound Eq_Fupp_start 'i' Eq_Fupp_end];
            Snopt_Constraint_Bound = [Snopt_Constraint_Bound for_loop_end];
        end
    end
else
    % This means that the current has different types: inequality and equality
    
    for_loop_start = ['for( int i = ' num2str(current_index) '; i<' num2str(current_indexpConstraint_Eqn_Length) '; i++)\n {  '];
    for_loop_end = ' } \n ';
    
    Snopt_Constraint_Bound = [Snopt_Constraint_Bound for_loop_start];
    
    Snopt_Constraint_Bound = [Snopt_Constraint_Bound Ineq_Flow_start 'i' Ineq_Flow_end];
    Snopt_Constraint_Bound = [Snopt_Constraint_Bound Ineq_Fupp_start 'i' Ineq_Fupp_end];
    for i = 1:Constraint_Eqn_Length
        if Constraint_Type(i) ==0
            Snopt_Constraint_Bound = [Snopt_Constraint_Bound Ineq_Flow_start 'i' Ineq_Flow_end];
            Snopt_Constraint_Bound = [Snopt_Constraint_Bound Ineq_Fupp_start 'i' Ineq_Fupp_end];
        else
            Snopt_Constraint_Bound = [Snopt_Constraint_Bound Ineq_Flow_start num2str(current_index + i - 1) Ineq_Flow_end];
            Snopt_Constraint_Bound = [Snopt_Constraint_Bound Ineq_Fupp_start num2str(current_index + i - 1) Ineq_Fupp_end];
        end
    end
    Snopt_Constraint_Bound = [Snopt_Constraint_Bound for_loop_end];   
end

Snopt_Constraint_Raw = [Snopt_Constraint_Raw; ones(Constraint_Eqn_Length,1)];

end
function Snopt_State_Bound = Snopt_State_Bound_Sub(Snopt_State_Bound, current_index, variable_low, variable_upp)

xlow_start = 'xlow[';
xlow_end = '] = ';
xupp_start = ';   xupp[';
xupp_middle = '] = ';
xupp_end = '; \n';

Snopt_State_Bound = [Snopt_State_Bound xlow_start num2str(current_index) xlow_end variable_low xupp_start num2str(current_index) xupp_middle variable_upp xupp_end];
end
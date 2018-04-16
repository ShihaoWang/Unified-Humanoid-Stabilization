function Optimized_State_Plot(StateNdot_tot, P)

StateNdot_tot = reshape(StateNdot_tot,26,30);
[~,n] = size(StateNdot_tot);
axes_plot = axes;

for i = 1:n
    
    q_array_i = StateNdot_tot(:,i)';
    Single_Frame_Plot(q_array_i, P,axes_plot);

end
end


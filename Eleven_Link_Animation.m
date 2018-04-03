function Eleven_Link_Animation(q_tot,handles)

% This function is used to plot the animation of this eleven-link robot
% given a set of configuration

[m,n] = size(q_tot); 
for i = 1:m
    q_tot_i = q_tot(i,:);
    Animation_Fn(q_tot_i,handles);
    pause(0.01);   
end
end


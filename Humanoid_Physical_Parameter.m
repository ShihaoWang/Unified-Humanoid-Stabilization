function p = Humanoid_Physical_Parameter()

% This function is used to give the values for the physical parameter
% Assume that the mass is propotional to the link length

% The item sequence is [foot, shank, thigh, body ,head, arm, forearm]
l_foot = 0.2;         l_shank = 0.3;        l_thigh = 0.3;
l_body = 0.35;        l_head = 0.15;        l_arm = 0.2;          
l_forearm = 0.25;
p.l = [l_foot; l_shank; l_thigh; l_body; l_head; l_forearm; l_arm];

l2m = 10;  % 1m is 10 kg
p.m = p.l *l2m;
p.I = 1/3 * p.m .* p.l.*p.l;

end


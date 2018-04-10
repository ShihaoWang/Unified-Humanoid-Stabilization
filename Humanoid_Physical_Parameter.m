function p = Humanoid_Physical_Parameter()

% This function is used to give the values for the physical parameter
% Assume that the mass is propotional to the link length

% The item sequence is [foot, shank, thigh, body ,head, arm, forearm]

l_shank = 0.325;        l_thigh = 0.325;            l_body = 0.55;
l_arm = 0.25;           l_forearm = 0.45;           
l_foot = 0.225;         l_heel = 0.1;

p.l = [l_shank; l_thigh; l_body; l_forearm; l_arm];

l2m = 15;  % 1m is 10 kg
p.m = p.l *l2m;
p.I = 1/3 * p.m .* p.l.*p.l;

p.KJ2JI = 0.25;
p.m_tot = sum(p.m);
end


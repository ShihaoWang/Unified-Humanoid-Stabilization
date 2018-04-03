function P = Environmental_Features_Initialization(P)
% This function is used to generate the environmental features
% The default environmental feature is the ground

ground_left = [-100,0];
ground_right = [100,0];

vertical_wall_bottom = [1.5,0];
vertical_wall_up = [1.5,10];

P.Obs_Info = [ground_left,            ground_right;...
              vertical_wall_bottom,   vertical_wall_up];

end


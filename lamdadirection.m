function lamda_vec = lamdadirection(theta)

% This function is used to generate the unit direction vector under a given
% angle

% This angle is computed with respect to the horizontal line and the
% counterclockwise direction is the positive direction

lamda_vec = [cos(theta), sin(theta)]';

end


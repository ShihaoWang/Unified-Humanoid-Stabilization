function Dist = Relative_Dist_Cal(rX, Obs_Info)

% This function is used to compute the relative distance given one point
% and an environmental features

% Currently assume that the obstacles are of vertical shape

[m,~] = size(rX);
if m ~=1   
    rX = rX';
end

[m,~] = size(Obs_Info);

Dist_Array = [];

for i = 1:m
    
    Dist_i = point_to_line( [rX,0], [Obs_Info(i,1:2),0],[Obs_Info(i,3:4),0]);
    
    Dist_Array = [Dist_Array; Dist_i];
    
end

Dist = min(Dist_Array);

end

function d = point_to_line(pt, v1, v2)

a = v1 - v2;

b = pt - v2;

d = norm(cross(a,b)) / norm(a);
end

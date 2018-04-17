function Obs_Dist = Obs_Dist_Fn(Point_i, Envi_Map, Direction)
% This function gives the signed distance from a point to the surface of
% environmental obstacle
if nargin<3
    [m,~] = size(Envi_Map);
    Obs_Dist_Array = zeros(m,1);    
    for i = 1:m
        Envi_Map_i = Envi_Map(i,:);
        Dist_i = Dist_Cal_Fn(Point_i, Envi_Map_i);
        Obs_Dist_Array(i,1) = Dist_i;
    end
    Obs_Dist = min(Obs_Dist_Array);
else
    [m,~] = size(Envi_Map);
    Obs_Dist_Array = zeros(m,1);
    for i = 1:m
        Envi_Map_i = Envi_Map(i,:);
        Dist_i = Dist_Cal_Fn(Point_i, Envi_Map_i, Direction);
        Obs_Dist_Array(i,1) = Dist_i;
    end
    Obs_Dist = min(Obs_Dist_Array);
end
end
function Dist_i = Dist_Cal_Fn(Point_i, Envi_Map_i, Direction)
% This function will compute the environmental obstacle geometry to have
% the linear expression 
Envi_Map_Point_A_x = Envi_Map_i(1);
Envi_Map_Point_A_y = Envi_Map_i(2);
Envi_Map_Point_B_x = Envi_Map_i(3);
Envi_Map_Point_B_y = Envi_Map_i(4);
if nargin<3 
    if Envi_Map_Point_A_x == Envi_Map_Point_B_x        
        Dist_i = Envi_Map_Point_A_x - Point_i(1);       
        return
    end
    if Envi_Map_Point_A_y == Envi_Map_Point_B_y
        Dist_i = Point_i(2) - Envi_Map_Point_A_y;        
        return
    end
    t = [Envi_Map_Point_A_x Envi_Map_Point_B_x];
    y = [Envi_Map_Point_A_y Envi_Map_Point_B_y];
    [~,m,b] = regression(t,y,'one');  % y = m*x + b
    Dist_i_Sign = sign(Point_i(2) - (m * Point_i(1) + b));
    v1 = [Envi_Map_Point_B_x Envi_Map_Point_B_y 0]';
    v2 = [Envi_Map_Point_A_x Envi_Map_Point_A_y 0]';
    pt = [Point_i(1) Point_i(2) 0]';
    a = v1 - v2;
    b = pt - v2;
    d = norm(cross(a,b)) / norm(a);
    Dist_i_Val = d;
    Dist_i = Dist_i_Sign * Dist_i_Val;
else
    if Direction =='x'
        if Envi_Map_Point_A_y == Envi_Map_Point_B_y
            Dist_i = Envi_Map_Point_A_x - Point_i(2);
            return
        end
    else
        if Envi_Map_Point_A_x == Envi_Map_Point_B_x
            
            Dist_i = Point_i(2) - Envi_Map_Point_A_y;
            return
        end       
    end
    Dist_i = Inf; 
end
end
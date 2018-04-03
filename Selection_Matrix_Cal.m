function [M_Pos, M_Vel] = Selection_Matrix_Cal(value_rX,value_rY)

M_Pos = zeros(2,2);

M_Vel = zeros(2,2);

if (value_rX == 1)&&(value_rY == 1)
    
    M_Pos(2,2) = 1;
    
    M_Vel(1,1) = 1;     M_Vel(2,2) = 1;
    
    return
    
end

if (value_rX == 1)&&(value_rY == 2)
    
    M_Pos(1,1) = 1;
    
    M_Vel(1,1) = 1;
    
    return
    
end

if (value_rX == 2)&&(value_rY == 1)
    
    M_Pos(2,2) = 1;
    
    M_Vel(2,2) = 1;
    
    return
    
end
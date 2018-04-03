function flag = In_Contact_Or_Not(Pos,P)

% This function is used to judge whether the current position is in contact
% with the nearby environmental features or not
Obs_Info = P.Obs_Info;
[m,~] = size(Obs_Info);
Dist_Info = [];
for i = 1:m
    A_i = [Obs_Info(i,1), Obs_Info(i,2), 0];
    B_i = [Obs_Info(i,3), Obs_Info(i,4), 0];
    C = [Pos(1), Pos(2),0];
    V_i=cross(A_i-C,B_i-C);
    Pos_i=V_i(3)/norm(A_i - B_i);
    Dist_Info = [Dist_Info; Pos_i];
end
eps = 0.001;
if min(Dist_Info)<=eps  
    flag = 1;
else
    flag = 0;
    
end
end


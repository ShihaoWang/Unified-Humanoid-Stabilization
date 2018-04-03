function Kinematic_Check()

if nargin == 0
    % This function is used to check the kinematic structure of the robot
    theta = 90;              % Checked    
    q1 = 90;                     % Checked
    q2 = 120;                     % Checked
    q3 = 235;                     % Checked
    q4 = 150;                     % Checked
    q5 = 150;                     % Checked
    q6 = 90;                      % Checked
    q7 = 60;                      % Checked
    q8 = 45;                      % Checked
    q9 = 60;                      % Checked
    q10 = 30;                     % Checked
    
    q_tot = [0;0; theta;q1;q2;q3;q4;q5;q6;q7;q8;q9;q10] * pi/180;    
end

Animation_Fn(q_tot);
end


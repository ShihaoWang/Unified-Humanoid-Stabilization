function [position_x_i, isterminal_x_i, direction_x_i] = Event_Condition_Generation(contact_status, point_x_index, Contact_Force_AtoO, P)

contact_status_i = contact_status_interprete(contact_status(point_x_index,:));

% This function is used to generate the three output values for a specific
% event condition.
rA = P.rA;
rB = P.rB;
rC = P.rC;
rD = P.rD;
rM = P.rM;
rO = P.rO;

vA = P.vA;
vB = P.vB;
vC = P.vC;
vD = P.vD;
vM = P.vM;
vO = P.vO;

contact_position_name = cellstr(P.contact_position_name);
r_i = eval(contact_position_name{point_x_index});

contact_velocity_name = cellstr(P.contact_velocity_name);
v_i = eval(contact_velocity_name{point_x_index});

if contact_status_i == 1
    
    % The sticking event is deactived
    
    position_x_1 = r_i(2);
    
    isterminal_x_1 = 0;
    
    direction_x_1 = 1;
    
    % The sliding in the x direction is possible.
    
    position_x_2 = (P.mu * Contact_Force_AtoO(point_x_index,2))^2 -(Contact_Force_AtoO(point_x_index,1))^2;
    
    isterminal_x_2 = 0;
    
    direction_x_2 = -1;
    
    % The sliding in the y direction is also possible.
    
    position_x_3 = (P.mu * Contact_Force_AtoO(point_x_index,1))^2 -(Contact_Force_AtoO(point_x_index,2))^2;
    
    isterminal_x_3 = 0;
    
    direction_x_3 = -1;
    
    
    % The separation condition
    if point_x_index<=4
        
        position_x_4 = Contact_Force_AtoO(point_x_index,2);
        
        isterminal_x_4 = 1;
        
        direction_x_4 = 0;
    else
        position_x_4 = Contact_Force_AtoO(point_x_index,1);
        
        isterminal_x_4 = 1;
        
        direction_x_4 = 0; 
    end   
end

% In this case, x sliding y sticking
if contact_status_i == 2
    
    % The sticking event is deactived
    
    position_x_1 = v_i(2);
    
    isterminal_x_1 = 0;
    
    direction_x_1 = 1;
    
    % The sliding in the x direction, measure the current velocity
    
    position_x_2 = v_i(1);
    
    isterminal_x_2 = 0;
    
    direction_x_2 = 0;
    
    % The sliding in the y direction is also possible.
    
    position_x_3 = v_i(2);
    
    isterminal_x_3 = 0;
    
    direction_x_3 = 0;
    
    % The separation condition
    position_x_4 = Contact_Force_AtoO(point_x_index,2);
    
    isterminal_x_4 = 1;
    
    direction_x_4 = 0;
    
end

% In this case, y sliding x sticking
if contact_status_i == 3
    
    % The sticking event is deactived
    
    position_x_1 = r_i(1);
    
    isterminal_x_1 = 0;
    
    direction_x_1 = 1;
    
    % The sliding in the y direction, measure the current velocity
    
    position_x_2 = v_i(2);
    
    isterminal_x_2 = 0;
    
    direction_x_2 = 0;
    
    % The sliding in the x direction is also possible.
    
    position_x_3 = v_i(1);
    
    isterminal_x_3 = 0;
    
    direction_x_3 = 0;
    
    % The separation condition
    position_x_4 = Contact_Force_AtoO(point_x_index,1);
    
    isterminal_x_4 = 1;
    
    direction_x_4 = 0;
    
end
% In this case, x, y are in all separation condition

if contact_status_i == 4
    
    % The sticking event is deactived
    
    position_x_1 = r_i(2);
    
    isterminal_x_1 = 1;
    
    direction_x_1 = -1;
    
    % The sliding in the x direction, measure the current velocity
    
    position_x_2 = v_i(2);
    
    isterminal_x_2 = 0;
    
    direction_x_2 = 0;
    
    % The sliding in the y direction is also possible.
    
    position_x_3 = v_i(1);
    
    isterminal_x_3 = 0;
    
    direction_x_3 = -1;
    
    % The separation condition
    position_x_4 = Contact_Force_AtoO(point_x_index,1);
    
    isterminal_x_4 = 0;
    
    direction_x_4 = 0;
    
end

position_x_i    = [  position_x_1,     position_x_2,    position_x_3,     position_x_4]';
isterminal_x_i  = [isterminal_x_1,   isterminal_x_2,  isterminal_x_3,   isterminal_x_4]';
direction_x_i   = [ direction_x_1,    direction_x_2,   direction_x_3,    direction_x_4]';

end
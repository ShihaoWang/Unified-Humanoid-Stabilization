function contact_status_index = contact_status_interprete(contact_status_i)


% This case, the contact remains a sticking condition with respect to the
% contact surface
if (max(contact_status_i) == min(contact_status_i))&&( min(contact_status_i)== 1)
    contact_status_index =1;
    return
end

if (max(contact_status_i) == 1)&&(min(contact_status_i) == 0.5)
    [~,contact_sticking_index] = max(contact_status_i);  % This is used to find out which direction remains a holonomic constraint
    if  contact_sticking_index == 1  % In this case, the x position remains fixed so the end effector is sliding along the y axis
        contact_status_index = 3;         % x sticking, y sliding
    else
        contact_status_index = 2;         % y sticking, x sliding      
    end
end

if (max(contact_status_i) == min(contact_status_i))&&( min(contact_status_i)== 0)
    contact_status_index = 4;
    return;
end
end
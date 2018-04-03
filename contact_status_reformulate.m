function contact_status = contact_status_reformulate(contact_status_raw)

% This contact_status vector is a 6 by 2 vector but the raw contact status
% vector from the GUI has only the information of the feet contact status
% so it is a 4 by 2 vector

contact_status = zeros(6,2);

for i = 1:4    
    contact_status(i,1) = contact_status_raw2sim(contact_status_raw(i,1));
    contact_status(i,2) = contact_status_raw2sim(contact_status_raw(i,2));
end

end
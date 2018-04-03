function contact_value = contact_status_raw2sim(contact_status_i)

if contact_status_i == 1
    contact_value = 1;  
else
    if contact_status_i == 2     
        contact_value = 0.5;       
    else
        contact_value = 0;       
    end
end
end
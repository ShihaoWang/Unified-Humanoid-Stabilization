function Contact_Force_AtoO_Reshaped = Contact_Force_Reshape(Contact_Force_AtoO)


Contact_Force_AtoO_Reshaped = zeros(6,2);

for i = 1:12
    if mod(i,2) == 1
        Contact_Force_AtoO_Reshaped((i + 1)/2, 1) = Contact_Force_AtoO(i);
    else
        Contact_Force_AtoO_Reshaped(i/2, 2) = Contact_Force_AtoO(i);         
    end
end
end
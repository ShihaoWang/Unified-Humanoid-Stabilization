function contact_force_AtoO = Contact_Force_Relocate(contact_status, lamda_t, M_t, contact_independ, contact_depend, Full_rank_Jac_Row_Index, Raw_rank_Jac_Row_Index)
% This function is used to output all the contact force at each contact
% point
contact_force_AtoO = zeros(12,1);

% The first job is to make sure that lamda_t is rescheduled in a way that
% the over-constrainted force can be calculated.

% Currently the robot feet are not planned to make vertical contact with
% the nearby environment

% lamda_t_raw = zeros(nnz())

if isempty(M_t) == 1  % This means that there is no sliding condition
    if isempty(lamda_t) == 1  % This means that all contacts are in separation conditions
        return
    else
        % This means that some forces are not separated.
        %                 contact_independ_number = nnz(~contact_independ);
        
        if isempty(Full_rank_Jac_Row_Index) == 1
            index_array = find(contact_independ);
            for i = 1:length(lamda_t)
                contact_force_AtoO(index_array(i)) = lamda_t(i);
            end
        else
            for i = 1:length(Full_rank_Jac_Row_Index)
                contact_force_AtoO(Full_rank_Jac_Row_Index(i)) = lamda_t(i);
            end
        end
    end
else % This means that there exists at least one sliding friction force
    lamda_d = M_t * lamda_t;
    lamda_d_index = find(contact_depend);
    for i = 1:length(Full_rank_Jac_Row_Index)
        contact_force_AtoO(Full_rank_Jac_Row_Index(i)) = lamda_t(i);
    end
    
    for i = 1:length(lamda_d_index)
        contact_force_AtoO(lamda_d_index(i)) = lamda_d(i);
    end
    
end

% Here is a final calibration of the force
contact_status_A = contact_status_interprete(contact_status(1,:));
contact_status_B = contact_status_interprete(contact_status(2,:));
contact_status_C = contact_status_interprete(contact_status(3,:));
contact_status_D = contact_status_interprete(contact_status(4,:));

if (contact_status_A == contact_status_B)&&(contact_status_A == 1)  % This means that currently there is an overconstraint equation
    contact_AB_horizontal = (contact_force_AtoO(1) + contact_force_AtoO(3))/2;
    contact_force_AtoO(1) = contact_AB_horizontal;
    contact_force_AtoO(3) = contact_AB_horizontal;
end

if (contact_status_C == contact_status_D)&&(contact_status_C == 1)  % This means that currently there is an overconstraint equation
    contact_CD_horizontal = (contact_force_AtoO(5) + contact_force_AtoO(7))/2;
    contact_force_AtoO(5) = contact_CD_horizontal;
    contact_force_AtoO(7) = contact_CD_horizontal;
end

end
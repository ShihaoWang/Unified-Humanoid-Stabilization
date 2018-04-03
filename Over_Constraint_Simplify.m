function [Full_rank_Jac_t, Full_rank_Jacdot_qdot_t,Full_rank_Jac_Row_Index, Raw_rank_Jac_Row_Index, Jac_independ_index_t, Jac_independ_index_in_Jac_t]= Over_Constraint_Simplify(Jac_t, Jacdot_qdot_t,Jac_independ_index_t, Jac_independ_index_in_Jac_t)
% This function is used to take care of the over-constrained equations
Full_rank_Jac_Row_Index = [];
Raw_rank_Jac_Row_Index = []; 
if nargin == 2
    [m,n] = size(Jac_t);
    if m == rank(Jac_t)
        Full_rank_Jac_t    = Jac_t;
        Full_rank_Jacdot_qdot_t = Jacdot_qdot_t;
    else
        A = Jac_t';
        [~,colind] = rref(A);
        B = A(:, colind);
        Full_rank_Jac_t = B';
        Full_rank_Jacdot_qdot_t = Jacdot_qdot_t(colind',:);
        Full_rank_Jac_Row_Index = colind;
        Raw_rank_Jac_Row_Index = 1:m;
    end
else
    [m,n] = size(Jac_t);
    if m == rank(Jac_t)
        Full_rank_Jac_t    = Jac_t;
        Full_rank_Jacdot_qdot_t = Jacdot_qdot_t;
    else
        A = Jac_t';
        [~,colind] = rref(A);
        B = A(:, colind);
        Full_rank_Jac_t = B';
        Full_rank_Jac_Row_Index = colind';
        Full_rank_Jacdot_qdot_t = Jacdot_qdot_t(colind',:);
        Raw_rank_Jac_Row_Index = 1:m;
        
        % Do not forget to change the Jac_independ_index_t and
        % Jac_independ_index_in_Jac_t data set
        
        % This function is used to compare the Raw Jac with the simplified Jac to figure out which row is linearly dependent
        
        Raw_Jac_index = 1:length(Jac_independ_index_t);
        Sim_Jac_index = Full_rank_Jac_Row_Index;
        
        Jac_independ_index_t = Jac_independ_index_t(colind);
                  
        [C,ia] = setdiff(Raw_Jac_index, Sim_Jac_index);
        
        % Where C is the vectors of the different values
        % Where ia is the index of the different values in Raw_Jac_index matrix        
        
        for i = 1:length(ia)
            Deleted_index_i = ia(i);
            for j = 1:length(Jac_independ_index_in_Jac_t)
                Jac_independ_index_in_Jac_t_j = Jac_independ_index_in_Jac_t(j);
                
                if Deleted_index_i<Jac_independ_index_in_Jac_t_j
                    Jac_independ_index_in_Jac_t(j) = Jac_independ_index_in_Jac_t_j - 1;               
                end           
            end           
        end 
    end
    
end
end
function [Contact_Force_Array, Contact_Status_Array, Avg_Grid_In_Each_Mode] = Contact_Force_Initialization(N, Contact_Force_Init, Contact_Status_Init)

% This function is used to initialize the contact force array over the whole time span

Contact_Status_Init = reshape(Contact_Status_Init.',12,1).';
Contact_Status_Init = Contact_Status_Init';

Contact_Status_Target = [ 1 1 1 1 1 1 1 1 0 0 0 0]';

% A straightforward approach is to minus the desired ending contact status
% with the initial contact status such that the contact offsets are made up

Contact_Status_Offset = Contact_Status_Target - Contact_Status_Init;

Non_Zero_Index_In_Offset = find(Contact_Status_Offset);

Trans_Contact_Mode_Num = length(Non_Zero_Index_In_Offset)/2;  % The total mode number needed is Trans_Contact_Mode_Num + 1

Avg_Grid_In_Each_Mode = floor(N/(Trans_Contact_Mode_Num + 1));

Contact_Force_Array = zeros(12, N);   % Each row is the description of how the force changes during the whole time

Contact_Force_Array(:,1:Avg_Grid_In_Each_Mode) = repmat(Contact_Force_Init,1,Avg_Grid_In_Each_Mode);

Even_Row_Array = 2:2:8;

Contact_Force_Vertical_Sum = sum(Contact_Force_Init(Even_Row_Array));

Contact_Status_Temp = Contact_Status_Init;

Contact_Status_Array = Contact_Status_Temp;

for i = 1:Trans_Contact_Mode_Num   % This is used to initialize the contact forces in other mode
    % The basic idea is to add or remove contact point and then evenly distribute the vertical support force
    % The current contact status should be known 
    Contact_Force_Temp = Contact_Force_Init;  
    [Contact_Status_Temp, NOS, Vertical_Index]= Contact_Status_Calculation(Contact_Status_Temp, Contact_Status_Offset, Non_Zero_Index_In_Offset, i);
    
    Contact_Status_Array = [Contact_Status_Array Contact_Status_Temp];
    
    Contact_Force_Vertical_Avg = Contact_Force_Vertical_Sum/NOS;
    
    for j = 1:length(Vertical_Index)
        Contact_Force_Temp(Vertical_Index(j)) = Contact_Force_Vertical_Avg;      
    end
    
    Leading_Index = i * Avg_Grid_In_Each_Mode + 1;
    Ending_Index = (i + 1) * Avg_Grid_In_Each_Mode;
    
    Contact_Force_Array(:,Leading_Index:Ending_Index) = repmat(Contact_Force_Temp,1,Avg_Grid_In_Each_Mode);
    
end

% However, there may exist nonfilled columns

if N == Avg_Grid_In_Each_Mode * (Trans_Contact_Mode_Num + 1)
else
    Nonfilled_Col_Num = N - Avg_Grid_In_Each_Mode * (Trans_Contact_Mode_Num + 1);   
    % The residual contact force column from the previous part is utilized
    % here
    Contact_Force_Array(:,end-Nonfilled_Col_Num+1:end) = repmat(Contact_Force_Temp,1,Nonfilled_Col_Num);     
end
end

function [Contact_Status_Temp, NOS, Vertical_Index]= Contact_Status_Calculation(Contact_Status_Init, Contact_Status_Offset, Non_Zero_Index_In_Offset, index)

% NOS: number of support


Contact_Status_Offset_Temp = zeros(12,1);

Contact_Status_Offset_Temp(Non_Zero_Index_In_Offset(2*index-1)) = Contact_Status_Offset(Non_Zero_Index_In_Offset(2*index-1));
Contact_Status_Offset_Temp(Non_Zero_Index_In_Offset(2*index)) = Contact_Status_Offset(Non_Zero_Index_In_Offset(2*index));

Contact_Status_Temp = Contact_Status_Init + Contact_Status_Offset_Temp;

Even_Row_Array = 2:2:12;

NOS = sum(Contact_Status_Temp(Even_Row_Array));

Vertical_Index = [];


for i = 1:6   
    if Contact_Status_Temp(Even_Row_Array(i))==1     
        Vertical_Index = [Vertical_Index; 2 * i];     
    end
end
end
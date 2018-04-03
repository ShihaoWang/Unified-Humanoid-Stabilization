function [Contact_Force_Array, Q_Qdot_Array, Control_Torque_Array] = Optimal_Variables_Unzip(z, P)

% This function is used to unzip the given z vector into three main
% classes:
%             Contact_Force_Array Variables         length: 12 * N
%             Q_Qdot_Array Coefficients             length: 26 * N
%             Control_Torque_Array States           length: 10 * N

Contact_Force_Vec = z(1:(12 * P.N));
Contact_Force_Array = reshape(Contact_Force_Vec, [12, P.N]);

Q_Qdot_Vec = z(12*P.N+1:12*P.N + 26 * P.N);
Q_Qdot_Array = reshape(Q_Qdot_Vec, [26, P.N]);

Control_Torque_Vec = z(12*P.N + 26 * P.N+1:12*P.N + 26 * P.N + 10*P.N);
Control_Torque_Array = reshape(Control_Torque_Vec, [10, P.N]);

% Here the contact force has been fully unzipped
end
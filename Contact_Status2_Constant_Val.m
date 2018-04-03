function [Const_Val, Const_Index] = Contact_Status2_Constant_Val(initial_mode_status, initial_state, P)

% This function is used to find out the contact value given the leading
% contact status and leading contact value

rIx     = initial_state(1);
rIy     = initial_state(2);
theta   = initial_state(3);
q1      = initial_state(4);
q2      = initial_state(5);
q3      = initial_state(6);
q4      = initial_state(7);
q5      = initial_state(8);
q6      = initial_state(9);
q7      = initial_state(10);
q8      = initial_state(11);
q9      = initial_state(12);
q10     = initial_state(13);

rIxdot = initial_state(14);
rIydot = initial_state(15);
thetadot = initial_state(16);
q1dot = initial_state(17);
q2dot = initial_state(18);
q3dot = initial_state(19);
q4dot = initial_state(20);
q5dot = initial_state(21);
q6dot = initial_state(22);
q7dot = initial_state(23);
q8dot = initial_state(24);
q9dot = initial_state(25);
q10dot = initial_state(26);

rA_fn = P.rA_fn;
rB_fn = P.rB_fn;
rC_fn = P.rC_fn;
rD_fn = P.rD_fn;
rM_fn = P.rM_fn;
rO_fn = P.rO_fn;

rA = rA_fn(q4,q5,q6,rIx,rIy,theta);
rB = rB_fn(q4,q5,q6,rIx,rIy,theta);
rC = rC_fn(q1,q2,q3,rIx,rIy,theta);
rD = rD_fn(q1,q2,q3,rIx,rIy,theta);
rM = rM_fn(q9,q10,rIx,rIy,theta);
rO = rO_fn(q7,q8,rIx,rIy,theta);

vA = P.vA_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vB = P.vB_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vC = P.vC_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vD = P.vD_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);

vM = P.vM_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
vO = P.vO_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);

rX = cellstr({'rA', 'rB','rC','rD','rM','rO'});
vX = cellstr({'vA', 'vB','vC','vD','vM','vO'}); 

Const_Val = [];

Const_Index = [];

for i = 1:6 
    if min(initial_mode_status(2*i-1,1)*initial_mode_status(2*i-1,1), initial_mode_status(2*i,1)*initial_mode_status(2*i,1)) ~= 0
        
        Const_Val = [Const_Val; eval(rX{i}); eval(vX{i})];

        Const_Index = [Const_Index; i];      
        
    end
end

end


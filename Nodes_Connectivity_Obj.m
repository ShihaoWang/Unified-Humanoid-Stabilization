function Obj = Nodes_Connectivity_Obj(z,P)

Ctrl_No = P.Ctrl_No;
stateNdot_ref = z(2:end);
StateNdot_tot = stateNdot_ref(1:13*2*Ctrl_No,:);

StateNdot_tot = reshape(StateNdot_tot, 26, Ctrl_No);

x_final = StateNdot_tot(:,end);

rIx = x_final(1);             rIy = x_final(2);             theta = x_final(3);
q1 = x_final(4);              q2 = x_final(5);              q3 = x_final(6);
q4 = x_final(7);              q5 = x_final(8);              q6 = x_final(9);
q7 = x_final(10);             q8 = x_final(11);             q9 = x_final(12);
q10 = x_final(13);
rIxdot = x_final(1+13);          rIydot = x_final(2+13);          thetadot = x_final(3+13);
q1dot = x_final(4+13);           q2dot = x_final(5+13);           q3dot = x_final(6+13);
q4dot = x_final(7+13);           q5dot = x_final(8+13);           q6dot = x_final(9+13);
q7dot = x_final(10+13);          q8dot = x_final(11+13);          q9dot = x_final(12+13);
q10dot = x_final(13+13);

T = P.T_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q10dot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,rIxdot,rIydot,thetadot,theta);

Obj = T;

end
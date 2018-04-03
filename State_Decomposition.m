function [rIx, rIy, theta,...
    q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,...
    rIxdot, rIydot, thetadot,...
       q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot...
       ,rIxddot, rIyddot, thetaddot,...
    q1ddot,q2ddot,q3ddot,q4ddot,q5ddot,q6ddot,q7ddot,q8ddot,q9ddot,q10ddot] = State_Decomposition(z)

rIx = z(1);
rIy = z(2);
theta = z(3);
q1 = z(4);
q2 = z(5);
q3 = z(6);
q4 = z(7);
q5 = z(8);
q6 = z(9);
q7 = z(10);
q8 = z(11);
q9 = z(12);
q10 = z(13);

rIxdot = z(14);
rIydot = z(15);
thetadot = z(16);
q1dot = z(17);
q2dot = z(18);
q3dot = z(19);
q4dot = z(20);
q5dot = z(21);
q6dot = z(22);
q7dot = z(23);
q8dot = z(24);
q9dot = z(25);
q10dot = z(26);


rIxddot = z(27);
rIyddot = z(28);
thetaddot = z(29);
q1ddot = z(30);
q2ddot = z(31);
q3ddot = z(32);
q4ddot = z(33);
q5ddot = z(34);
q6ddot = z(35);
q7ddot = z(36);
q8ddot = z(37);
q9ddot = z(38);
q10ddot = z(39);

end


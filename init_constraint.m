function [c,ceq] = Init_Constraint(z,P)

rIx        = z(1);
rIy        = z(2);
theta      = z(3);
q1         = z(4);
q2         = z(5);
q3         = z(6); 
q4         = z(7);
q5         = z(8);
q6         = z(9);
q7         = z(10);
q8         = z(11);

rIxdot     = z(12);
rIydot     = z(13);
thetadot   = z(14);
q1dot      = z(15);
q2dot      = z(16);
q3dot      = z(17);
q4dot      = z(18);
q5dot      = z(19);
q6dot      = z(20);
q7dot      = z(21);
q8dot      = z(22); 

rA_fn = P.rA_fn;
rB_fn = P.rB_fn;
rC_fn = P.rC_fn;
rD_fn = P.rD_fn;
rE_fn = P.rE_fn;
rF_fn = P.rF_fn;
rG_fn = P.rG_fn;
rH_fn = P.rH_fn;
rI_fn = P.rI_fn;
rJ_fn = P.rJ_fn;
rK_fn = P.rK_fn;

vA_fn = P.vA_fn;
vB_fn = P.vB_fn;
vC_fn = P.vC_fn;
vD_fn = P.vD_fn;

rA = rA_fn(q1,q2,rIx,rIy,theta);
rB = rB_fn(q3,q4,rIx,rIy,theta);
rC = rC_fn(q5,q6,rIx,rIy,theta);
rD = rD_fn(q7,q8,rIx,rIy,theta);
rE = rE_fn(q1,rIx,rIy,theta);
rF = rF_fn(q3,rIx,rIy,theta);
rG = rG_fn(q5,rIx,rIy,theta);
rH = rH_fn(q7,rIx,rIy,theta);
rI = rI_fn(rIx,rIy);
rJ = rJ_fn(rIx,rIy,theta);
rK = rK_fn(rIx,rIy,theta);

vA = vA_fn(q1,q2,q1dot,q2dot,rIxdot,rIydot,thetadot,theta);
vB = vB_fn(q3,q4,q3dot,q4dot,rIxdot,rIydot,thetadot,theta);
% vC = vC_fn(q5,q6,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
% vD = vD_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);

init_contas = P.init_contas;
[coef_A, coef_B] = contas_decipher(init_contas);

ceq = [coef_A * rA(2);...
       coef_B * rB(2);...
       coef_A * vA;...
       coef_B * vB];

eps = 0.1;

c =   -[xor(coef_A, coef_B) * (rA(2) - coef_B * eps);
        xor(coef_A, coef_B) * (rB(2) - coef_A * eps);
        rA(2);...
        rB(2);...
        rC(2);...
        rD(2);...
        rE(2);...
        rF(2);...
        rG(2);...
        rH(2);...
        rI(2);...
        rJ(2);...
        rK(2);...
        (coef_A * coef_B) * (rB(1) - rA(1) - eps)];
end

function [coef_A, coef_B] = contas_decipher(init_contas)
coef_A = 0;

coef_B = 0;

if init_contas(1) == 1
    coef_A = 1;
end

if init_contas(2) == 1
    coef_B = 1;
end
end
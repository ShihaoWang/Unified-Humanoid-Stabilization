function Q_Qdot_Array = State_Angle_Angular_Initialization_Constraint_Optimization(init0, Contact_Status_Array, N, Avg_Grid_In_Each_Mode, P)

x0 = init0;

[~,n] = size(Contact_Status_Array);

Q_Qdot_Array = [];

for i = 1:n-1
    
    value_rAx = Contact_Status_Array(1,i+1);
    value_rAy = Contact_Status_Array(2,i+1);
    value_rBx = Contact_Status_Array(3,i+1);
    value_rBy = Contact_Status_Array(4,i+1);
    value_rCx = Contact_Status_Array(5,i+1);
    value_rCy = Contact_Status_Array(6,i+1);
    value_rDx = Contact_Status_Array(7,i+1);
    value_rDy = Contact_Status_Array(8,i+1);
    
    options.lb = [-inf,  0, -pi/2,   0,   0,      0,      0,   0,   0, -pi,   0, -pi,   0];
    options.ub = [inf, inf,  pi,  pi,  pi, 3*pi/2, 3*pi/2,  pi,  pi,  pi,  pi,  pi,  pi];
    options.cl = [0 0 0 0 0 0 0 0];
    options.cu = [0 0 0 0 0 0 0 0];
    options.auxdata.value_rAx = value_rAx;
    options.auxdata.value_rAy = value_rAy;
    options.auxdata.value_rBx = value_rBx;
    options.auxdata.value_rBy = value_rBy;
    options.auxdata.value_rCx = value_rCx;
    options.auxdata.value_rCy = value_rCy;
    options.auxdata.value_rDx = value_rDx;
    options.auxdata.value_rDy = value_rDy;
    options.auxdata.x_start = x0;
    options.auxdata.P = P;
    
    fmincon_opt = optimoptions('fmincon','Algorithm','sqp');
    
    x = fmincon(@objective,x0,[],[],[],[],options.lb,options.ub,@fmincon_constraint,fmincon_opt, options.auxdata);
       
    Q_Qdot_t = Discretization_P2N(init0, x, Avg_Grid_In_Each_Mode);
    
    Q_Qdot_Array = [Q_Qdot_Array Q_Qdot_t];
end

% To make sure all the states have been fulfilled, 

Q_Qdot_Array = [Q_Qdot_Array repmat(Q_Qdot_Array(:,end),1,N-(Avg_Grid_In_Each_Mode * (n-1)))];

end

function Q_Qdot_t = Discretization_P2N(z0_minus, z0_plus, Avg_Grid_In_Each_Mode)
Q_Qdot_t = zeros(26, Avg_Grid_In_Each_Mode);
for i = 1:26
    Q_Qdot_t(i,:) = linspace(z0_minus(i), z0_plus(i),Avg_Grid_In_Each_Mode);
end
end

function obj_val = objective(x, auxdata)
% This is the value function for the optimization

x_start = x;

x_off = 90 - 0*x_start(3);

obj_val = dot(x_off, x_off);
end

function [c, ceq] = fmincon_constraint(x, auxdata)
rIx        = x(1);
rIy        = x(2);
theta      = x(3);
q1         = x(4);
q2         = x(5);
q3         = x(6);
q4         = x(7);
q5         = x(8);
q6         = x(9);
q7         = x(10);
q8         = x(11);
q9         = x(12);
q10        = x(13);

rIxdot     = x(14);
rIydot     = x(15);
thetadot   = x(16);
q1dot      = x(17);
q2dot      = x(18);
q3dot      = x(19);
q4dot      = x(20);
q5dot      = x(21);
q6dot      = x(22);
q7dot      = x(23);
q8dot      = x(24);
q9dot      = x(25);
q10dot     = x(26);

value_rAx = auxdata.value_rAx;
value_rAy = auxdata.value_rAy;
value_rBx = auxdata.value_rBx;
value_rBy = auxdata.value_rBy;
value_rCx = auxdata.value_rCx;
value_rCy = auxdata.value_rCy;
value_rDx = auxdata.value_rDx;
value_rDy = auxdata.value_rDy;

rA_fn = auxdata.P.rA_fn;
rB_fn = auxdata.P.rB_fn;
rC_fn = auxdata.P.rC_fn;
rD_fn = auxdata.P.rD_fn;
rE_fn = auxdata.P.rE_fn;
rF_fn = auxdata.P.rF_fn;
rG_fn = auxdata.P.rG_fn;
rH_fn = auxdata.P.rH_fn;
rI_fn = auxdata.P.rI_fn;
rJ_fn = auxdata.P.rJ_fn;
rK_fn = auxdata.P.rK_fn;
rL_fn = auxdata.P.rL_fn;
rM_fn = auxdata.P.rM_fn;
rN_fn = auxdata.P.rN_fn;
rO_fn = auxdata.P.rO_fn;


vA_fn = auxdata.P.vA_fn;
vB_fn = auxdata.P.vB_fn;
vC_fn = auxdata.P.vC_fn;
vD_fn = auxdata.P.vD_fn;

rA = rA_fn(q4,q5,q6,rIx,rIy,theta);
rB = rB_fn(q4,q5,q6,rIx,rIy,theta);
rC = rC_fn(q1,q2,q3,rIx,rIy,theta);
rD = rD_fn(q1,q2,q3,rIx,rIy,theta);

rE = rE_fn(q4,q5,rIx,rIy,theta);
rF = rF_fn(q4,rIx,rIy,theta);
rG = rG_fn(q2,q3,rIx,rIy,theta);
rH = rH_fn(q3,rIx,rIy,theta);
rI = rI_fn(rIx,rIy);
rJ = rJ_fn(rIx,rIy,theta);
rK = rK_fn(rIx,rIy,theta);
rL = rL_fn(q9,rIx,rIy,theta);
rM = rM_fn(q9,q10,rIx,rIy,theta);
rN = rN_fn(q7,rIx,rIy,theta);
rO = rO_fn(q7,q8,rIx,rIy,theta);

vA = vA_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vB = vB_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vC = vC_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vD = vD_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);

[M_rAx_Pos, M_rAy_Vel] = Selection_Matrix_Cal(value_rAx,value_rAy);
[M_rBx_Pos, M_rBy_Vel] = Selection_Matrix_Cal(value_rBx,value_rBy);
[M_rCx_Pos, M_rCy_Vel] = Selection_Matrix_Cal(value_rCx,value_rCy);
[M_rDx_Pos, M_rDy_Vel] = Selection_Matrix_Cal(value_rDx,value_rDy);

ceq = [M_rAx_Pos * rA;...
    M_rBx_Pos * rB;...
    M_rCx_Pos * rC;...
    M_rDx_Pos * rD;...
    M_rAy_Vel * vA;...
    M_rBy_Vel * vB;...
    M_rCy_Vel * vC;...
    M_rDy_Vel * vD];

eps = 0.1;
eps_rAy = eps;
eps_rBy = eps;
eps_rCy = eps;
eps_rDy = eps;

if (value_rAy == 1)
    eps_rAy = 0;
end
if (value_rBy == 1)
    eps_rBy = 0;
end
if (value_rCy == 1)
    eps_rCy = 0;
end
if (value_rDy == 1)
    eps_rDy = 0;
end

c =   -[rA(2) - eps_rAy;...
    rB(2) - eps_rBy;...
    rC(2) - eps_rCy;...
    rD(2) - eps_rDy;...
    rE(2);...
    rF(2);...
    rG(2);...
    rH(2);...
    rI(2);...
    rJ(2);...
    rK(2);...
    rL(2);...
    rM(2);...
    rN(2);...
    rO(2)];
end

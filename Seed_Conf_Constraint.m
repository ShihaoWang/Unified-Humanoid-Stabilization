function [c, ceq] = Seed_Conf_Constraint(z,P)
c = []; ceq = [];

rIx = z(1);             rIy = z(2);             theta = z(3);
q1 = z(4);              q2 = z(5);              q3 = z(6);
q4 = z(7);              q5 = z(8);              q6 = z(9);
q7 = z(10);             q8 = z(11);             q9 = z(12);
q10 = z(13);

rIxdot = z(1+13);          rIydot = z(2+13);          thetadot = z(3+13);
q1dot = z(4+13);           q2dot = z(5+13);           q3dot = z(6+13);
q4dot = z(7+13);           q5dot = z(8+13);           q6dot = z(9+13);
q7dot = z(10+13);          q8dot = z(11+13);          q9dot = z(12+13);
q10dot = z(13+13);

rIx_ref = P.Refer0(1);      rIy_ref = P.Refer0(2);      theta_ref = P.Refer0(3);
q1_ref = P.Refer0(4);       q2_ref = P.Refer0(5);       q3_ref = P.Refer0(6);
q4_ref = P.Refer0(7);       q5_ref = P.Refer0(8);       q6_ref = P.Refer0(9);
q7_ref = P.Refer0(10);      q8_ref = P.Refer0(11);      q9_ref = P.Refer0(12);     q10_ref = P.Refer0(13);   

rA = P.rA_fn(q1,q2,q3,rIx,rIy,theta);
rB = P.rB_fn(q1,q2,q3,rIx,rIy,theta);
rC = P.rC_fn(q4,q5,q6,rIx,rIy,theta);
rD = P.rD_fn(q4,q5,q6,rIx,rIy,theta);
rE = P.rE_fn(q7,q8,rIx,rIy,theta);
rF = P.rF_fn(q9,q10,rIx,rIy,theta);
% rG = P.rG_fn(q1,q2,rIx,rIy,theta);
% rH = P.rH_fn(q1,rIx,rIy,theta);
rI = P.rI_fn(rIx,rIy);
% rJ = P.rJ_fn(q4,q5,rIx,rIy,theta);
% rK = P.rK_fn(q4,rIx,rIy,theta);
% rL = P.rL_fn(rIx,rIy,theta);
% rM = P.rM_fn(q7,rIx,rIy,theta);
% rN = P.rN_fn(q9,rIx,rIy,theta);
% rT = P.rT_fn(rIx,rIy,theta);

vA = P.vA_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vB = P.vB_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vC = P.vC_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vD = P.vD_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vE = P.vE_fn(q7,q8,q7dot,q8dot,rIxdot,rIydot,thetadot,theta);
vF = P.vF_fn(q9,q10,q10dot,q9dot,rIxdot,rIydot,thetadot,theta);
% vG = P.vG_fn(q1,q2,q1dot,q2dot,rIxdot,rIydot,thetadot,theta);
% vH = P.vH_fn(q1,q1dot,rIxdot,rIydot,thetadot,theta);
% vI = P.vI_fn(rIxdot,rIydot);
% vJ = P.vJ_fn(q4,q5,q4dot,q5dot,rIxdot,rIydot,thetadot,theta);
% vK = P.vK_fn(q4,q4dot,rIxdot,rIydot,thetadot,theta);
% vL = P.vL_fn(rIxdot,rIydot,thetadot,theta);
% vM = P.vM_fn(q7,q7dot,rIxdot,rIydot,thetadot,theta);
% vN = P.vN_fn(q9,q9dot,rIxdot,rIydot,thetadot,theta);

% According to the way that sigma_child is generated, this offset vector
% can at most have one nonzero value
sigma_offset = P.sigma_i_child - P.sigma_i; 
P.sigma_offset = sigma_offset;

% sigma_i_child_AB = P.sigma_i_child(1);
% sigma_i_child_CD = P.sigma_i_child(2);
% sigma_i_child_E = P.sigma_i_child(3);
% sigma_i_child_F = P.sigma_i_child(4);

sigma_i_AB = P.sigma_i(1);
sigma_i_CD = P.sigma_i(2);
sigma_i_E = P.sigma_i(3);
sigma_i_F = P.sigma_i(4);


sigma_i_child_AB = P.sigma_i_child(1);
sigma_i_child_CD = P.sigma_i_child(2);
sigma_i_child_E = P.sigma_i_child(3);
sigma_i_child_F = P.sigma_i_child(4);

%% 1. Relative distance constraints: a. all distances have to be at least on the surface; b. the desired mode has to be satisfied.
c = [c; -Obs_Dist_Fn(rA, P.Envi_Map) + P.mini * ~sigma_i_child_AB;...
        -Obs_Dist_Fn(rB, P.Envi_Map) + P.mini * ~sigma_i_child_AB;...
        -Obs_Dist_Fn(rC, P.Envi_Map) + P.mini * ~sigma_i_child_CD;...
        -Obs_Dist_Fn(rD, P.Envi_Map) + P.mini * ~sigma_i_child_CD;...
        -Obs_Dist_Fn(rE, P.Envi_Map) + P.mini * ~sigma_i_child_E;...
        -Obs_Dist_Fn(rF, P.Envi_Map) + P.mini * ~sigma_i_child_F];
    
ceq = [ceq; sigma_i_child_AB * rA(2); sigma_i_child_AB * vA;...
            sigma_i_child_AB * rB(2); sigma_i_child_AB * vB;...
            sigma_i_child_CD * rC(2); sigma_i_child_CD * vC;...
            sigma_i_child_CD * rD(2); sigma_i_child_CD * vD;...
            sigma_i_child_E * Obs_Dist_Fn(rE, P.Envi_Map, 'x'); sigma_i_child_E * vE;...
            sigma_i_child_F * Obs_Dist_Fn(rF, P.Envi_Map, 'x'); sigma_i_child_F * vF];
 
%% 2. Contact Constraint Maintenance: the previous contacts have to be satisfied
if max(sigma_offset)==1
    % This is a contact addition
        ceq = [ceq; sigma_i_AB * (rA - P.rA_ref); sigma_i_AB * vA;
                    sigma_i_AB * (rB - P.rB_ref); sigma_i_AB * vB;
                    sigma_i_CD * (rC - P.rC_ref); sigma_i_CD * vC;
                    sigma_i_CD * (rD - P.rD_ref); sigma_i_CD * vD;
                    sigma_i_E * (rE - P.rB_ref);  sigma_i_E * vE; 
                    sigma_i_F * (rF - P.rB_ref);  sigma_i_F * vF]; 
else
    % This is a contact reduction  
        ceq = [ceq; (sigma_offset(1)==0) * sigma_i_AB * (rA - P.rA_ref); (sigma_offset(1)==0) * sigma_i_AB * vA;
                    (sigma_offset(1)==0) * sigma_i_AB * (rB - P.rB_ref); (sigma_offset(1)==0) * sigma_i_AB * vB;
                    (sigma_offset(2)==0) * sigma_i_CD * (rC - P.rC_ref); (sigma_offset(2)==0) * sigma_i_CD * vC;
                    (sigma_offset(2)==0) * sigma_i_CD * (rD - P.rD_ref); (sigma_offset(2)==0) * sigma_i_CD * vD;
                    (sigma_offset(3)==0) * sigma_i_E *  (rE - P.rB_ref); (sigma_offset(3)==0) * sigma_i_E * vE;
                    (sigma_offset(4)==0) * sigma_i_F *  (rF - P.rB_ref); (sigma_offset(4)==0) * sigma_i_F * vF]; 
end
%% 3. Heuristic Stability Constraints: rI and rCOM have to be lied within the support polygon
r_Foot_Pos = [rA(1) rB(1);  rC(1) rD(1)];
rCOM = P.rCOM_fn(q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,rIx,rIy,theta);
% % This is the heuristic part
if (max(P.sigma_i) == min(P.sigma_i))&&(max(P.sigma_i) == 0)
    % There is no contact in the parent mode
    % So the robot is jumping in the air so the idea is to make sure that the center of mass align
    % with the neck position in the horizontal direction
    ceq = [ceq; rCOM(1) - rT(1)];
else
    % There is at least one contact in the parent mode
    if(P.sigma_i(1)==1)||(P.sigma_i(2)==1)        
        % At least foot contact is involved
        if find(sigma_offset)<3
            % The change is also foot contact
            if (P.vI_ref(1)>0)
                % The robot is moving forward               
                [~,n] = find(sigma_offset);
                Swing_Leg_Ind = n;
                Stance_Leg_Ind = (Swing_Leg_Ind == 1)+ 1;
                Swing_Leg_Dis = min(r_Foot_Pos(Swing_Leg_Ind,:));
                Stance_Leg_Dis = max(r_Foot_Pos(Stance_Leg_Ind,:));
                c = [c; Stance_Leg_Dis - Swing_Leg_Dis];
                c = [c; min(r_Foot_Pos(Stance_Leg_Ind,:)) - rI(1);...
                    rI(1) - max(r_Foot_Pos(Swing_Leg_Ind,:))];
                c = [c; min(r_Foot_Pos(Stance_Leg_Ind,:)) - rCOM(1);...
                    rCOM(1) - max(r_Foot_Pos(Swing_Leg_Ind,:))];
            else
                [~,n] = find(sigma_offset);
                Swing_Leg_Ind = n;
                Stance_Leg_Ind = (Swing_Leg_Ind == 1)+ 1;
                Swing_Leg_Dis = max(r_Foot_Pos(Swing_Leg_Ind,:));
                Stance_Leg_Dis = min(r_Foot_Pos(Stance_Leg_Ind,:));
                c = [c; -Stance_Leg_Dis + Swing_Leg_Dis];
                c = [c; -max(r_Foot_Pos(Stance_Leg_Ind,:)) + rI(1);...
                    -rI(1) + min(r_Foot_Pos(Swing_Leg_Ind,:))];
                c = [c; -max(r_Foot_Pos(Stance_Leg_Ind,:)) + rCOM(1);...
                    -rCOM(1) + min(r_Foot_Pos(Swing_Leg_Ind,:))];
            end
        else
            ceq = [ceq; rCOM(1) - rT(1)];
        end 
    end
end
end
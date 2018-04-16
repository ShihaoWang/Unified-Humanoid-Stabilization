function [c,ceq] = Nodes_Connectivity_Constraint(z, P)

c = []; ceq = [];
Ctrl_No = P.Ctrl_No;

t_span = [0 z(1)];
t_intp_span = linspace(t_span(1),t_span(2),Ctrl_No);
delta_t = t_intp_span(2) - t_intp_span(1);    

stateNdot_ref = z(2:end);
StateNdot_tot = stateNdot_ref(1:13*2*Ctrl_No,:);

Ctrl_tot = stateNdot_ref(length(StateNdot_tot)+1:end,:);
StateNdot_tot = reshape(StateNdot_tot, 26, Ctrl_No);

Ctrl_tot = reshape(Ctrl_tot, 10, Ctrl_No);

Active_In = P.Active_In;

sigma_offset = P.sigma_i_child - P.sigma_i; 
P.sigma_offset = sigma_offset;

sigma_i_child_AB = P.sigma_i_child(1);
sigma_i_child_CD = P.sigma_i_child(2);
sigma_i_child_E = P.sigma_i_child(3);
sigma_i_child_F = P.sigma_i_child(4);

sigma_i_AB = P.sigma_i(1);
sigma_i_CD = P.sigma_i(2);
sigma_i_E = P.sigma_i(3);
sigma_i_F = P.sigma_i(4);

for i = 1:Ctrl_No
    
    stateNdot_i = StateNdot_tot(:,i);
    
    Ctrl_i = Ctrl_tot(:,i);
    
    rIx_i = stateNdot_i(1);             rIy_i = stateNdot_i(2);             theta_i = stateNdot_i(3);
    q1_i = stateNdot_i(4);              q2_i = stateNdot_i(5);              q3_i = stateNdot_i(6);
    q4_i = stateNdot_i(7);              q5_i = stateNdot_i(8);              q6_i = stateNdot_i(9);
    q7_i = stateNdot_i(10);             q8_i = stateNdot_i(11);             q9_i = stateNdot_i(12);
    q10_i = stateNdot_i(13);
    rIxdot_i = stateNdot_i(1+13);          rIydot_i = stateNdot_i(2+13);          thetadot_i = stateNdot_i(3+13);
    q1dot_i = stateNdot_i(4+13);           q2dot_i = stateNdot_i(5+13);           q3dot_i = stateNdot_i(6+13);
    q4dot_i = stateNdot_i(7+13);           q5dot_i = stateNdot_i(8+13);           q6dot_i = stateNdot_i(9+13);
    q7dot_i = stateNdot_i(10+13);          q8dot_i = stateNdot_i(11+13);          q9dot_i = stateNdot_i(12+13);
    q10dot_i = stateNdot_i(13+13);
    
    xstate_i = stateNdot_i(1:13,:);
    xstatedot_i = stateNdot_i(14:26,:);
    
    if i == Ctrl_No
        x_statep1 = xstate_i;
    else
        x_statep1 = StateNdot_tot(1:13,i+1);
    end
    
    qddot = (x_statep1 - xstate_i - xstatedot_i * delta_t)/(1/2 * delta_t^2) ;
    
    D_q = P.D_q_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,theta_i);
    B_q = P.B_q_fn();
    C_q_qdot = P.C_q_qdot_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,...
        q10dot_i,q1dot_i,q2dot_i,q3dot_i,q4dot_i,q5dot_i,q6dot_i,q7dot_i,q8dot_i,q9dot_i,thetadot_i,theta_i);
    
    Jac_Full = P.Jac_Full_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,theta_i);
    Jacdot_qdot_Full = P.Jacdot_qdot_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,...
        q10dot_i,q1dot_i,q2dot_i,q3dot_i,q4dot_i,q5dot_i,q6dot_i,q7dot_i,q8dot_i,q9dot_i,thetadot_i,theta_i);
    
    Jac = Jac_Full(Active_In,:);
    Jacdot_qdot = Jacdot_qdot_Full(Active_In,:);
    
    lamda = (Jac*D_q^(-1)*Jac')\(Jac*D_q^(-1)*C_q_qdot-Jacdot_qdot-Jac*D_q^(-1)*B_q*Ctrl_i);
    
    %% 1. Contact force constraints
    if max(Active_In)<5  % Then it is all foot contact
        if find(Active_In)==0
            return
        else
            for j = 1:length(find(Active_In))
                Lamda_Ind = Active_In(j);
                if Lamda_Ind<5
                    if mod(Lamda_Ind,2)==0
                        c = [c; -lamda(j)];
                    end
                else
                    if mod(Lamda_Ind,2)==1
                        c = [c; lamda(j)];
                    end
                    
                end
            end
        end
    end
    
    %% 2. Dynamics constraints
    Dyn_Via = D_q * qddot + C_q_qdot - Jac' * lamda - B_q * Ctrl_i;
    ceq = [ceq; Dyn_Via];
    
    %% 3. Contact distance constraints
    rA = P.rA_fn(q1_i,q2_i,q3_i,rIx_i,rIy_i,theta_i);
    rB = P.rB_fn(q1_i,q2_i,q3_i,rIx_i,rIy_i,theta_i);
    rC = P.rC_fn(q4_i,q5_i,q6_i,rIx_i,rIy_i,theta_i);
    rD = P.rD_fn(q4_i,q5_i,q6_i,rIx_i,rIy_i,theta_i);
    rE = P.rE_fn(q7_i,q8_i,rIx_i,rIy_i,theta_i);
    rF = P.rF_fn(q9_i,q10_i,rIx_i,rIy_i,theta_i);
    %     rG = P.rG_fn(q1_i,q2_i,rIx_i,rIy_i,theta_i);
    %     rH = P.rH_fn(q1_i,rIx_i,rIy_i,theta_i);
        rI = P.rI_fn(rIx_i,rIy_i);
    %     rJ = P.rJ_fn(q4_i,q5_i,rIx_i,rIy_i,theta_i);
    %     rK = P.rK_fn(q4_i,rIx_i,rIy_i,theta_i);
    %     rL = P.rL_fn(rIx_i,rIy_i,theta_i);
    %     rM = P.rM_fn(q7_i,rIx_i,rIy_i,theta_i);
    %     rN = P.rN_fn(q9_i,rIx_i,rIy_i,theta_i);
    %     rT = P.rT_fn(rIx_i,rIy_i,theta_i);
    %
    
    vA = P.vA_fn(q1_i,q2_i,q3_i,q1dot_i,q2dot_i,q3dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    vB = P.vB_fn(q1_i,q2_i,q3_i,q1dot_i,q2dot_i,q3dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    vC = P.vC_fn(q4_i,q5_i,q6_i,q4dot_i,q5dot_i,q6dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    vD = P.vD_fn(q4_i,q5_i,q6_i,q4dot_i,q5dot_i,q6dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    vE = P.vE_fn(q7_i,q8_i,q7dot_i,q8dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    vF = P.vF_fn(q9_i,q10_i,q10dot_i,q9dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    
    %     vG = P.vG_fn(q1_i,q2_i,q1dot_i,q2dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    %     vH = P.vH_fn(q1_i,q1dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    %     vI = P.vI_fn(rIxdot_i,rIydot_i);
    %     vJ = P.vJ_fn(q4_i,q5_i,q4dot_i,q5dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    %     vK = P.vK_fn(q4_i,q4dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    %     vL = P.vL_fn(rIxdot_i,rIydot_i,thetadot_i,theta_i);
    %     vM = P.vM_fn(q7_i,q7dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    %     vN = P.vN_fn(q9_i,q9dot_i,rIxdot_i,rIydot_i,thetadot_i,theta_i);
    
    c = [c; -Obs_Dist_Fn(rA, P.Envi_Map);...
            -Obs_Dist_Fn(rB, P.Envi_Map);...
            -Obs_Dist_Fn(rC, P.Envi_Map);...
            -Obs_Dist_Fn(rD, P.Envi_Map);...
            -Obs_Dist_Fn(rE, P.Envi_Map);...
            -Obs_Dist_Fn(rF, P.Envi_Map)];
    c = [c; -rA(2);...
            -rB(2);...
            -rC(2);...
            -rD(2);...
            -rE(2);...
            -rF(2)];
    if i == Ctrl_No
        ceq = [ceq; sigma_i_child_AB * rA(2); sigma_i_child_AB * vA;...
            sigma_i_child_AB * rB(2); sigma_i_child_AB * vB;...
            sigma_i_child_CD * rC(2); sigma_i_child_CD * vC;...
            sigma_i_child_CD * rD(2); sigma_i_child_CD * vD;...
            sigma_i_child_E * Obs_Dist_Fn(rE, P.Envi_Map, 'x'); sigma_i_child_E * vE;...
            sigma_i_child_F * Obs_Dist_Fn(rF, P.Envi_Map, 'y'); sigma_i_child_F * vF];    
    end
    
    %% 4. Holonomic constraint reserved
    if max(sigma_offset)==1
        % This is a contact addition
        if max(P.sigma_i)==1
            ceq = [ceq; sigma_i_AB * (rA(1) - P.rA_ref(1));...
                        sigma_i_AB * (rB(1) - P.rB_ref(1));...
                        sigma_i_CD * (rC(1) - P.rC_ref(1));...
                        sigma_i_CD * (rD(1) - P.rD_ref(1));...
                        sigma_i_E * (rE(2) - P.rB_ref(2));...
                        sigma_i_F * (rF(2) - P.rB_ref(2))];
        end
    else
        % This is a contact reduction
        if max(P.sigma_i)==1
            ceq = [ceq; (sigma_offset(1)==0)*sigma_i_AB * (rA(1) - P.rA_ref(1));...
                        (sigma_offset(1)==0)*sigma_i_AB * (rB(1) - P.rB_ref(1));...
                        (sigma_offset(2)==0)*sigma_i_CD * (rC(1) - P.rC_ref(1));...
                        (sigma_offset(2)==0)*sigma_i_CD * (rD(1) - P.rD_ref(1));...
                        (sigma_offset(3)==0)*sigma_i_E *  (rE(2) - P.rB_ref(2));...
                        (sigma_offset(4)==0)*sigma_i_F *  (rF(2) - P.rB_ref(2))];
        end
    end
    %% 5. Heuristic Constraint
    r_Foot_Pos = [rA(1) rB(1);
        rC(1) rD(1)];
    rCOM = P.rCOM_fn(q1_i,q2_i,q3_i,q4_i,q5_i,q6_i,q7_i,q8_i,q9_i,q10_i,rIx_i,rIy_i,theta_i);
    if find(sigma_offset)<3
        % There is a change of the foot contact point
        if (P.vI_ref(1)>0)
            % The robot is moving forward
            [~,n] = max(sigma_offset);
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
            [~,n] = max(sigma_offset);
            Swing_Leg_Ind = n;
            Stance_Leg_Ind = (Swing_Leg_Ind == 1)+ 1;
            Swing_Leg_Dis = max(r_Foot_Pos(Swing_Leg_Ind,:));
            Stance_Leg_Dis = min(r_Foot_Pos(Stance_Leg_Ind,:));
            c = [c; -Stance_Leg_Dis + Swing_Leg_Dis];
            c = [c; -max(r_Foot_Pos(Stance_Leg_Ind,:)) + rI(1);...
                -rI(1) + min(r_Foot_Pos(Swing_Leg_Ind,:))];
            c = [c; -max(r_Foot_Pos(Stance_Leg_Ind,:)) + rCOM(1);...
                -rrCOM(1) + min(r_Foot_Pos(Swing_Leg_Ind,:))];
        end
        
    end
end

end

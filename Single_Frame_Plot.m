function Single_Frame_Plot(q_array_i, P)

% This function plot the robot configuration given the current q_array
rIx = q_array_i(1);    rIy = q_array_i(2);   theta = q_array_i(3);
q1 = q_array_i(4);     q2 = q_array_i(5);    q3 = q_array_i(6);
q4 = q_array_i(7);     q5 = q_array_i(8);    q6 = q_array_i(9);
q7 = q_array_i(10);    q8 = q_array_i(11);  

rA = P.rA_fn(q1,q2,rIx,rIy,theta);
rB = P.rB_fn(q3,q4,rIx,rIy,theta);
rC = P.rC_fn(q5,q6,rIx,rIy,theta);
rD = P.rD_fn(q7,q8,rIx,rIy,theta);

rE = P.rE_fn(q1,rIx,rIy,theta);
rF = P.rF_fn(q3,rIx,rIy,theta);
rG = P.rG_fn(q5,rIx,rIy,theta);
rH = P.rH_fn(q7,rIx,rIy,theta);
rI = P.rI_fn(rIx,rIy);
rJ = P.rJ_fn(rIx,rIy,theta);
rK = P.rK_fn(rIx,rIy,theta);

link_color = [8 128 224] ./ 255;
Linewidth_Coef = 2.5;
axes_plot = axes;
plot(axes_plot, [rA(1),rE(1)]',[rA(2),rE(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
plot(axes_plot, [rE(1),rI(1)]',[rE(2),rI(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
plot(axes_plot, [rI(1),rF(1)]',[rI(2),rF(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
plot(axes_plot, [rF(1),rB(1)]',[rF(2),rB(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
plot(axes_plot, [rI(1),rK(1)]',[rI(2),rK(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
plot(axes_plot, [rJ(1),rH(1)]',[rJ(2),rH(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
plot(axes_plot, [rH(1),rD(1)]',[rH(2),rD(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
plot(axes_plot, [rJ(1),rG(1)]',[rJ(2),rG(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
plot(axes_plot, [rG(1),rC(1)]',[rG(2),rC(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');

MarkSize_Coef = 5;
MarkerEdgeColor_Coef = [28 148 149] ./ 255;
MarkerFaceColor_Coef = [225 9 92] ./ 255;

% Edge point plots
for i = 1:11
    r_Point = strcat('r',char(i + 64));
    r_Point_Var = genvarname(r_Point);
    evalc(['r_Point_Plot', ' = ', r_Point_Var]);
    plot(axes_plot, r_Point_Plot(1),r_Point_Plot(2),'o','MarkerSize',MarkSize_Coef,...
        'MarkerEdgeColor',MarkerEdgeColor_Coef,...
        'MarkerFaceColor',MarkerFaceColor_Coef);  hold on;
end
axis(axes_plot, 'equal');
hold(axes_plot,'off');

end
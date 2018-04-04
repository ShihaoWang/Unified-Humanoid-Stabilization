function Single_Frame_Plot(q_array_i, P)

% This function plot the robot configuration given the current q_array
rIx = q_array_i(1);    rIy = q_array_i(2);   theta = q_array_i(3);
q1 = q_array_i(4);     q2 = q_array_i(5);    q3 = q_array_i(6);
q4 = q_array_i(7);     q5 = q_array_i(8);    q6 = q_array_i(9);
q7 = q_array_i(10);    q8 = q_array_i(11);  

rA_fn = P.rA_fn;%@(q1,q2,rIx,rIy,theta)
rB_fn = P.rB_fn;%@(q3,q4,rIx,rIy,theta)
rC_fn = P.rC_fn;%@(q5,q6,rIx,rIy,theta)
rD_fn = P.rD_fn;%@(q7,q8,rIx,rIy,theta)
rE_fn = P.rE_fn;%@(q1,rIx,rIy,theta)
P.rF_fn = matlabFunction(rF);%@(q3,rIx,rIy,theta)
P.rG_fn = matlabFunction(rG);%@(q5,rIx,rIy,theta)
P.rH_fn = matlabFunction(rH);%@(q7,rIx,rIy,theta)
P.rI_fn = matlabFunction(rI);%@(rIx,rIy)
P.rJ_fn = matlabFunction(rJ);%@(rIx,rIy,theta)
P.rK_fn = matlabFunction(rK);%@(rIx,rIy,theta)

link_color = [8 128 224] ./ 255;
Linewidth_Coef = 2.5;
% Stance foot plot
plot(axes_plot, [rA(1),rB(1)]',[rA(2),rB(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
% Stance shank plot
plot(axes_plot, [rE(1),rF(1)]',[rE(2),rF(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
% Stance thigh plot
plot(axes_plot, [rF(1),rI(1)]',[rF(2),rI(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
% Body plot
plot(axes_plot, [rI(1),rK(1)]',[rI(2),rK(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
% Stance side forearm plot
plot(axes_plot, [rJ(1),rL(1)]',[rJ(2),rL(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');
% Stance side arm plot
plot(axes_plot, [rL(1),rM(1)]',[rL(2),rM(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(axes_plot,'on');

% Swing foot plot
plot(axes_plot, [rC(1),rD(1)]',[rC(2),rD(2)]','LineWidth',2.5, 'color', link_color);
hold(axes_plot,'on');
% Swing shank plot
plot(axes_plot, [rG(1),rH(1)]',[rG(2),rH(2)]','LineWidth',2.5, 'color', link_color);
hold(axes_plot,'on');
% Swing thigh plot
plot(axes_plot, [rH(1),rI(1)]',[rH(2),rI(2)]','LineWidth',2.5, 'color', link_color);
hold(axes_plot,'on');

% Swing side forearm plot
plot(axes_plot, [rJ(1),rN(1)]',[rJ(2),rN(2)]','LineWidth',2.5, 'color', link_color);
hold(axes_plot,'on');
% Swing side arm plot
plot(axes_plot, [rN(1),rO(1)]',[rN(2),rO(2)]','LineWidth',2.5, 'color', link_color);
hold(axes_plot,'on');

MarkSize_Coef = 5;
MarkerEdgeColor_Coef = [28 148 149] ./ 255;
MarkerFaceColor_Coef = [225 9 92] ./ 255;
% Edge point plots
plot(axes_plot, rA(1),rA(2),'o','MarkerSize',MarkSize_Coef,...
    'MarkerEdgeColor',MarkerEdgeColor_Coef,...
    'MarkerFaceColor',MarkerFaceColor_Coef);  hold(handles.axes1,'on');

plot(axes_plot, rB(1),rB(2),'o','MarkerSize',MarkSize_Coef,...
    'MarkerEdgeColor',MarkerEdgeColor_Coef,...
    'MarkerFaceColor',MarkerFaceColor_Coef);  hold(handles.axes1,'on');

for i = 1:15
    r_Point = strcat('r',char(i + 64));
    r_Point_Var = genvarname(r_Point);
    evalc(['r_Point_Plot', ' = ', r_Point_Var]);
    plot(axes_plot, r_Point_Plot(1),r_Point_Plot(2),'o','MarkerSize',MarkSize_Coef,...
        'MarkerEdgeColor',MarkerEdgeColor_Coef,...
        'MarkerFaceColor',MarkerFaceColor_Coef);  hold(handles.axes1,'on');
end
axis(axes_plot, 'equal');
hold(axes_plot,'off');

end
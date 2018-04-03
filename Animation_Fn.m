function Animation_Fn(q_tot, handles)

p = Humanoid_Physical_Parameter();


[m,n] = size(q_tot);

% This function is used to plot the animation of a given configuration(s)

linklength = p.l;

if (n ==1)
    q_tot = q_tot';
    Robot_Config_Plot(q_tot, linklength, handles)
else
    for i = 1:m
        q_array_i = q_tot(i,:);
        Robot_Config_Plot(q_array_i, linklength, handles);
    end
end
end

function Robot_Config_Plot(q_array, linklength, handles)

axes_plot = handles.axes3;

% This function plot the robot configuration given the current q_array
rIx = q_array(1);    rIy = q_array(2);   theta = q_array(3);
q1 = q_array(4);     q2 = q_array(5);    q3 = q_array(6);
q4 = q_array(7);     q5 = q_array(8);    q6 = q_array(9);
q7 = q_array(10);    q8 = q_array(11);   q9 = q_array(12);    q10 = q_array(13);

AngxIK = theta;
AngxIH = -(2 * pi - theta - q3);
AngxIF = -(2 * pi - theta - q4);
AngIFx = pi + AngxIF;
AngIHx = pi + AngxIH;
AngxFE = -(2 * pi - AngIFx - q5);
AngFEx = pi + AngxFE;
AngxBA = -(q6 - AngFEx);
AngxHG = -(2 * pi - AngIHx  -q2);
AngHGx = pi + AngxHG;
AngxDC = -(q1 - AngHGx);
AngxJL = -(pi - AngxIK - q9);
AngJLx = pi + AngxJL;
AngMLx = q10 - (pi - AngJLx);
AngxJN = -(q7 + q9 -AngxJL);
AngJNx = pi + AngxJN;
AngxNO = -(pi - q8 - AngJNx);

rI = [rIx, rIy]';
rIF = linklength(3) * lamdadirection(AngxIF);
rF = rI + rIF;
rE = linklength(2) * lamdadirection(AngxFE) + rF;
rA = 1/2 * linklength(1) * lamdadirection(AngxBA) + rE;
rB = 1/2 * linklength(1) * lamdadirection(pi + AngxBA) + rE;
rH = rI + linklength(3) * lamdadirection(AngxIH);
rG = rH + linklength(2) * lamdadirection(AngxHG);
rC = rG + 1/2 * linklength(1) * lamdadirection(AngxDC);
rD = rG - 1/2 * linklength(1) * lamdadirection(AngxDC);
rJ = rI + linklength(4) * lamdadirection(AngxIK);
rK = rJ + linklength(5) * lamdadirection(AngxIK);
rL = rJ + linklength(6) * lamdadirection(AngxJL);
rM = rL + linklength(7) * lamdadirection(AngMLx);
rN = rJ + linklength(6) * lamdadirection(AngxJN);
rO = rN + linklength(7) * lamdadirection(AngxNO);

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

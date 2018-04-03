function Eleven_Link_Sim(contact_status_ref, q_ref, handles)
P = handles.P;
global contact_status
contact_status  = contact_status_reformulate(contact_status_ref);

global events_state

rIx       = q_ref(1);
rIy       = q_ref(2);
theta     = q_ref(3);                % Checked
q1        = q_ref(4);                % Checked
q2        = q_ref(5);                % Checked
q3        = q_ref(6);                % Checked
q4        = q_ref(7);                % Checked
q5        = q_ref(8);                % Checked
q6        = q_ref(9);                % Checked
q7        = q_ref(10);               % Checked
q8        = q_ref(11);               % Checked
q9        = q_ref(12);               % Checked
q10       = q_ref(13);               % Checked

rIxdot    = q_ref(14);
rIydot    = q_ref(15);
thetadot  = q_ref(16);
q1dot     = q_ref(17);
q2dot     = q_ref(18);
q3dot     = q_ref(19);
q4dot     = q_ref(20);
q5dot     = q_ref(21);
q6dot     = q_ref(22);
q7dot     = q_ref(23);
q8dot     = q_ref(24);
q9dot     = q_ref(25);
q10dot    = q_ref(26);

init = [rIx, rIy, theta,...
        q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,...
        rIxdot, rIydot, thetadot,...
       q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot]';

P.mu = 0.5;        % Initialization of the friction coefficient
P.u = zeros(10,1);


simu_duration = 1;
simu_duration_i = 0;

simu_t = [];
simu_z = [];
simu_te = [];
simu_ye = [];
simu_ie = [];

tspan = linspace(0,simu_duration,101);
odeopt = odeset('AbsTol', 1e-7, 'RelTol', 1e-7, 'Events', @Contact_Detection);

while(simu_duration_i<simu_duration)
    [t_i,z_i,te_i,ye_i,ie_i] = ode23(@Eleven_Link_Model_ODE, tspan, init, odeopt, P);
    tspan = tspan + t_i(end) + t_i(2) - t_i(1); 
    simu_t = [simu_t; t_i];
    simu_z = [simu_z; z_i];
    simu_te = [simu_te; te_i];
    simu_ye = [simu_ye; ye_i];
    simu_ie = [simu_ie; ie_i];
%     events_state_plot(events_state);
    [init, contact_status] = Robot_State_Mode_Shaping(ye_i(end,:), ie_i(end), contact_status, P);   
    Eleven_Link_Animation(z_i,handles);
end


end

function events_state_plot(events_state)

% This function is used to plot the event state

figure() 
subplot(3,2,1);
plot(events_state(:,1));
title('point A vertical position');
subplot(3,2,2);
plot(events_state(:,3));
title('point B vertical position');

subplot(3,2,3);
plot(events_state(:,5));
title('point C vertical position');

subplot(3,2,4);
plot(events_state(:,7));
title('point D vertical position');

subplot(3,2,5);
plot(events_state(:,9));
title('point M vertical position');

subplot(3,2,6);
plot(events_state(:,11));
title('point O vertical position');

figure 
subplot(3,2,1);
plot(events_state(:,2));
title('point A vertical force');

subplot(3,2,2);
plot(events_state(:,4));
title('point B vertical force');

subplot(3,2,3);
plot(events_state(:,6));
title('point C vertical force');

subplot(3,2,4);
plot(events_state(:,8));
title('point D vertical force');

subplot(3,2,5);
plot(events_state(:,10));
title('point M vertical force');

subplot(3,2,6);
plot(events_state(:,12));
title('point O vertical force');

end
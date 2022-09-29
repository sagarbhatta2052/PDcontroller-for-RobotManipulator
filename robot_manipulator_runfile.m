%%% *Joint Reference tracking for robot manipulator with time delay estimation* 
%%% the idea of this code is the implementation of PD controller
%%% for 3 joint robot manipulator in joint space to minimized the joints error. 

%% clear and close all the warning
clc;
clear;
close all;
warning off;

%% Definition of variables

End_time = 40;                  % total time intervel
Ts = 0.001;                        % sampling time  

%% initialization of states and input variables
% state variables q = [thita1,thita2,thita2]
q = [0,0.5,0]';
qd = [0,0,0]';
qdd = [0,0,0]';
T = [0,0,0]';

% Define the actual system states and inputs for the plots for the whole simulation range

q_t = zeros(3,End_time/Ts+1);
qd_t = zeros(3,End_time/Ts+1);
qdd_t = zeros(3,End_time/Ts+1);
q_t(:,1) = q;
qd_t(:,1) = qd;
qdd_t(:,1) = qdd;

T_t = zeros(3,End_time/Ts+1);
T_t(:,1) = T;


k = 2;
for t = 0:Ts:End_time-Ts

   
    
% trajectory 0 check

q_ref_1 = 0.5*sin(pi/5*(t));
qd_ref_1 = 0.5*pi/5*cos(pi/5*(t));
qdd_ref_1 = -0.5*pi*pi/25*sin(pi/5*(t));
q_ref_2 = 0.5*cos(pi/5*(t));
qd_ref_2 = -0.5*pi/5*sin(pi/5*(t));
qdd_ref_2 = -0.5*pi*pi/25*cos(pi/5*(t));
q_ref_3 = sin(0.5*t);
qd_ref_3 = 0.5*cos(0.5*t);
qdd_ref_3 = -0.5*0.5*sin(0.5*t);

% trajectory 1 check

% q_ref_1 = cos(0.5*t + pi/3);
% qd_ref_1 = -0.5*sin(0.5*t + pi/3);
% qdd_ref_1 = -0.5*0.5*cos(0.5*t + pi/3);
% q_ref_2 = sin(0.5*t + pi/5);
% qd_ref_2 = 0.5*cos(0.5*t + pi/5);
% qdd_ref_2 = -0.5*0.5*sin(0.5*t + pi/5);
% q_ref_3 = sin(0.5*t);
% qd_ref_3= 0.5*cos(0.5*t);
% qdd_ref_3 = -0.5*0.5*sin(0.5*t);

% Trajectory 2 check

% q_ref_1 = 1 + 0.6*cos(pi*t/5);
% qd_ref_1 = -0.6*pi/5*sin(pi*t/5);
% qdd_ref_1 = -0.6*pi/5*pi/5*cos(pi*t/5);
% q_ref_2 = 1 + 0.6*sin(pi*t/5);
% qd_ref_2 = 0.6*pi/5*cos(pi*t/5);
% qdd_ref_2 = 0.6*pi/5*pi/5*sin(pi*t/5);
% q_ref_3 = sin(0.5*t + pi/5);
% qd_ref_3 = 0.5*cos(0.5*t + pi/5);
% qdd_ref_3 = -0.5*0.5*sin(0.5*t + pi/5);

%% System dynamic
% cos and sine
% Mass-Inertia matrix
c2 = cos(q_t(2,k-1));
c3 = cos(q_t(3,k-1));
c23 = cos(q_t(2,k-1)+q_t(3,k-1));
s2 = sin(q_t(2,k-1));
s3 = sin(q_t(3,k-1));
s23 = sin(q_t(2,k-1)+q_t(3,k-1));
% M-components
m11 = 1.0425+0.08094*c23+0.3484*c2+.0561*c3;
m12 = 0.4398+.04047*c23+.1742*c2+.0561*c3;
m13 = .1788+.04047*c23+.02809*c3;
m22 = .4398+.0519*c3;
m23 = .1788+.02809*c3;
m33 = .1788;
M = [m11 m12 m13;
     m12 m22 m23;
     m13 m23 m33];
% N-components
n11 = -.1742*s2*qd(2)^2-0.04047*s23*qd(2)^2-.02809*s3*qd(3)^2-.04047*s23*qd(3)^2-0.3484*s2*qd(1)*qd(2)-.08094*s23*qd(1)*qd(2);
n12 = -0.05619*s3*qd(1)*qd(3)-.08094*s23*qd(1)*qd(3)-.05619*s3*qd(2)*qd(3)-.08094*s23*qd(2)*qd(3);
n2 = .1742*s2*qd(1)^2+.04047*s23*qd(1)^2-.02809*s3*qd(3)^2-0.0561*s3*qd(1)*qd(3)-0.05619*s3*qd(2)*qd(3);
n3 = 0.02809*s3*qd(1)^2+.02809*s3*qd(2)^2+.04047*s23*qd(1)^2+.05619*s3*qd(1)*qd(2);
% N matrix
N = [n11+n12;
     n2;
     n3];
% Friction coefficients
F = [2.6e-4 0 0;
     0 2.6e-4 0; 
     0 0 2.6e-4];
    

%% TD with PD controller 
% Definition of variables
    D_bar = diag([0.05 0.05 0.05]);
    kd = diag([4 3 2]);
    kp = diag([2 3 6]);
    

    % Design of PD controller with time delay estimation 
    % the delail theory and the design is refered form the literature *PD
    % with time delay esitmation for robot manipulator

    e =  q - [q_ref_1;q_ref_2;q_ref_3];
    ed =  qd - [qd_ref_1;qd_ref_2;qd_ref_3];
    
    qdd_ref = [qdd_ref_1;qdd_ref_2;qdd_ref_3];
    u = qdd_ref - kd*ed - kp*e;
    T_t(:,k) = D_bar*u + T_t(:,k-1) - D_bar*qdd_t(:,k-1);
   
  %% robot system dynamics and updated value 
 
    qdd=M\(T_t(:,k)-N-F*qd);
    qd = qd + qdd*Ts;
    q = q + qd*Ts;

    q_t(:,k) = q;
    qd_t(:,k) = qd;
    qdd_t(:,k) = qdd;
 
    
    k = k + 1
end


%% references analysis and error plots

figure
%plot(thita1, thita2, thita3)
plot3(q_t(1,:),q_t(2,:),q_t(3,:),'--');
time_serial = 0:Ts:End_time;
hold on
%plot(thita1ref, thita2ref,thita3ref)
plot3(0.5*sin(pi/5*(time_serial)),0.5*cos(pi/5*(time_serial)),sin(0.5*(time_serial)));
legend('Actual trajectory','Reference trajectory');
xlabel('x');
ylabel('y');
zlabel('z');
title('Tracking Performance of Robot manipulator')
set(gcf,'color','w');

%plot for 1 reference
% plot3(cos(0.5*(time_serial) + pi/3),sin(0.5*(time_serial) + pi/5),sin(0.5*(time_serial)));
% legend('Actual trajectory','Reference trajectory');
% xlabel('x');
% ylabel('y');
% zlabel('z');
% title('Tracking Performance of Robot manipulator')
% set(gcf,'color','w');


% %plot for 2 reference
% plot3((1 + 0.6*cos(pi*(time_serial)/5)),(1 + 0.6*sin(pi*(time_serial)/5)),sin(0.5*(time_serial) + pi/5));
% legend('Actual trajectory','Reference trajectory');
% xlabel('x');
% ylabel('y');
% zlabel('z');
% title('Tracking Performance of Robot manipulator')
% set(gcf,'color','w');

%%
figure
subplot(3,1,1)
%plot thita1 error over time horizon
plot( 0:Ts:End_time , q_t(1,:) - 0.5*sin(pi/5*(time_serial)) );

%  plot( 0:Ts:End_time , q_t(1,:) - cos(0.5*(time_serial) + pi/3));
% plot( 0:Ts:End_time , q_t(1,:) - (1 + 0.6*cos(pi*(time_serial)/5)));
% title('theta_1 error')
xlabel('time(s)')
ylabel('amplitude(Grad)')
legend('error e1');
title('Joint1')

subplot(3,1,2)
%plot thita2 error over time horizon
plot( 0:Ts:End_time , q_t(2,:) - 0.5*cos(pi/5*(time_serial)) );

% plot( 0:Ts:End_time , q_t(2,:) - sin(0.5*(time_serial) + pi/5));
% plot( 0:Ts:End_time , q_t(2,:) - (1 + 0.6*sin(pi*(time_serial)/5)));
% title('theta_2 error')
xlabel('time(s)')
ylabel('amplitude(Grad)')
legend('error e2');
title('Joint2')

subplot(3,1,3)
%plot thita3 error over time horizon
plot( 0:Ts:End_time , q_t(3,:) - sin(0.5*(time_serial)));

% plot( 0:Ts:End_time , q_t(3,:) - sin(0.5*(time_serial)));
% plot( 0:Ts:End_time , q_t(3,:) - sin(0.5*(time_serial) + pi/5));
% title('theta_3 error')
xlabel('time(s)')
ylabel('amplitude(Grad)')
legend('error e3');
title('Joint3')

set(gcf,'color','w');
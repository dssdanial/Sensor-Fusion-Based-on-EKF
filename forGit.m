clc
clear 
close all

dt=0.01; %sample time

%% Load Trajectory and Sensor Input Data

% position
Pos_ref=readmatrix('Ref_Pos.xlsx');
Xref=Pos_ref(:,2); % meter
Yref=Pos_ref(:,3);

% velocity
Vel_ref=readmatrix('Ref_vel.xlsx');
Vel_Xref=Vel_ref(:,2); % m/s
Vel_Yref=Vel_ref(:,3);

% Theta
Theta_ref=readmatrix('Ref_yaw.xlsx');
Theta_ref=Theta_ref(:,2).*pi/180;   %convert to degree

% ************* Sensor Inputs**************
% IMU
Acc_ref=readmatrix('Ref_Accel.xlsx');
Acc_x=Acc_ref(:,2).*9.80665 ; % based on m/s^2
Acc_y=Acc_ref(:,3).*9.80665 ; % based on m/s^2
Acc_z=Acc_ref(:,4).*9.80665 ; % based on m/s^2
Gyr_ref=readmatrix('Ref_Gyro.xlsx');
Gyr_x=Gyr_ref(:,2).*(pi/180);  % Based on Rad/s
Gyr_y=Gyr_ref(:,3).*(pi/180); % Based on Rad/s
Gyr_z=Gyr_ref(:,4).*(pi/180); % Based on Rad/s

% Magnetometer
Mag_ref=readmatrix('Ref_Mag.xlsx');
Mag_x=Mag_ref(:,2); % ?????????????
Mag_y=Mag_ref(:,3);
Mag_z=Mag_ref(:,4);

% Encoder
Enc_ref=readmatrix('Ref_Gyro.xlsx');
Wenc=Enc_ref(:,4);
Enc_Vl=Acc_x +Acc_x*dt;  % integration of Ax-->  X linear velocity 
Enc_Vr=Acc_y +Acc_y*dt;  % integration of Ay -->  Y linear velocity 



% body to inertial 
phi=0;
theta=0;
psi=Theta_ref(1);

Rx =  [   1          0            0
    0     cos(phi)    -sin(phi)                         % Page _ 36_  kargah
    0     sin(phi)    cos(phi)];

Ry = [ cos(theta)      0      sin(theta)
    0               1           0
    -sin(theta)      0      cos(theta)];


Rz   =   [ cos(psi)     -sin(psi)      0
    sin(psi)      cos(psi)      0
    0                0            1];


C_b2i =Rz*Ry*Rx; %??????????????????????????????

%% System requirements


T_final=length(Theta_ref);
% X=zeros(17,1);
L=0.5; %wheels distance
% dVxa=0; % initial Xaccel error model
% dVya=0;  % initial Yaccel error model
% dTheta_c=0; % initial Azimuth angle error
dSlw=0;
dSrw=0;
dS_Dw=0;
dSax=0;
dBax=0;
dSay=0;
dBay=0;
dSgz=0;
dBgz=0;
dB_c=0;
dVxa=0;
dVya=0;



Theta_correct=Theta_ref(1,1);   % initial Theta
X_correct=Xref(1);   % initial X
Y_correct=Yref(1);   % initial Y

% initial errors
Error_x=0;
Error_y=0;
Error_theta=0;

% initial saving data

Xsave=zeros(T_final,1);
Ysave=zeros(T_final,1);
Theta_save=zeros(T_final,1);
X_Err=zeros(T_final,1);
Y_Err=zeros(T_final,1);
Ang_Err=zeros(T_final,1);



%% KF initialization
G=eye(17,10); % Noise distribution matrix
Q_k=eye(10); % Process noise covarinace Matrix
R_k=eye(3); % Measurment noise covarinace Matrix
P= eye(17); % Covariance matrix

W1=0.0*randn(10,1); % process noise
W2=0.0*randn(3,1); % Measurement noise



%% Main Loop

for i=1:T_final



% Matrix A1 initialization
A1=zeros(17,4);
A1(3,3)=1;
A1(4,4)=1;

% Matrix A2 initialization
A2=zeros(17,5);
A2(1,1)=0.5*(  -(Enc_Vl(i)+Enc_Vr(i))*dt *sin(Wenc(i)*dt)  );
A2(1,4)=0.5*(  Enc_Vl(i)*dt *cos(Wenc(i)*dt)  );
A2(1,5)=0.5*(  Enc_Vr(i)*dt *cos(Wenc(i)*dt)  );
A2(2,1)=0.5*(  (Enc_Vl(i)+Enc_Vr(i))*dt *cos(Wenc(i)*dt)  );
A2(2,4)=0.5*(  Enc_Vl(i)*dt *sin(Wenc(i)*dt)  );
A2(2,5)=0.5*(  Enc_Vr(i)*dt *sin(Wenc(i)*dt)  );
A2(5,4)=-Enc_Vl(i)/L;
A2(5,5)=Enc_Vr(i)/L;
A2(6,3)=1;
A2(8,4)=1;
A2(9,5)=1;

% Matrix A3 initialization
A3=zeros(17,8);
A3(5,1)=(Enc_Vl(i)-Enc_Vr(i))/2;
A3(10,1)=1;
A3(3,2)=Acc_x(i)*dt;
A3(11,2)=1;
A3(3,3)=1;
A3(12,3)=1;
A3(4,4)=Acc_y(i)*dt;
A3(13,4)=1;
A3(4,5)=1;
A3(14,5)=1;
A3(6,6)=Gyr_z(i)*dt;
A3(15,6)=1;
A3(6,7)=1;
A3(16,7)=1;
A3(7,8)=1;
A3(17,8)=1;
%

% Creat the total of Matrix A (Discreate form)
A=[A1,A2,A3]; %17*17


%% Sensor Error Modeling

dWg=dSgz*Gyr_z(i) +dBgz;  % Gyro error model
dVxa=dVxa +dSax*Acc_x(i)*dt +dBax; % X-accel error model
dVya=dVya +dSay*Acc_y(i)*dt +dBay; % Y-accel error model

dWe=(Enc_Vl(i)+Enc_Vr(i))/L *dS_Dw -(dSlw*Enc_Vl(i) + dSrw*Enc_Vr(i))/L;  % Encoder angle error
dVxe=0.5*(dSlw*Enc_Vl(i) + dSrw*Enc_Vr(i))*cos(dWe*dt)- 0.5*(Enc_Vl(i)+Enc_Vr(i))*dWe*dt*sin(dWe*dt); % X-Encoder model error
dVye=0.5*(Enc_Vl(i)+Enc_Vr(i))*cos(dWe*dt) + 0.5*(dSlw*Enc_Vl(i) + dSrw*Enc_Vr(i))*dWe*dt*sin(dWe*dt) ; % Y-Encoder model error

dTheta_c=-Theta_ref(i)+ dB_c; % Compass anlge error model


%% Kalman Filter
% States
% X=[dVxe;dVye;dVxa;dVya; dWe; dWg; dTheta_c; dSlw; dSrw; dS_Dw; dSax; dBax; dSay; dBay; dSgz; dBgz; dBc]; %17*1
X=[dVxe;dVye;dVxa;dVya; dWe; dWg; dTheta_c; dSlw; dSrw; dS_Dw; dSax; dBax; dSay; dBay; dSgz; dBgz; dB_c]; %17*1


% Time Update
xhat=A*X +G*W1;
P_= A * P * A' +G*Q_k*G'; % Covariance

% Measurement Update
Z=C*X + W2 ; % Measurment Equation
Res=Z - C * xhat; % residual
K=P_ * C' /(C * P_ * C' + R_k);
xhat = xhat + K * Res; 
P = (eye(17) - K * C) * P_ * (eye(17) - K *C)' + K * R_k * K';   % jozef form
norm(P)
%% Sys Update
% dVxe=xhat(1);
% dVye=xhat(2);
dVxa=xhat(3);
dVya=xhat(4);
% dWe=xhat(5);
% dWg=xhat(6);
% dTheta_c=xhat(7);
dSlw=xhat(8);
dSrw=xhat(9);
dS_Dw=xhat(10);
dSax=xhat(11);
dBax=xhat(12);
dSay=xhat(13);
dBay=xhat(14);
dSgz=xhat(15);
dBgz=xhat(16);
dB_c=xhat(17);


%% Designing of Controller and Robot Dynamic

Vref_T=(Vel_Xref(i)+Vel_Yref(i))/2;

[Vt,W] = Controller(Gyr_z(i),Vref_T,Error_x, Error_y, Error_theta, Theta_correct); % Controller design

% robot
[Dx,Dy,Dtheta] = Robot_Dynamic(Vt,W,Theta_correct); % Robot Dynamic model

%% Correction
Vx_correct=Dx  - dVxa;
Vy_correct=Dy  - dVya;
DTheta_correct=Dtheta - dTheta_c;

X_correct=X_correct +Vx_correct*dt;
Y_correct=Y_correct +Vy_correct*dt;
Theta_correct=Theta_correct+ DTheta_correct*dt;

Error_x=Xref(i) -X_correct;
Error_y=Yref(i) -Y_correct;
Error_theta=Theta_ref(i) -Theta_correct;



%% Save data


Xsave(i)=X_correct;
Ysave(i)=Y_correct;
Theta_save(i)=Theta_correct;

X_Err(i)=Error_x;
Y_Err(i)=Error_y;
Ang_Err(i)=Error_theta;

%%






iteration=i
end % for loop

Time=1:T_final;

figure(1);
plot(Time,Xref,Time,Xsave,'--','linewidth',1.5)
xlabel('Time (s)')
ylabel('X (m)')
title('X trajectory');
legend('Ref','Robot');


figure(2);
plot(Time,Yref,Time,Ysave,'--','linewidth',1.5)
xlabel('Time (s)')
ylabel('Y (m)')
title('Y trajectory');
legend('Ref','Robot');

figure(3);
plot(Xref,Yref,'linewidth',1.5);
hold on;
plot(Xsave,Ysave,'--','linewidth',1.5)
xlabel('X (m)')
ylabel('Y (m)')
title('XY trajectory');
legend('Ref','Robot');


figure(4);
plot(Time,Theta_ref,Time,Theta_save,'--','linewidth',1.5)
xlabel('Time (s)')
ylabel('Theta (rad)')
title('Theta trajectory');
legend('Ref','Robot');













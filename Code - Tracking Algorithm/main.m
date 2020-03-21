% Source: https://github.com/Spratm-Asleaf/GSE-Tracking
% Author: WANG Shixiong (s.wang@u.nus.edu)
% Affiliate: Department of Industrial SystemsEngineering and Management, National University of Singapore, Singapore 117576

% For mathematical details, see "IMM.pdf"

clear all;
close all;
clc;

%% Load raw data
load('RawData.mat');

% data sampling period
Ts = 0.1;

% data size
N = length(Y_Mear(1,:));

% load the scenario data
z_true = X_Real;        % Real position of the leading vehicle, i.e., true value of "X^t" of Eq. (2) and (3)
z = Y_Mear;             % Noised measurements of the positions of the leading vehicle, i.e., "Y^t" of Eq. (2) and (3)

%% init tracking algorithm
x_pro_IMM = zeros(6,N);         % estimated "X^t" by multiple model tracking algorithm (IMM-KF)
P_IMM     = zeros(6,6,N);       % estimation error covariance matrix

%% Load Models
% Eq. (2) and (3) in the paper
% System Dynamics Matrix, i.e., "\Phi" in Eq. (2) and (3)
    % case 'CV'
        F1 = [
                1   Ts   0
                0   1   0
                0   0   0
            ];
        F1 = blkdiag(F1,F1);
    % case 'CA'
        F2 = [
                1   Ts   Ts^2/2
                0   1   Ts
                0   0   1
            ];
        F2 = blkdiag(F2,F2);
    % case 'Singer'
        a = 1/5;    % suppose the average time for turning the direction is 5 seconds
        F3 = [
                1   Ts   (a*Ts - 1 + exp(-a*Ts))/(a^2)
                0   1   (1 - exp(-a*Ts))/a
                0   0   exp(-a*Ts)
            ];
        F3 = blkdiag(F3,F3);
    % case 'CS'
        a = 1/5;    % suppose the average time for turning the direction is 5 seconds
        F4 = [
                1   Ts   (a*Ts - 1 + exp(-a*Ts))/(a^2)
                0   1   (1 - exp(-a*Ts))/a
                0   0   exp(-a*Ts)
            ];
        F4 = blkdiag(F4,F4);
% Noise Driven Matrix, i.e., "G" in Eq. (2) and (3)
    % case 'CV'
        G1 = F1(1:3,3);
        G1 = blkdiag(G1,G1);
    % case 'CA'
        G2 = F2(1:3,3);
        G2 = blkdiag(G2,G2);
    % case 'Singer'
        G3 = F3(1:3,3);
        G3 = blkdiag(G3,G3);
%     % case 'CS'
        G4 = F4(1:3,3);
        G4 = blkdiag(G4,G4);
% Measurement Matrix, i.e., "H" in Eq. (2) and (3)
% Note that we have argumented "\Phi", "G" and "H" for "M = CV" with zero
% values so that dimensions are aligned to six
    H = [
            1   0   0   0   0   0
            0   0   0   1   0   0
        ];
    
% By Table 1, we know R = 0.7cm^2
R = diag([0.7, 0.7])*(10^-2)^2; % Measurement noise covariance, i.e., Covirance of V in Eq. (2) and (3)
Q = diag([0.08, 0.005]);        % Processing noise covariance (determined by trial-and-error), i.e., Covirance of W in Eq. (2) and (3)

% Innovation defined in Kalman Filtering
r1_IMM = zeros(2,1);
r2_IMM = zeros(2,1);
r3_IMM = zeros(2,1);
r4_IMM = zeros(2,1);

% Innovation covariance defined in Kalman Filtering
S1_IMM = zeros(2,2);
S2_IMM = zeros(2,2);
S3_IMM = zeros(2,2);
S4_IMM = zeros(2,2);

% initial value of the estimated "X^t"
% note that along the time, estimated "X^t" converges to "X^t" in distribution
x0 = [0 0 0 0 0 0]' + eps;
x1_IMM=x0;  % initial value of the estimated "X^t" for CV
x2_IMM=x0;  % initial value of the estimated "X^t" for CA
x3_IMM=x0;  % initial value of the estimated "X^t" for Singer 
x4_IMM=x0;  % initial value of the estimated "X^t" for CS 
x_pro_IMM(:,1)=x0;  % initial value of the estimated "X^t" of IMM-KF

% initial value of the estimation error covariance
P0 = eye(6)*1e5;
P1_IMM=P0;
P2_IMM=P0;
P3_IMM=P0;  
P4_IMM=P0; 
P_IMM(:,:,1)=P0;  

% Eq. (4)
pij=[0.91,0.03,0.03,0.03;
     0.03,0.91,0.03,0.03;
     0.03,0.03,0.91,0.03;
     0.03,0.03,0.03,0.91
]; 

% model probabolity defined in IMM-KF
u_IMM=zeros(4,N); 
u_IMM(:,1)=[0.7,0.1,0.1,0.1]';

% "\bar{A}^t" in Eq. (3)
a_avrg = [0;0];

% Main loop  
for T=1:N-1   
    % 1 Model Probability Update
    c_j = pij'*u_IMM(:,T);      
    ui1 = (1/c_j(1))*pij(:,1).*u_IMM(:,T);
    ui2 = (1/c_j(2))*pij(:,2).*u_IMM(:,T); 
    ui3 = (1/c_j(3))*pij(:,3).*u_IMM(:,T);  
    ui4 = (1/c_j(4))*pij(:,4).*u_IMM(:,T); 

    x01 = x1_IMM*ui1(1)+x2_IMM*ui1(2)+x3_IMM*ui1(3)+x4_IMM*ui1(4);
    x02 = x1_IMM*ui2(1)+x2_IMM*ui2(2)+x3_IMM*ui2(3)+x4_IMM*ui2(4); 
    x03 = x1_IMM*ui3(1)+x2_IMM*ui3(2)+x3_IMM*ui3(3)+x4_IMM*ui3(4);
    x04 = x1_IMM*ui4(1)+x2_IMM*ui4(2)+x3_IMM*ui4(3)+x4_IMM*ui4(4);
    
    P01=(P1_IMM+(x1_IMM-x01)*(x1_IMM-x01)')*ui1(1)+... 
        (P2_IMM+(x2_IMM-x01)*(x2_IMM-x01)')*ui1(2)+...
        (P3_IMM+(x3_IMM-x01)*(x3_IMM-x01)')*ui1(3)+...
        (P4_IMM+(x4_IMM-x01)*(x4_IMM-x01)')*ui1(4);
 
    P02=(P1_IMM+(x1_IMM-x02)*(x1_IMM-x02)')*ui2(1)+...
        (P2_IMM+(x2_IMM-x02)*(x2_IMM-x02)')*ui2(2)+...
        (P3_IMM+(x3_IMM-x02)*(x3_IMM-x02)')*ui2(3)+...
        (P4_IMM+(x4_IMM-x02)*(x4_IMM-x02)')*ui2(4);
 
    P03=(P1_IMM+(x1_IMM-x03)*(x1_IMM-x03)')*ui3(1)+...
        (P2_IMM+(x2_IMM-x03)*(x2_IMM-x03)')*ui3(2)+... 
        (P3_IMM+(x3_IMM-x03)*(x3_IMM-x03)')*ui3(3)+...
        (P4_IMM+(x4_IMM-x03)*(x4_IMM-x03)')*ui3(4);
 
    P04=(P1_IMM+(x1_IMM-x04)*(x1_IMM-x04)')*ui4(1)+...
        (P2_IMM+(x2_IMM-x04)*(x2_IMM-x04)')*ui4(2)+... 
        (P3_IMM+(x3_IMM-x04)*(x3_IMM-x04)')*ui4(3)+...
        (P4_IMM+(x4_IMM-x04)*(x4_IMM-x04)')*ui4(4);
    
    % 2 Individual Kalman Filtering by CV, CA, Singer, CS
    [x1_IMM,P1_IMM,r1_IMM,S1_IMM,a_avrg] = Kalman(x01,P01,z(:,T+1),F1,G1,Q,H,R,false,G2,a_avrg,a,Ts);	% CV 
    [x2_IMM,P2_IMM,r2_IMM,S2_IMM,a_avrg] = Kalman(x02,P02,z(:,T+1),F2,G2,Q,H,R,false,G2,a_avrg,a,Ts);   % CA
    [x3_IMM,P3_IMM,r3_IMM,S3_IMM,a_avrg] = Kalman(x03,P03,z(:,T+1),F3,G3,Q,H,R,false,G2,a_avrg,a,Ts);   % Singer
    [x4_IMM,P4_IMM,r4_IMM,S4_IMM,a_avrg] = Kalman(x04,P04,z(:,T+1),F4,G4,Q,H,R,true,G2,a_avrg,a,Ts);    % CS
    
    % 3 Model prob update
    [u_IMM(:,T+1)] = Model_P_Update(r1_IMM,r2_IMM,r3_IMM,r4_IMM,S1_IMM,S2_IMM,S3_IMM,S4_IMM,c_j);
    
    % 4 Model integration    
    [x_pro_IMM(:,T+1),P_IMM(:,:,T+1)] = Model_Mix(x1_IMM,x2_IMM,x3_IMM,x4_IMM,P1_IMM,P2_IMM,P3_IMM,P4_IMM,u_IMM(:,T));
end 

%% plot
T=1:N; 
Time = (T-1)*Ts;

% position
figure;
plot(Time,z_true(1,:),'r',Time,x_pro_IMM(1,:),'b',Time,z(1,:),'g--','linewidth',2);
legend('Real Position in X (m)','Estimated Position in X (m)','Measure in X');
%axis([0 40.1 0 2000]);
xlabel('Time (s)','fontsize',14);
ylabel('Displacement (m)','fontsize',14);
set(gca,'fontsize',14);

figure;
plot(Time,z_true(1,:)-x_pro_IMM(1,:),'r',Time,X_Real(1,:)-Y_Mear(1,:),'b--','linewidth',2);
%axis([0 40.1 -500 500]);
xlabel('Time (s)','fontsize',14);
legend('Position Estimation Error in X (m)','Position Measurement Error in X (m)','fontsize',14);
set(gca,'fontsize',14);

figure;
plot(Time,z_true(3,:),'r',Time,x_pro_IMM(4,:),'b',Time,z(2,:),'g--','linewidth',2);
%axis([0 40.1 -300 300]);
legend('Real Position in Y (m)','Estimated Position in Y (m)','Measure in Y');
xlabel('Time (s)','fontsize',14);
ylabel('Displacement (m)','fontsize',14);
set(gca,'fontsize',14);

figure;
plot(Time,z_true(3,:)-x_pro_IMM(4,:),'r',Time,X_Real(3,:)-Y_Mear(2,:),'b--','linewidth',2);
%axis([0 40.1 -300 300]);
xlabel('Time (s)','fontsize',14);
legend('Position Estimation Error in Y (m)','Position Measurement Error in Y (m)','fontsize',14);
set(gca,'fontsize',14);

% velocity
figure;
plot(Time,z_true(2,T),'r',Time,x_pro_IMM(2,T),'b--','LineWidth',2);
%axis([0 40.1 -10 50]);
legend('Real Velocity in X (m/s)','Estimated Velocity in X (m/s)');
xlabel('Time (s)','fontsize',14);
ylabel('Velocity (m/s)','fontsize',14);
set(gca,'fontsize',14);

figure;
plot(Time,z_true(2,T)-x_pro_IMM(2,T),'r','LineWidth',2);
axis([0 20 -0.2 0.25]);
xlabel('Time (s)','fontsize',14);
ylabel('Velocity Estimation Error in X (m/s)','fontsize',14);
set(gca,'fontsize',14);

figure;
plot(Time,z_true(4,T),'r',Time,x_pro_IMM(5,T),'b--','LineWidth',2);
axis([0 20 4.85 5.2]);
legend('Real Velocity in Y (m/s)','Estimated Velocity in Y (m/s)');
xlabel('Time (s)','fontsize',14);
ylabel('Velocity (m/s)','fontsize',14);
set(gca,'fontsize',14);

figure;
plot(Time,z_true(4,T)-x_pro_IMM(5,T),'r','LineWidth',2);
axis([0 20 -0.15 0.15]);
xlabel('Time (s)','fontsize',14);
ylabel('Velocity Estimation Error in Y (m/s)','fontsize',14);
set(gca,'fontsize',14);

% model prob
figure 
plot(Time,u_IMM(1,T),'m',Time,u_IMM(2,T),'g-.',Time,u_IMM(3,T),'b--',Time,u_IMM(4,T),'r-..','LineWidth',2);
grid on;
xlabel('Time (s)','fontsize',14);
ylabel('Model Probability','fontsize',14);
legend('CV','CA','Singer','CS');
set(gca,'fontsize',14);

%% Calculate errors
% Measurement Error
error_Measure_X_Axis = X_Real(1,:)-Y_Mear(1,:);
3*sqrt(var(error_Measure_X_Axis*100))    % unit: cm
error_Measure_Y_Axis = X_Real(3,:)-Y_Mear(2,:);
3*sqrt(var(error_Measure_Y_Axis*100))    % unit: cm

% Estimation Error by IMM-KF
error_Tracking_X_Axis = X_Real(1,:)-x_pro_IMM(1,:);
3*sqrt(var(error_Tracking_X_Axis*100))    % unit: cm
error_Tracking_Y_Axis = X_Real(3,:)-x_pro_IMM(4,:);
3*sqrt(var(error_Tracking_Y_Axis*100))    % unit: cm





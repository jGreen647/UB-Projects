clear all; close all;

m_c = 0.980; %kg
I_c = 0.0092336;
l = 0.06;
m_w = 0.15;
I_w = 0.000115;
R = 0.04;

fs = 50; %Hz
T = 1/fs;

alpha = I_c+m_c*R*l;
beta = m_c*9.8*l;
gamma = I_w+(m_w+m_c)*R^2;

%Continuous model
Ac = [0 1;beta/alpha 0];
Bc = [0;-gamma/alpha];
Cc = [0 1];
Dc = 0;

sysc = ss(Ac,Bc,Cc,Dc);

%Discrete Model
sysd = c2d(sysc,T);
Ad = expm(Ac*T);
Bd = Ac\(Ad - eye(2))*Bc;
Cd = Cc;
Dd = Dc;

%adding third state;
Aaug = [Ad [0;0];0 0 1];
Baug = [Bd;T];
Caug = [Cd 0;0 0 1];
Daug = Dd;

% Picking Desired roots
wn_cont = 8;
zeta_cont = 1;
wd_cont = wn_cont*sqrt(1-zeta_cont^2);

%Continuous Roots
s1 = -zeta_cont*wn_cont+wd_cont*1i;
s2 = -zeta_cont*wn_cont-wd_cont*1i;

%Discrete Roots
z1_cont = exp(s1*T);
z2_cont = exp(s2*T);
z3_cont = abs(z1_cont); %for same settling time as other roots

Kaug = acker(Aaug,Baug,[z3_cont z2_cont z1_cont]);
[P,V] = eig(Aaug-Baug*Kaug); %check - Yes, eigenvalues are the roots.

%Estimator
zeta_est = 0.5;
wn_est = 18;
wd_est = wn_est*sqrt(1-zeta_est^2);

s1_est = -zeta_est*wn_est+wd_est*1i;
s2_est = -zeta_est*wn_est-wd_est*1i;

z1_est = exp(s1_est*T);
z2_est = exp(s2_est*T);
z3_est = abs(z1_est);

Laug = place(Aaug',Caug',[z1_est z2_est z3_est])';
%[P,V] = eig(Aaug-Laug*Caug); %check - Yep eigenvalues are the roots.

%Simulation
t = 0:T:2;

x_act = zeros(3,length(t)); %Actual States --Pitch Angle;Pitch Angular Velocity; Wheel Angular Velocity--
x_act(:,1) = [0;0.2;0];
x_est = zeros(3,length(t)); %Estimated states
x_est(:,1) = [0;0;0];
u = zeros(1,length(t)); %Input --Angular acceleration of the wheels--
u(:,1) = -Kaug*x_est(:,1);
y = zeros(2,length(t)); %Output
y(:,1) = Caug*x_act(:,1);

for i = 2:length(t)
    
    x_act(:,i) = Aaug*x_act(:,i-1)+Baug*u(:,i-1);
    x_est(:,i) = (Aaug-Baug*Kaug-Laug*Caug)*x_est(:,i-1)+Laug*y(:,i-1);
    y(:,i) = Caug*x_act(:,i);
    u(:,i) = -Kaug*x_est(:,i);
    
end

figure
plot(t,x_act(1,:));hold on;grid on;title('Pitch angle');
plot(t,x_est(1,:));legend('Actual','Estimated');
figure
plot(t,x_act(2,:));hold on;grid on;title('Pitch Angular Velocity');
plot(t,x_est(2,:));legend('Actual','Estimated');
figure
plot(t,x_act(3,:));hold on;grid on;title('Wheel Angular Velocity');
plot(t,x_est(3,:));legend('Actual','Estimated');
figure
plot(t,y);title('Output');grid on;legend('Pitch Angular Velocity','Wheel Angular Velocity');
xlabel('Time (s)');ylabel('Amplitude');
figure
plot(t,x_act-x_est);grid on; title('Error');legend('Pitch Angle','Pitch Angular Velocity','Wheel Angular Velocity');
xlabel('Time (s)');ylabel('Amplitude');
figure
plot(t,x_est);grid on;title('Estimated States');legend('Pitch Angle','Pitch Angular Velocity','Wheel Angular Velocity');
xlabel('Time (s)');ylabel('Amplitude');


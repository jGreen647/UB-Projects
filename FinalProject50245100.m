%Final Project

%given vehicle parameters
m=1572;
load=m*9.81;
Iz=1800;
Ix=450;
l=2.81;
% WF=0.523; 
WF=0.5; %Modified Parameter
lr=WF*l; lf=l-lr;
load_f=WF*load;
load_r=load-load_f;
% tf=1.58;
tf=1.88; %Modified Parameter
% tr=1.6;
tr=1.88; %Modfied Parameter
% h_cg=0.4572;
h_cg=0.3; %Modified Parameter
% h_rf=0.076;
h_rf=0.3; %Modified Parameter
% h_rr=0.189;
h_rr=0.3; %Modified Parameter
% KF=0.53;
KF=0.5; %Modified Parameter
k_phi=97193; k_phif=KF*k_phi; k_phir=k_phi-k_phif;
h_cgr=h_cg-(h_rf+((h_rr-h_rf)/(k_phif+k_phir))*lf);

%steer angle parameters
t=transpose(0:.001:4);
dt=t(2)-t(1);

A=(3*pi)/180;
om=2*pi;

delta1=A.*sin(om.*t)-A*sin(om.*t).*heaviside(t-(2*pi)/om);
delta2=(A.*sin(om.*(t-(4*pi)/om)).*heaviside(t-6*pi/om)-A.*sin(om.*(t-4*pi/om))).*heaviside(t-4*pi/om);
delta=delta1+delta2;

%initial conditions
ay=zeros(length(t),1);
ax=zeros(length(t),1);

V_in=17.8816; %17.8816
Vx=V_in.*ones(length(t),1);
Vy=zeros(length(t),1);
Vx_I=zeros(length(t),1);
Vy_I=zeros(length(t),1);
dVx=zeros(length(t),1); 
dVy=zeros(length(t),1);
Vwx11=V_in.*ones(length(t),1);
Vwx12=V_in.*ones(length(t),1);
Vwx21=V_in.*ones(length(t),1);
Vwx22=V_in.*ones(length(t),1);

x=zeros(length(t),1);
y=zeros(length(t),1);
x_I=zeros(length(t),1);
y_I=zeros(length(t),1);

dpsi=zeros(length(t),1);
psi=zeros(length(t),1);
heading=zeros(length(t),1);

alpha11=zeros(length(t),1);
alpha12=zeros(length(t),1);
alpha21=zeros(length(t),1);
alpha22=zeros(length(t),1);

beta=zeros(length(t),1);

roll=zeros(length(t),1);

Fy11=zeros(length(t),1);
Fy12=zeros(length(t),1);
Fy21=zeros(length(t),1);
Fy22=zeros(length(t),1);

V_slip=zeros(length(t),1);

Fz11=load_f/2.*ones(length(t),1);
Fz12=load_f/2.*ones(length(t),1);
Fz21=load_r/2.*ones(length(t),1);
Fz22=load_r/2.*ones(length(t),1);

dFz11=zeros(length(t),1);
dFz12=zeros(length(t),1);
dFz21=zeros(length(t),1);
dFz22=zeros(length(t),1);


for i=1:length(t)-1
    
    %Get lateral forces
    Fy11(i)=nonlintire(alpha11(i),Fz11(i),Vwx11(i));
    Fy12(i)=nonlintire(alpha12(i),Fz12(i),Vwx12(i));
    Fy21(i)=nonlintire(alpha21(i),Fz21(i),Vwx21(i));
    Fy22(i)=nonlintire(alpha22(i),Fz22(i),Vwx22(i));
    
    %Yaw rate // Yaw rate rate
    dpsi(i)=(1/Iz)*(lf*(Fy11(i)*cos(delta(i))+Fy12(i)*cos(delta(i)))-lr*(Fy21(i)+Fy22(i))+(tf/2)*(Fy11(i)*abs(sin(delta(i)))-Fy12(i)*abs(sin(delta(i)))));
    psi(i+1)=psi(i)+dpsi(i)*dt;
    heading(i+1)=heading(i)+psi(i)*dt;
    
    %Accelerations // Velocities (1.38&1.39)
    ay(i)=(1/m)*(Fy11(i)*cos(delta(i))+Fy12(i)*cos(delta(i))+Fy21(i)+Fy22(i));
    ax(i)=(-1/m)*(Fy11(i)*sin(delta(i))+Fy12(i)*sin(delta(i)));
    
    %acceleration equations given from textbook excerpt&integration
    dVx(i)=Vy(i)*psi(i)+ax(i);
    dVy(i)=-Vx(i)*psi(i)+ay(i);
    Vx(i+1)=Vx(i)+dVx(i)*dt;
    Vy(i+1)=Vy(i)+dVy(i)*dt;
    Vx_I(i)=Vx(i)*cos(heading(i))-Vy(i)*sin(heading(i));
    Vy_I(i)=Vx(i)*sin(heading(i))+Vy(i)*cos(heading(i));
    
    x(i+1)=x(i)+Vx(i)*dt;
    y(i+1)=y(i)+Vy(i)*dt;
    
    x_I(i+1)=x_I(i)+Vx_I(i)*dt;
    y_I(i+1)=y_I(i)+Vy_I(i)*dt;
    
    Vwx11(i+1)=(Vx(i)-(psi(i)*tf/2))*cos(delta(i))+(Vy(i)+psi(i)*lf)*sin(delta(i));
    Vwx12(i+1)=(Vx(i)+(psi(i)*tf/2))*cos(delta(i))+(Vy(i)+psi(i)*lf)*sin(delta(i));
    Vwx21(i+1)=Vx(i)-(psi(i)*tr/2);
    Vwx22(i+1)=Vx(i)-(psi(i)*tr/2);
    
    %slip angles **maybe i+1** **negative sign per Dr. Estes**
    alpha11(i+1)=-(delta(i)-atan((Vy(i)+lf*psi(i))/(Vx(i)-tf*psi(i)/2)));
    alpha12(i+1)=-(delta(i)-atan((Vy(i)+lf*psi(i))/(Vx(i)+tf*psi(i)/2)));
    alpha21(i+1)=-(-atan((Vy(i)-lr*psi(i))/(Vx(i)-tr*psi(i)/2)));
    alpha22(i+1)=-(-atan((Vy(i)-lr*psi(i))/(Vx(i)+tr*psi(i)/2)));
    
    %sideslip angle
    beta(i)=atan(Vy(i)/Vx(i));
    
    %roll angle
    roll(i)=-(m*ay(i)*h_cgr)/(k_phif+k_phir-m*9.81*h_cgr);
    
    %load transfer
    dFz11(i)=-(1/tf)*((lr/l)*m*ay(i)*h_rf+k_phif*roll(i));
    dFz12(i)=-dFz11(i);
    dFz21(i)=-(1/tr)*((lr/l)*m*ay(i)*h_rr+k_phif*roll(i));
    dFz22(i)=-dFz21(i);
    
    Fz11(i+1)=load_f/2+dFz11(i);
    Fz12(i+1)=load_f/2+dFz12(i);
    Fz21(i+1)=load_r/2+dFz21(i);
    Fz22(i+1)=load_r/2+dFz22(i);
    
    
end

%(i) inertial x and y position
figure
plot(x_I,y_I); xlabel('x position (m)'); ylabel('y position (m)'); grid on

% %(ii) sideslip angle vs time
% figure
% plot(t,beta); xlabel('time (s)'); ylabel('sideslip angle (rad)'); grid on
% 
% %(iii) longitudinal velocity vs time
% figure
% plot(t,Vx); xlabel('time (s)'); ylabel('Vx (m/s)'); grid on
% 
% %(iv)lateral acceleration vs time
% figure
% plot(t,ay*0.101972); xlabel('time (s)'); ylabel('ay (gs)'); grid on
% 
% %(v) all slip angle
% figure
% plot(t,alpha11*(360/(2*pi))); xlabel('time (s)'); ylabel('slip angle 11 (deg)'); grid on
% figure
% plot(t,alpha12*(360/(2*pi))); xlabel('time (s)'); ylabel('slip angle 12 (deg)'); grid on
% figure
% plot(t,alpha21*(360/(2*pi))); xlabel('time (s)'); ylabel('slip angle 21 (deg)'); grid on
% figure
% plot(t,alpha22*(360/(2*pi))); xlabel('time (s)'); ylabel('slip angle 22 (deg)'); grid on
% 
% %(vi) all tire loads vs time
% figure
% plot(t,Fz11); xlabel('time (s)'); ylabel('Normal Load 11 (N)'); grid on
% figure
% plot(t,Fz12); xlabel('time (s)'); ylabel('Normal Load 12 (N)'); grid on
% figure
% plot(t,Fz21); xlabel('time (s)'); ylabel('Normal Load 21 (N)'); grid on
% figure
% plot(t,Fz22); xlabel('time (s)'); ylabel('Normal Load 22 (N)'); grid on
% 
%(vii) roll angle vs time
figure
plot(t,roll*(360/(2*pi))); xlabel('time (s)'); ylabel('roll angle (deg)'); grid on

%To optimize the performance of the vehicle during this maneuver we want to
%adjust parameters to make the vehicle more neutral steer, and to reduce load transfer. To do this
%(assuing identical tires all around) set the front weight and roll
%stiffness distributions to 50%.  The front and rear trackwidths should be
%as wide as possible ( due to the 1/t factor in the front and rear load
%transfer equations) but identical to maintain neutral steer during load
%transfer.  Given adjustment limitations, set tr and tf to 1.88m.  Since
%the height of the CG above the roll axis is a factor in the roll angle,
%and in turn the load transfer equations we want to make them at the same
%height, but as low as possible since the roll axis height is also a factor
%in the load transfer equation.  Since the lowest the CG can be is 0.3m we
%make this the CG height, and the roll center height in the front and rear.
%This way, the roll angle is zero, and the factor of roll center height is
%minimized.  With these modifications you will get a neutral steer car
%which will maximize the lateral force achievable before the tires give
%out.  Additionally the load transfer is minimized which also maximizes
%lateral forces achievable by the tires for this manuever, since the normal
%load on the tires for any given turn is minimized.

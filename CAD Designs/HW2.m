% %% Problem 3
% 
% T = 0.2;
% zeta = 0.298;
% omega_n = 3;
% omega_d = omega_n*sqrt(1-zeta^2);
% 
% roots = [exp(-zeta*omega_n*T)*(cos(omega_d*T)+sin(omega_d*T)*1i);exp(-zeta*omega_n*T)*(cos(omega_d*T)-sin(omega_d*T)*1i)];
% charEqn = poly(roots);
% 
% G = tf(0.2899,charEqn,.2);
% step(G);

%%Problem 4
k = -0.0068;
G = tf([k -10*k],[1 k-1.5 0.5-10*k],0.01);
step(G);

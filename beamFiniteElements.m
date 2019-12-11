clear all; close all;

%FEA Parameters
num_elem = 6;
num_nodes = num_elem+1;
ndof = 2 * num_nodes;

%Beam Parameters
tipMass = 0.01536; %kg
E = 6.9e10; %Pa
l = 0.160; %meters
elemLeng = l/6;
width = 0.009525; %meters
thick = 0.00238; %meters
rho = 2700; %kg/m^3
I = (1/12)*width*thick^3;
A = thick*width;

springStiffness = (E*I)/(elemLeng^3);
massFactor = (rho*A*elemLeng)/420;

%Local Mass and Stiffness Elements
k_elem = [12 6*elemLeng -12 6*elemLeng;...
          6*elemLeng 4*elemLeng^2 -6*elemLeng 2*elemLeng^2;...
         -12 -6*elemLeng 12 -6*elemLeng;...
          6*elemLeng 2*elemLeng^2 -6*elemLeng 4*elemLeng^2];
      
m_elem = [156 22*elemLeng 54 -13*elemLeng;...
          22*elemLeng 4*elemLeng^2 13*elemLeng -3*elemLeng^2;...
          54 13*elemLeng 156 -22*elemLeng;...
         -13*elemLeng -3*elemLeng^2 -22*elemLeng 4*elemLeng^2];

%Global Mass and Stiffness Matrix Assembly
K = zeros(ndof,ndof);
M = zeros(ndof,ndof);

for i = 1:num_elem
    
   n = 2*i-1;
   m = n+3;
   M(n:m,n:m) = M(n:m,n:m) + m_elem;
   K(n:m,n:m) = K(n:m,n:m) + k_elem;
   
end


K = K(3:14,3:14)*springStiffness;
M = M(3:14,3:14)*massFactor;

M(end-1,end-1) = M(end-1,end-1)+tipMass;

%Determine Natural Frequencies 
[P,V] = eig(K,M);

w = sqrt(V)/(2*pi);


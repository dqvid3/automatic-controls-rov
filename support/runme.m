clearvars,close all;clc

% Dati:
M = 580; % [kg]
J = 560; % [Nms^2]
B = 20; % [N]
b = 1.2; % [m]
k_z = 133; % [Ns/m]
k_r = 168; % [Nms]
k_th = 143; % [Nm]

% All'equilibrio valgono:
x_2 = pi/6; % [rad]
x_3 = 0; % [m/s]
u = -k_th*sin(x_2)/b; % [N]
F_L = (u*cos(x_2)-B)/sin(x_2); % [N]

% Forma di stato:
alfa = -k_z*cos(x_2)/M;
beta = -u*sin(x_2)/M + k_z*x_3*sin(x_2)/M - F_L*cos(x_2)/M;
gamma = -k_th*cos(x_2)/J;
epsilon = -k_r/J;
delta = epsilon^2+4*gamma;
omega = imag(sqrt(delta)/2);

A = [[0, 0, 1, 0]; [0, 0, 0, 1]; [0, beta, alfa, 0]; [0, gamma, 0, epsilon]]
B = [[0]; [0]; [cos(x_2)/M]; [-b/J]]
C = [1, -b*cos(x_2), 0, 0]
D = [0]

% Autovalori e matrice diagonale:
t=3;
X0 = [1;1;1;1];
[T, Lambda] = eig(A);
T_inv = inv(T);
evolLib=T*expm(Lambda*t)*T_inv*X0

mt=[1; expm(alfa*t);  exp((epsilon/2)*t)*cos(omega*t); exp((epsilon/2)*t)*sin(omega*t)];

M=[[X0(1) + ((-(beta*epsilon)/(alfa*gamma))*X0(2)) - ((1/alfa)*X0(3)) +  ((beta/(alfa*gamma))*X0(4)), (1/alfa)*(((beta*(epsilon-alfa)*X0(2)-beta*X0(4))/(-(alfa*alfa)+alfa*epsilon+gamma))+X0(3)), (((2*beta*(sqrt(delta)+epsilon))/(gamma*(sqrt(delta)-epsilon)*(2*alfa+sqrt(delta)-epsilon)))*((2*gamma*X0(2)-(sqrt(delta)-epsilon)*X0(4))/(2*sqrt(delta))))+(((2*beta*(sqrt(delta)-epsilon))/(gamma*(sqrt(delta)+epsilon)*(-2*alfa+sqrt(delta)+epsilon)))*((2*gamma*X0(2)+(sqrt(delta)+epsilon)*X0(4))/(2*sqrt(delta)))), (((2*beta*(sqrt(delta)+epsilon))/(gamma*(sqrt(delta)-epsilon)*(2*alfa+sqrt(delta)-epsilon)))*((2*gamma*X0(2)-(sqrt(delta)-epsilon)*X0(4))/(2*sqrt(delta))))*(-1i)+(((2*beta*(sqrt(delta)-epsilon))/(gamma*(sqrt(delta)+epsilon)*(-2*alfa+sqrt(delta)+epsilon)))*((2*gamma*X0(2)+(sqrt(delta)+epsilon)*X0(4))/(2*sqrt(delta))))*(1i)];
   [0, 0, (((sqrt(delta)+epsilon)/(2*gamma))*((2*gamma*X0(2)-(sqrt(delta)-epsilon)*X0(4))/(2*sqrt(delta))))+(((sqrt(delta)-epsilon)/(2*gamma))*((2*gamma*X0(2)+(sqrt(delta)+epsilon)*X0(4))/(2*sqrt(delta)))), (((sqrt(delta)+epsilon)/(2*gamma))*((2*gamma*X0(2)-(sqrt(delta)-epsilon)*X0(4))/(2*sqrt(delta))))*(-1i)+(((sqrt(delta)-epsilon)/(2*gamma))*((2*gamma*X0(2)+(sqrt(delta)+epsilon)*X0(4))/(2*sqrt(delta))))*(1i)];
   [0, (((beta*(epsilon-alfa)*X0(2)-beta*X0(4))/(-(alfa*alfa)+alfa*epsilon+gamma))+X0(3)), (((beta*(sqrt(delta)+epsilon))/(gamma*(2*alfa+sqrt(delta)-epsilon)))*(((sqrt(delta)-epsilon)*X0(4)-2*gamma*X0(2))/(2*sqrt(delta)))) + (((beta*(sqrt(delta)-epsilon))/(gamma*(-2*alfa+sqrt(delta)+epsilon)))*(((sqrt(delta)+epsilon)*X0(4)+2*gamma*X0(2))/(2*sqrt(delta)))), (((beta*(sqrt(delta)+epsilon))/(gamma*(2*alfa+sqrt(delta)-epsilon)))*(((sqrt(delta)-epsilon)*X0(4)-2*gamma*X0(2))/(2*sqrt(delta))))*(-1i) + (((beta*(sqrt(delta)-epsilon))/(gamma*(-2*alfa+sqrt(delta)+epsilon)))*(((sqrt(delta)+epsilon)*X0(4)+2*gamma*X0(2))/(2*sqrt(delta))))*(1i)];
   [0, 0, (((sqrt(delta)-epsilon)*X0(4)-2*gamma*X0(2))/(2*sqrt(delta)))+(((sqrt(delta)+epsilon)*X0(4)+2*gamma*X0(2))/(2*sqrt(delta))), (((sqrt(delta)-epsilon)*X0(4)-2*gamma*X0(2))/(2*sqrt(delta)))*(-1i)+(((sqrt(delta)+epsilon)*X0(4)+2*gamma*X0(2))/(2*sqrt(delta)))*(1i)]];

fattorizzazione = M*mt

% Funzione di trasferimento, poli e zeri:

sys = ss(A, B,C,D);
G=tf(sys)
BodeForm = zpk(G);
BodeForm.DisplayFormat='frequency'
zeros = zero(G)
poles = pole(G)

%controllore 

s=tf('s');
Ctrl=tf(-11.4*(1+s/0.274)/(1+s/0.902));
Ctrl = zpk(Ctrl)
Gc = feedback(Ctrl*G,1);
Gc = minreal(Gc);
Gc = zpk(Gc)
System = ss(G);
Controller = ss(Ctrl)
simulink
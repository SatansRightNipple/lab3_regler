%% 3.1

Lm = 2;
Rm = 21;
b = 1;
Kt = 38;
Km = 0.5;
n = 1/20;
s = tf("s");
[J,umax] = lab3robot(010930);

G0 = 1/(J*s+b) * Kt * 1/(s*Lm + Rm);

G = n/s * G0 * 1/(1+Km*G0);

lab3robot(G,010930)

%% 3.2.2

K = 6.75;
G_cl = K * G/(1+K*G);

[y, t] = step(G_cl);
plot(t, y);
title('Step Response of Closed-Loop System');
xlabel('Time');
ylabel('Response');
grid on;

%% 3.2.3

margin(K*G)
grid on;
w_b = bandwidth(K*G); 

%% 3.3
beta = 0.14;
t_d = 1/(1.172*sqrt(beta));
t_i = 18/1.172;
gamma = 0.043;
K_l = 6.75/(10^(-11/20)*1.2);

F_lead = K_l*(t_d*s + 1)/(beta*t_d*s + 1);
F_lag = (t_i*s + 1)/(t_i*s + gamma);
G_cl_l = F_lead* F_lag * G/(1+F_lead* F_lag*G);
r_to_u = F_lead* F_lag/(1+F_lead* F_lag*G);

t = 0:0.01:20;
u = ones(size(t));
u_ramp = t;

figure()
margin(F_lead*F_lag*G)
hold on

lab3robot(G,K,F_lead* F_lag,0,0,0,0,0,010930)

[y, t, x] = lsim( G*F_lead*F_lag / (1+F_lead*F_lag*G), u, t);
[u_control, t, x] = lsim(r_to_u, u, t);

umax_calc = max(u_control);
umin_calc = min(u_control);

umax_calc = max(umax_calc,abs(umin_calc))
disp(umax)

figure()
plot(t, y);
grid on

[y_ramp, t, x] = lsim( G*F_lead*F_lag / (1+F_lead*F_lag*G), u_ramp, t);
figure()
plot(t, abs(y_ramp-u_ramp'));

grid on;

%% 3.4

s1 = 1/(1+K*G);
s2 = 1/(1+F_lead* F_lag*G);

bodemag(s1,s2)
grid on

%% 3.5

T = 1-s2;

dG1 = (s+10)/40;
dG2 = (s+10)/(4*(s+0.01));

bodemag(T,dG1,dG2)
grid on

%% 3.6

A = [0, n, 0; 0, -b/J, Kt/J; 0, -Km/Lm, -Rm/Lm ];
B = [0;0;1/Lm];
C = [1,0,0];

S = [B, A*B, A^2*B];

O = [C; C*A; C*A^2];

%% 3.7
poles = [ -3, -3-2i ,-3+2i];

L = place(A,B,poles);

sys = ss(A-B*L, B, C, 0);

[y, t] = step(sys);

step(sys)

L2 = acker(A, B, poles);

info = stepinfo(y, t);

final_value = info.SettlingMax;

l0 = -n*L(1)*Kt/J;

gain = dcgain(sys);

lab3robot(G,K,F_lead* F_lag,A,B,C,L,1/gain,010930)





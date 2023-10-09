%% 3.1

Lm = 2;
Rm = 21;
b = 1;
Kt = 38;
Km = 0.5;
n = 1/20;
s = tf("s");
[J,umax] = lab3robot(010727);

G0 = 1/(J*s+b) * Kt * 1/(s*Lm + Rm);

G = n/s * G0 * 1/(1+Km*G0);

lab3robot(G,010727)

%% 3.2.2

K = 4.12;
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

t_d = 11.34;
t_i = 53.19;
gamma = 1;
beta = 0.22;
K_l = 6;


F = K_l*(t_d*s + 1)/(beta*t_d*s + 1)*(t_i*s + 1)/(t_i*s + gamma);
G_cl_l = F * G/(1+F*G);
r_to_u = F/(1+F*G);

t = 0:0.01:100;
u = ones(size(t));
u_ramp = t;

%bode(F*G)
hold on

[y, t, x] = lsim(G_cl_l, u, t);
[u_control, t, x] = lsim(G_cl_l, u, t);

umax_calc = max(u_control);
umin_calc = min(u_control);

umax_calc = max(umax_calc,abs(umin_calc))
disp(umax)

plot(t, y);

title('Step Response of Closed-Loop System');
xlabel('Time');
ylabel('Response');
grid on;




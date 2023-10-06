%% 3.1

Lm = 2;
Rm = 21;
b = 1;
Kt = 38;
Km = 0.5;
n = 1/20;

[J,umax] = lab3robot(010727);

s = tf("s");

G0 = 1/(J*s+b) * Kt * 1/(s*Lm + Rm);

G = n/s * G0 * 1/(1+Km*G0);

lab3robot(G,010727)

%% 3.2

K = 226;
G_cl = K * G/(1+K*G);

[y, t] = step(G_cl);
plot(t, y);
title('Step Response of Closed-Loop System');
xlabel('Time');
ylabel('Response');
grid on;

%% 3.3

bode(K*G)
grid on;
w_b = bandwidth(K*G);
[GM, PM, w_c] = margin(K*G);

%% 3.4



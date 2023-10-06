%% 3.1

Lm = 2;
Rm = 21;
b = 1;
Kt = 38;
Km = 0.5;
n = 1/20;

[J,umax] = lab3robot(010727);

numerator = Kt*n;         
denominator = [0, b*Rm*(1+Km), (J*Rm+b*Lm)*(1+Km),J*Lm*(1+Km)];

G = tf(numerator, denominator);

lab3robot(G,010727)

%% 3.2

bode(G)
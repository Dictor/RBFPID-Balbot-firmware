% e is error, PID input
% y is output of plant
% mg is momentum factor of RBF NN
% lr is learning rate of RBF NN
function [kpo, kio, kdo, uo, ymo, jo]= RBFNN_PID(e, y, mg, gain_mg, lr, init_kp, init_ki, init_kd, u_limit, init_rbf_width, init_rbf_weight)
persistent flag;

% PID controller parameters
persistent ek ek1 ek2; % e(k), e(k-1), e(k-2)
persistent sumek;
persistent xc1 xc2 xc3; % xc(1), xc(2), xc(3)
persistent u du;
persistent kp ki kd;

% RBF NN parameters
n = 3; % size of input
m = 8; % size of hidden
persistent X; % input X [n x 1]
persistent Ck Ck1 Ck2; % center vector C [m x n]
persistent H; % radial vector [m x 1]
persistent Bk Bk1 Bk2; % radial width vector B [m x 1]
persistent Wk Wk1 Wk2; % weight vector W [m x 1]
persistent yk yk1; % output of plant
persistent ym; % output of RBF NN
persistent J; % jacobian information of RBF NN

% initialize inital value just first time
if isempty(flag)
    flag = 1;

    ek1 = 0;
    ek2 = 0;
    sumek = 0;
    
    u = 0;
    du = 0;

    kp = init_kp;
    ki = init_ki;
    kd = init_kd;
    X = zeros(n, 1);
    Ck = zeros(m, n);
    Ck1 = zeros(m, n);
    Ck2 = zeros(m, n);
    H = zeros(m, 1);
    Bk = init_rbf_width * ones(m, 1) + rand(m, 1); %ones(m, 1);
    Bk1 = Bk;
    Bk2 = Bk;
    Wk = init_rbf_weight * ones(m, 1);
    Wk1 = Wk;
    Wk2 = Wk;

    yk1 = 0;
end

% set input vector
yk = y;
X(1) = u;
X(2) = yk;
X(3) = yk1;

% calculate centor vector
for j = 1:m
    dist = norm(X - Ck(j));
    H(j) = exp(-((dist^2) / (2 * Bk(j)^2)));
end

% calculate RBF NN parameters
ym = Wk' * H;
for j = 1:m
    dWk = mg * (yk - ym) * H(j);
    Wk(j) = Wk1(j) + dWk + lr * (Wk1(j) - Wk2(j));
    dBk = mg * (yk - ym) * Wk(j) * H(j) * ((norm(X - Ck(j))^2) / (Bk(j) ^ 3));
    Bk(j) = Bk1(j) + dBk + lr * (Bk1(j) - Bk2(j));
    for i = 1:n
        dCk = mg * (yk - ym) * Wk(j) * ((X(i) - Ck(j, i)) / (Bk(j)^2));
        Ck(j, i) = Ck1(j, i) + dCk + lr * (Ck1(j, i) - Ck2(j, i));
    end
end

% calculate RBF NN J
J = 0;
for j = 1:m
    J = J + Wk(j) * H(j) * ((Ck(j, 1) - X(1)) / Bk(j) ^ 2);
end
jo = J;


% ek is current error.
ek = e;
sumek = sumek + ek;

% calculate controller input
xc1 = ek - ek1;
xc2 = ek;
xc3 = ek - 2 * ek1 + ek2;

% update PID gain
kp = kp + gain_mg * mg * ek * J * xc1;
ki = ki + gain_mg * mg * ek * J * xc2;
kd = kd + gain_mg * ek * J * xc3;

% calculate pid output
du = kp * xc1 + ki * xc2 + kd * xc3;
u = u + du;
if abs(u) > u_limit
    u = du;
end

uo = u;

% return output
kpo = kp;
kio = ki;
kdo = kd;
ymo = ym;

% in final, proceed time k, shift
ek1 = ek;
ek2 = ek1;
yk1 = yk;
Ck1 = Ck;
Ck2 = Ck1;
Bk1 = Bk;
Bk2 = Bk1;
Wk1 = Wk;
Wk2 = Wk1;

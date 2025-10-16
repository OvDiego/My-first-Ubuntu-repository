clear; clc; close all;

%% PARÁMETROS 
dt = 0.1;           
v = 1;              
w = 0;              


x_est = [0; 0; 0];


Q = 0.01 * eye(3);
R = 0.1 * eye(6);   
P = eye(3);


landmarks = [1.5, -0.5;
             2.0, -0.1;
             0.5,  0.5];


z = [1.55; -0.31;
     1.85; -0.03;
     0.61;  0.91];

%% ===================== PREDICCIÓN =====================

x_pred = x_est + [dt * v * cos(x_est(3));
                  dt * v * sin(x_est(3));
                  dt * w];

F = [1, 0, -dt * v * sin(x_est(3));
     0, 1,  dt * v * cos(x_est(3));
     0, 0,  1];

P_pred = F * P * F' + Q;


fprintf('-PREDICCIÓN\n');
fprintf('Estado predicho [x, y, theta]:\n');
disp(x_pred');
fprintf('Covarianza predicha P_pred:\n');
disp(P_pred);

%% ===================== ACTUALIZACIÓN =====================
h = zeros(6,1);
H = zeros(6,3); 

for i = 1:3
    lx = landmarks(i,1);
    ly = landmarks(i,2);
    dx = lx - x_pred(1);
    dy = ly - x_pred(2);
    q = dx^2 + dy^2;

    r = sqrt(q);
    phi = atan2(dy, dx) - x_pred(3);
    h(2*i-1:2*i) = [r; phi];

    H_i = [ -dx/sqrt(q), -dy/sqrt(q), 0;
             dy/q,        -dx/q,      -1];
    H(2*i-1:2*i, :) = H_i;
end


y = z - h;

for k = 2:2:length(y)
    y(k) = atan2(sin(y(k)), cos(y(k)));
end


S = H * P_pred * H' + R;
K = P_pred * H' / S;

x_est = x_pred + K * y;
P = (eye(3) - K * H) * P_pred;

fprintf('-ACTUALIZACIÓN-\n');
fprintf('Estado actualizado [x, y, theta]:\n');
disp(x_est');
fprintf('Covarianza actualizada P:\n');
disp(P);

figure; hold on; grid on; axis equal;
plot(x_est(1), x_est(2), 'ro', 'MarkerSize',10, 'LineWidth',2);
plot(landmarks(:,1), landmarks(:,2), 'b*', 'MarkerSize',10);
title('Estimación de posición del robot - EKF');
xlabel('X [m]'); ylabel('Y [m]');
legend('Estimación EKF','Marcas observadas');

% Leer datos del archivo (7 columnas separadas por comas)
data = readmatrix('data.txt');

% Posición real del robot
robot_x = data(:,2); % columna 2 en x
robot_y = data(:,1); % columna 1 en y

% Posición deseada
goal_y  = data(:,4); % columna 4 en x
goal_x  = data(:,5); % columna 5 en y

% Velocidades
v = data(:,6); % velocidad lineal
w = data(:,7); % velocidad angular

% ====== Gráfica 1: Ruta deseada vs ruta seguida ======
figure;
plot(robot_x, robot_y, 'b-', 'LineWidth',1.5, 'DisplayName','Ruta seguida (real)'); hold on;
plot(goal_x, goal_y, 'r--o', 'LineWidth',0.2, 'MarkerSize',4, 'DisplayName','Ruta deseada');
xlabel('X Robot, Goal');
ylabel('Y Robot Goal');
title('Posición real vs seguida.');
legend('Location','best');
grid on; 

% ====== Grafica 2a: Velocidad lineal ======
N = length(v);              
dt = 0.001;                 
t = (0:N-1)*dt;             

figure;
plot(t, v, 'b', 'LineWidth',1.5);
xlabel('Tiempo [s]');
ylabel('v [m/s]');
title('V. α=2.5,  β=0.5');
grid on;
axis tight;               % ajusta los ejes al rango de los datos
pbaspect([2 1 1]);        % proporción rectangular (ancho:alto:profundidad)

% ====== Grafica 2b: Velocidad angular ======
figure;
plot(t, w, 'r', 'LineWidth',1.5);
xlabel('Tiempo [s]');
ylabel('\omega [rad/s]');
title('\omega. α=2.5,  β=0.5');
grid on;
axis tight;
pbaspect([2 1 1]);        % misma proporción rectangular

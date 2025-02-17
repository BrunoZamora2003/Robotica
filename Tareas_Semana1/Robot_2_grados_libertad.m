% Limpieza de pantalla
clear all
close all
clc

% Declaración de variables simbólicas
syms th1(t) th2(t) l1 l2

% Configuración del robot, 0 para junta rotacional, 1 para junta prismática
RP = [0 0]; % Ambas articulaciones son rotacionales

% Creamos el vector de coordenadas generalizadas
Q = [th1, th2];
disp('Coordenadas generalizadas');
pretty(Q);

% Creamos el vector de velocidades generalizadas
Qp = diff(Q, t);
disp('Velocidades generalizadas');
pretty(Qp);

% Número de grados de libertad del robot
GDL = size(RP, 2);
gdl = num2str(gdl);
disp(['Número de grados de libertad: ', GDL_str]);

% Junta 1
% Posición de la junta 1 respecto a 0
P(:,:,1) = [l1 * cos(th1); 
            l1 * sin(th1); 
            0]; % Incluimos z = 0 aunque no haya movimiento en z

% Matriz de rotación de la junta 1 respecto a 0
R(:,:,1) = [cos(th1) -sin(th1) 0;
            sin(th1)  cos(th1) 0;
            0         0        1];

% Junta 2
% Posición de la junta 2 respecto a 1
P(:,:,2) = [l2 * cos(th2); 
            l2 * sin(th2); 
            0]; % Incluimos z = 0 aunque no haya movimiento en z

% Matriz de rotación de la junta 2 respecto a 1
R(:,:,2) = [cos(th2) -sin(th2) 0;
            sin(th2)  cos(th2) 0;
            0         0        1];

% Resultados
disp('Posición de la junta 1 respecto a 0:');
pretty(P(:,:,1));

disp('Matriz de rotación de la junta 1 respecto a 0:');
pretty(R(:,:,1));

disp('Posición de la junta 2 respecto a junta 1:');
pretty(P(:,:,2));

disp('Matriz de rotación de la junta 2 respecto a junta 1:');
pretty(R(:,:,2));

% Derivadas parciales para el Jacobiano de velocidad lineal
% Derivadas parciales de la posición de la junta 1 respecto a th1 y th2
% Se trabaja en la junta 1
Jv11 = functionalDerivative(P(1,:,1), th1); % Derivada de x respecto a th1
Jv21 = functionalDerivative(P(2,:,1), th1); % Derivada de y respecto a th1
Jv31 = functionalDerivative(P(3,:,1), th1); % Derivada de z respecto a th1

% Derivadas parciales de la posición de la junta 2 respecto a th1 y th2
% Se trabaja en la junta 2
Jv12 = functionalDerivative(P(1,:,2), th2); % Derivada de x respecto a th2
Jv22 = functionalDerivative(P(2,:,2), th2); % Derivada de y respecto a th2
Jv32 = functionalDerivative(P(3,:,2), th2); % Derivada de z respecto a th2

% Jacobiano de velocidad lineal 
Jacobiano_lineal = simplify([Jv11, Jv12;
                             Jv21, Jv22;
                             Jv31, Jv32]);

disp('Jacobiano de velocidad lineal:');
pretty(Jacobiano_lineal);

% Jacobiano de velocidad angular
% Para juntas rotacionales, la velocidad angular es simplemente el vector [0; 0; 1] multiplicado por la velocidad articular
Jacobiano_angular = [0 0;  % No hay contribución de th1 en la velocidad angular en x
                     0 0;  % No hay contribución de th1 en la velocidad angular en y
                     1 1]; % th1 y th2 contribuyen directamente a la velocidad angular en z

disp('Jacobiano de velocidad angular:');
disp(Jacobiano_angular);

% Jacobiano completo (combinación de Jacobiano lineal y angular)
Jacobiano_completo = [Jacobiano_lineal;
                      Jacobiano_angular];

disp('Jacobiano completo (lineal + angular):');
disp(Jacobiano_completo);
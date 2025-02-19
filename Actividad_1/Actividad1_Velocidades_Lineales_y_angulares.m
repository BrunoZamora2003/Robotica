% Limpieza de pantalla
clear all
close all
clc

% Declaración de variables simbólicas (No tienen un valor específico)
syms th1(t) th2(t) l1 l2 t  

% Configuración del robot, 0 para junta rotacional, 1 para junta prismática
RP = [0, 0]; % Ambas articulaciones son rotacionales

% Creamos el vector de coordenadas articulares
Q = [th1(t); th2(t)];
disp('Coordenadas articulares');
pretty(Q);

% Creamos el vector de velocidades articulares
Qp = diff(Q, t); % Utilizo diff para derivadas cuya variable de referencia no depende de otra: ejemplo el tiempo
disp('Velocidades articulares');
pretty(Qp);

% Número de grado de libertad del robot
GDL = size(RP, 2); % Siempre se coloca 2, ya que indica la dimensión de las columnas
GDL_str = num2str(GDL); % Convertimos el valor numérico a una cadena de carácteres tipo string

% Articulación 1 
% Posición de la junta 1 respecto a 0
P(:,:,1) = [l1 * cos(th1);
            l1 * sin(th1);
            0]; % Vector de posición indexado por página

% Matriz de rotación de la articulación 1 respecto a 0
R(:,:,1) = [cos(th1) -sin(th1) 0;
            sin(th1)  cos(th1) 0;
            0         0        1];

% Articulación 2
% Posición de la junta 2 respecto a 1
P(:,:,2) = [l2 * cos(th1 + th2);
            l2 * sin(th1 + th2);
            0]; % Vector de posición indexado por página

% Matriz de rotación de la articulación 2 respecto a 1
R(:,:,2) = [cos(th1 + th2) -sin(th1 + th2) 0;
            sin(th1 + th2)  cos(th1 + th2) 0;
            0               0              1];

% Creamos un vector de ceros
Vector_Zeros = zeros(1, 3);

% Inicializamos las matrices de transformación Homogénea locales
A(:,:,1) = simplify([R(:,:,1) P(:,:,1); Vector_Zeros 1]);
A(:,:,2) = simplify([R(:,:,2) P(:,:,2); Vector_Zeros 1]);

% Inicializamos las matrices de transformación Homogénea globales
T(:,:,1) = A(:,:,1);
T(:,:,2) = simplify(T(:,:,1) * A(:,:,2));

% Inicializamos los vectores de posición vistos desde el marco de referencia inercial
PO(:,:,1) = P(:,:,1);
PO(:,:,2) = T(1:3,4,2);

% Inicializamos las matrices de rotación vistas desde el marco de referencia inercial
RO(:,:,1) = R(:,:,1);
RO(:,:,2) = T(1:3,1:3,2);

for i = 1:GDL
    i_str = num2str(i);
    % Locales
    disp(strcat('Matriz de Transformación local A', i_str));
    pretty(A(:,:,i));

    % Globales
    disp(strcat('Matriz de Transformación global T', i_str));
    pretty(T(:,:,i));

    % Obtenemos la matriz de rotación "RO" y el vector de translación PO de la
    % matriz de transformación Homogénea global T(:,:,GDL)
    RO(:,:,i) = T(1:3,1:3,i);
    PO(:,:,i) = T(1:3,4,i);
    pretty(RO(:,:,i));
    pretty(PO(:,:,i));
end

%%%%%%%%%%%Hasta aqui Cinemática directa%%%%%%%%%%%%%%%%%%%

% Calculamos el jacobiano lineal de forma diferencial
disp('Jacobiano lineal obtenido de forma diferencial');
% Derivadas parciales de x respecto a th1 y th2
Jv11 = functionalDerivative(PO(1,1,GDL), th1);
Jv12 = functionalDerivative(PO(1,1,GDL), th2);
% Derivadas parciales de y respecto a th1 y th2
Jv21 = functionalDerivative(PO(2,1,GDL), th1);
Jv22 = functionalDerivative(PO(2,1,GDL), th2);
% Derivadas parciales de z respecto a th1 y th2
Jv31 = functionalDerivative(PO(3,1,GDL), th1);
Jv32 = functionalDerivative(PO(3,1,GDL), th2);

% Creamos la matríz del Jacobiano lineal
jv_d = simplify([Jv11 Jv12;
                 Jv21 Jv22;
                 Jv31 Jv32]);
pretty(jv_d);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculamos el jacobiano lineal y angular de forma analítica
% Inicializamos jacobianos analíticos (lineal y angular)
Jv_a = sym(zeros(3, GDL));
Jw_a = sym(zeros(3, GDL));

for k = 1:GDL
    if (RP(k) == 0) % Casos: articulación rotacional
        % Para las articulaciones rotacionales
        try
            Jv_a(:,k) = cross(RO(:,3,k-1), PO(:,:,GDL) - PO(:,:,k-1));
            Jw_a(:,k) = RO(:,3,k-1);
        catch
            Jv_a(:,k) = cross([0,0,1], PO(:,:,GDL)); % Matriz de rotación de 0 con respecto a 0 es la Matriz Identidad, la posición previa también será 0
            Jw_a(:,k) = [0,0,1]; % Si no hay matriz de rotación previa se obtiene la Matriz identidad
        end
    elseif (RP(k) == 1) % Casos: articulación prismática
        % Para las articulaciones prismáticas
        try
            Jv_a(:,k) = RO(:,3,k-1);
        catch
            Jv_a(:,k) = [0,0,1]; % Si no hay matriz de rotación previa se obtiene la Matriz identidad
        end
        Jw_a(:,k) = [0,0,0];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%DESPLIEGUE DE CINEMATICA DIFERENCIAL%%%%%%%%%%%%%%%%%%%%%%%%

Jv_a = simplify(Jv_a);
Jw_a = simplify(Jw_a);
disp('Jacobiano lineal obtenido de forma analítica');
pretty(Jv_a);
disp('Jacobiano ángular obtenido de forma analítica');
pretty(Jw_a);

disp('Velocidad lineal obtenida mediante el Jacobiano lineal');
V = simplify(Jv_a * Qp);
pretty(V);
disp('Velocidad angular obtenida mediante el Jacobiano angular');
W = simplify(Jw_a * Qp);
pretty(W);
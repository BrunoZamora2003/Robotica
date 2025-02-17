clear all
close all
clc

syms x y z

%F(x, y, z) , componentes
F1 = sin(5*x^3 + 3*y - 4*y*x*z^2);       % Primera subfunción de F
F2 = -10*x^5 - 4*y*x*z + 15*x*z^4;       % Segunda subfunción de F
F3 = cos(-x*y*z^5 - 6*x*y^5*z - 7*y*x*z^2); % Tercera subfunción de F

%Vector de funciones componentes de F(x, y, z)
F = [F1; F2; F3];

% Calcular la matriz jacobiana usando la función jacobian
matriz_jacobiana = jacobian(F, [x, y, z]);

disp('Matriz Jacobiana (derivadas parciales):');
pretty(matriz_jacobiana)

% Valores de la matriz jacobiana en el punto (-5(x), -4(y), 1(z))
Jacobiana_evaluada = subs(matriz_jacobiana, {x, y, z}, {-5, -4, 1});

% Mostrar el resultado
disp('*******************************************************************************')
disp('Matriz Jacobiana evaluada en el punto(-5, -4, 1):');
pretty(Jacobiana_evaluada);
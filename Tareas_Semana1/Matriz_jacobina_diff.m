clear all
close all
clc

%Variables simbólicas
syms x y z

%Función F(x, y, z)
F1 = sin(5*x^3 + 3*y - 4*y*x*z^2);       % Primera subfuncion de F
F2 = -10*x^5 - 4*y*x*z + 15*x*z^4;       % Segunda subfuncion de F
F3 = cos(-x*y*z^5 - 6*x*y^5*z - 7*y*x*z^2); % Tercera subfuncion de F

%Aclaracion: 
%Para esta actividad no use "functionalDerivate", al intertarlo usar me marcaba error acerca de que x,y,z son variables simbolicas y, dicha funcion solo puede trabajar con funciones simbolicas. Investigando mejor, encontre la siguiente informcion: 
%FunctionalDerivative está diseñada para calcular derivadas funcionales, es decir, derivadas de funcionales (funciones de funciones). Esto es común en problemas de cálculo variacional, donde se trabaja con funcionales en lugar de funciones simples. 
%En mi caso como estoy trabajando con derivadas parciales de expresiones simbolicas con respecto a variables simbolicas (x,y,z). Por eso decidi usar  la funcion "diff", puesto que esta funcion esta disenada para trabajar con derivadas parciales de expresiones simbolicas con respecto a variables simbolicas.

% Derivadas parciales de F1 con respecto a x,y,z
dF1dx = diff(F1, x);  
dF1dy = diff(F1, y);  
dF1dz = diff(F1, z);  

% Derivadas parciales de F1 con respecto a x,y,z
dF2dx = diff(F2, x);  
dF2dy = diff(F2, y);  
dF2dz = diff(F2, z);  

% Derivadas parciales de F1 con respecto a x,y,z
dF3dx = diff(F3, x);  
dF3dy = diff(F3, y); 
dF3dz = diff(F3, z);  

% Matriz jacobiana
matriz_jacobiana = [dF1dx, dF1dy, dF1dz;
                    dF2dx, dF2dy, dF2dz;
                    dF3dx, dF3dy, dF3dz];

disp('Matriz Jacobiana (derivadas parciales):');
pretty(matriz_jacobiana)

% Valores de la matriz jacobiana en el punto (-5(x), -4(y), 1(z)
Jacobiana_evaluada = subs(matriz_jacobiana, {x, y, z}, {-5, -4, 1});

% Mostrar el resultado
disp('*******************************************************************************')
disp('Matriz Jacobiana evaluada en (-5, -4, 1):');
pretty(Jacobiana_evaluada);
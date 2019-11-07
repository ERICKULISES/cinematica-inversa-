%%Estevez Moran Erick Ulises
clear all
close all
clc

%% Lectura de datos
prompt = 'Introduza la longitud del primer eslabón:      ';
L1 = input(prompt);
prompt = 'Introduza la longitud del segundo eslabón:     ';
L2 = input(prompt);
prompt = 'Introduza la longitud del tercer eslabón:      ';
L3 = input(prompt);

prompt = 'Introduza el punto final en x:    ';
Xf = input(prompt);
prompt = 'Introduza el punto final en y:    ';
Yf = input(prompt);
prompt = 'Introduza el punto final en z:    ';
Zf = input(prompt);

%% Definición de los parametros de Denavit-Hartenberg
D1 = L1;
D2 = 0;
D3 = 0;

A1 = 0;
A2 = L2;
A3 = L3;

Alpha1 = 90;
Alpha2 = 0;
Alpha3 = 0;

Angulo1 = deg2rad(Alpha1);
Angulo2 = deg2rad(Alpha2);
Angulo3 = deg2rad(Alpha3);

%% Defnición del punto inicial y angulos de rotación del robot

    P1 = [0,0,0];

    Theta1 = atan2(Yf,Xf);

    CTheta3 = ((Xf^2+Yf^2+(Zf-L1)^2-L2^2-L3^2)/(2*L2*L3));
    Theta3 = atan2((sqrt(1-CTheta3^2)),CTheta3);

    Beta = atan2((Zf-L1),(Xf^2+Yf^2));
    Alpha = atan2((L3*(1-CTheta3)),(L2+L3*CTheta3));

    Theta2 = Beta-Alpha;

    AnguloT1 = Theta1;
    AnguloT2 = Theta2;
    AnguloT3 = Theta3;
%% Definición de movimientos del robot

if AnguloT1>=0
    Angulo = 0: 0.01:AnguloT1;
else
    Angulo = 0:-0.01:AnguloT1;
end

for i=1:length(Angulo)
    clf
    printAxis();
    grid on 
    Rz = [cos(Angulo(i)) -sin(Angulo(i))  0; sin(Angulo(i)) cos(Angulo(i)) 0; 0 0 1];
    AI = dhParameters(Angulo(i),D1,A1,Angulo1);
    AII = dhParameters(0,D2,A2,Angulo2);
    AIII = dhParameters(0,D3,A3,Angulo3);
    
    Af1 = AI*AII;
    Af2 = AI*AII*AIII; 

    P1 = [0 0 0]';
    P2 = AI(1:3,4);
    P3 = Af1(1:3,4);
    P4 = Af2(1:3,4);
    
    printLink(P1,P2);
    printLink(P2,P3);
    printLink(P3,P4);
    printMiniAxes(P1,Rz);
    printMiniAxes(P2,Af1);
    printMiniAxes(P3,Af2);
    printMiniAxes(P4,Af2);
    view(30,30);
    pause(0.01);
end
    pause(1);
       
   
if AnguloT2>=0
    Angulo = 0: 0.01:AnguloT2;
else
    Angulo = 0:-0.01:AnguloT2;
end

for i=1:length(Angulo)
    clf
    printAxis();
    grid on 
    Rz = [cos(AnguloT1) -sin(AnguloT1)  0; sin(AnguloT1) cos(AnguloT1) 0; 0 0 1];
    AI = dhParameters(AnguloT1,D1,A1,Angulo1);
    AII = dhParameters(Angulo(i),D2,A2,Angulo2);
    AIII = dhParameters(0,D3,A3,Angulo3);
    
    Af1 = AI*AII;
    Af2 = AI*AII*AIII; 

    P1 = [0 0 0]';
    P2 = AI(1:3,4);
    P3 = Af1(1:3,4);
    P4 = Af2(1:3,4);
    
    printLink(P1,P2);
    printLink(P2,P3);
    printLink(P3,P4);
    printMiniAxes(P1,Rz);
    printMiniAxes(P2,Af1);
    printMiniAxes(P3,Af2);
    printMiniAxes(P4,Af2);
    view(30,30);
    pause(0.01);
end

pause(1);

if AnguloT3>=0
    Angulo = 0: 0.01:AnguloT3;
else
    Angulo = 0:-0.01:AnguloT3;
end

for i=1:length(Angulo)
    clf
    printAxis();
    grid on 
    Rotz = [cos(AnguloT1) -sin(AnguloT1)  0; sin(AnguloT1) cos(AnguloT1) 0; 0 0 1];
    AI = dhParameters(AnguloT1,D1,A1,Angulo1);
    AII = dhParameters(AnguloT2,D2,A2,Angulo2);
    AIII = dhParameters(Angulo(i),D3,A3,Angulo3);
    
    Af1 = AI*AII;
    Af2 = AI*AII*AIII; 

    P1 = [0 0 0]';
    P2 = AI(1:3,4);
    P3 = Af1(1:3,4);
    P4 = Af2(1:3,4);
    
    printLink(P1,P2);
    printLink(P2,P3);
    printLink(P3,P4);
    printMiniAxes(P1,Rz);
    printMiniAxes(P2,Af1);
    printMiniAxes(P3,Af2);
    printMiniAxes(P4,Af2);
    
    line([0 Xf],[0 Yf],[0 Xf],'color',[0.8 0.8 0.2],'linewidth',2); 
    
    view(30,30);
    pause(0.01);
end



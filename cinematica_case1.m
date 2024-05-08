%% Simulación robot con ruedas - Fundamentos de Robótica móvil
% Universidad Nacional de Colombia - Sede Bogotá (2024-1S)
% Equipo 1: Kobuki
% Daniel Esteban Molano Garzón
% Camilo Esteban Zambrano Pereira
% Cristhian Sandoval
% Juan Sebastián Dueñas

%% Programa para el caso 1

%Programa para verificar el uso de handles y desde MATLAB leer variables en Coppelia Sim.
%Establecer la conexión
vrep=remApi('remoteApi'); % usar el archivo prototipo (remoteApiProto.m)
vrep.simxFinish(-1); % si se requiere, cerrar todas las conexiones abiertas.
% asigna el handle de identificación de cliente clientID
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Conexión exitosa')
end
%Algoritmo
% Consulta el handle del objeto Caja1 en la escena Esc01 y lo asigna al handle caja_m.
[returnCode,kobuki_m]=vrep.simxGetObjectHandle(clientID,'kobuki',vrep.simx_opmode_blocking);

% Ruedas
[returnCode,wR]=vrep.simxGetObjectHandle(clientID,'jointR',vrep.simx_opmode_blocking);
[returnCode,wL]=vrep.simxGetObjectHandle(clientID,'jointL',vrep.simx_opmode_blocking);

r= 0.0684/2;                        %Radio de las ruedas motrices [m]
l= 0.1025;                          %Distancia del centroide del robot a la rueda [m]
vel_ang = 10;                       %Velocidad angular de la rueda 1  [rad/s]
vel_lin = vel_ang*r;                %Velocidad lineal del robot[m/s]
x = 4;                              %Distancia a recorrer [m]
d_duration = x/vel_lin;             %Duración del recorrido [s]

% Calculo de la velocidad angular de las ruedas a partir del jacobiano del
% sistema y la velocidad angular objetivo
vel_rotation = [1/r l/r;1/r -l/r]*[0; pi/2];

disp("d_duration: "+ d_duration)

vrep.simxGetPingTime(wR)
vrep.simxGetPingTime(wL)
% Se establece la posición y orientación inicial del robot
Pos_ini=[2,2,0.0445];
Or_ini=[0,0,pi/2];
[returnCode]= vrep.simxSetObjectPosition(clientID, kobuki_m,-1,Pos_ini,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetObjectOrientation(clientID,kobuki_m,-1,Or_ini,vrep.simx_opmode_blocking);
% Bucle para definir el movimiento del robot
for c = 1:4
    vrep.simxSetJointTargetVelocity(clientID, wL, vel_ang, vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, wR, vel_ang, vrep.simx_opmode_oneshot);
    pause(d_duration);
    vrep.simxSetJointTargetVelocity(clientID, wL, 0, vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, wR, 0, vrep.simx_opmode_oneshot);
    pause(1)
    vrep.simxSetJointTargetVelocity(clientID, wL, vel_rotation(1), vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, wR, vel_rotation(2), vrep.simx_opmode_oneshot);
    pause(1)
    vrep.simxSetJointTargetVelocity(clientID, wL, 0, vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, wR, 0, vrep.simx_opmode_oneshot);
    pause(1)
end
vrep.simxSetJointTargetVelocity(clientID, wR, 0, vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetVelocity(clientID, wL, 0, vrep.simx_opmode_oneshot)

[returnCode,P]=vrep.simxGetObjectPosition(clientID,kobuki_m,-1,vrep.simx_opmode_blocking);

disp('Programa terminado')
vrep.delete(); % llama el destructor!

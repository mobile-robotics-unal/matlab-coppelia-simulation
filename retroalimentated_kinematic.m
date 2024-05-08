%% Simulación robot con ruedas - Fundamentos de Robótica móvil
% Universidad Nacional de Colombia - Sede Bogotá (2024-1S)
% Equipo 1: Kobuki
% Daniel Esteban Molano Garzón
% Camilo Esteban Zambrano Pereira
% Cristhian Sandoval
% Juan Sebastián Dueñas
%% Programa para el caso 2


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
[returnCode_K,kobuki_m]=vrep.simxGetObjectHandle(clientID,'kobuki',vrep.simx_opmode_blocking);

[returnCode_K,floor_id]=vrep.simxGetObjectHandle(clientID,'Floor',vrep.simx_opmode_blocking);

% Ruedas
[returnCode_R,wR]=vrep.simxGetObjectHandle(clientID,'jointR',vrep.simx_opmode_blocking);
[returnCode_L,wL]=vrep.simxGetObjectHandle(clientID,'jointL',vrep.simx_opmode_blocking);

%Se define una matriz con los vértices del cuadrado.

lado = 4;
vertices = [2,2,0.0445;
            2,-2,0.0445;
            -2,-2,0.0445;
            -2,2,0.0445;
            2,2,0.0445];



% Se establece la posición y orientación inicial del robot
Pos_ini=[2,2,0.0445];
Or_ini=[0,0,pi/2];
Pos_ini=[-2,2,0.0445];
Or_ini=[0, 0, 0];
[returnCode]= vrep.simxSetObjectPosition(clientID, kobuki_m,-1,Pos_ini,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetObjectOrientation(clientID,kobuki_m,-1,Or_ini,vrep.simx_opmode_blocking);

[returnCode, orient] = vrep.simxGetObjectOrientation(clientID, kobuki_m, floor_id, vrep.simx_opmode_blocking)
[returnCode, pos] = vrep.simxGetObjectPosition(clientID, kobuki_m, floor_id, vrep.simx_opmode_blocking);

pose = []
error_min = [0.2,0.001];
log_error =[];

%% Se establecen las condiciones para el movimiento del robot

get_pose = @()(get_robot_pose(vrep, clientID, kobuki_m, floor_id));

for i=1:4

    pose(end+1,:) = [pos(1:2),orient(3)];;

    vertice_objetivo = vertices(i+1,:);   
    
    error_orient = orient_objetivo - orient(3);

    while  abs(error_orient) > error_min(2)
        orient_objetivo = atan2(vertice_objetivo(2) - pos(2), vertice_objetivo(1) - pos(1));
        [pos, orient] = get_pose();
        error_orient = orient_objetivo - orient(3);
        vel_wheels = move_robot(vrep,clientID, 0, error_orient);
        log_error(end+1,:) = [ (vertice_objetivo(1:2) - pos(1:2)), error_orient];
        pose(end+1,:) = [pos(1:2),orient(3)];
        
        message = "robot orient "+(orient(3)*180/pi) + " target orient " + (orient_objetivo*180/pi);
        message = message + " error orient " + (error_orient*180/pi);
        disp(message)
        disp(vel_wheels)
        %pause(1)

    end

    error_pos = norm(vertice_objetivo(1:2) - pos(1:2));
    
    while  abs(error_pos) > error_min(1)
        [pos, orient] = get_pose();
        orient_objetivo = atan2(vertice_objetivo(2) - pos(2), vertice_objetivo(1) - pos(1));
        error_orient = orient_objetivo - orient(3);
        delta_pos = vertice_objetivo(1:2) - pos(1:2);
        error_pos = norm(vertice_objetivo(1:2)-pos(1:2));
        log_error(end+1,:) =[ vertice_objetivo(1:2) - pos(1:2), error_orient];
        vel_wheels = move_robot(vrep,clientID, 0.5*error_pos , 0.3*error_orient);
        pose(end+1,:) = [pos(1:2),orient(3)];
        message =sprintf('pos [%2.3f %2.3f] target pos [%2.3f %2.3f] error pos [%2.3f %2.3f]',pos(1),pos(2), vertice_objetivo(1), vertice_objetivo(1), delta_pos(1),delta_pos(2));
        
        disp(message)
        disp(vel_wheels)
    end

    
end

vel_wheels = move_robot(vrep,clientID, 0, 0);
[returnCode,P]=vrep.simxGetObjectPosition(clientID,kobuki_m,-1,vrep.simx_opmode_blocking);
 
disp('Programa terminado')
vrep.delete(); % llama el destructor!

%%
close 'all'
subplot(2,2,1)
plot(log_error)
legend(["x","y","\theta"])

subplot(2,2,2)
plot(pose(:,1),pose(:,2),vertices(:,1),vertices(:,2))
axis equal

subplot(2,2,3)
plot(pose(:,3))

function vel_wheels = move_robot(vrep,clientID,vel_lineal,vel_angular)
    r= 0.0684/2;            %Radio de las ruedas motrices [m]
    L= 0.1025;              %Distancia del centroide del robot a la rueda [m]
    %Velocidad angular de la rueda 1 [rad/s]
    %Velocidad lineal del robot[m/s]
    vel_wheels = [-1/r L/r; -1/r -L/r]*[vel_lineal; vel_angular];

    % Ruedas
    [returnCode_R,wR]=vrep.simxGetObjectHandle(clientID,'jointR',vrep.simx_opmode_blocking);
    [returnCode_L,wL]=vrep.simxGetObjectHandle(clientID,'jointL',vrep.simx_opmode_blocking);
    
    vrep.simxSetJointTargetVelocity(clientID, wR, vel_wheels(1), vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID, wL, vel_wheels(2), vrep.simx_opmode_oneshot);
end

function [pos,orient] = get_robot_pose(vrep,clientID,robot_id, world_id)
    [returnCode, orient] = vrep.simxGetObjectOrientation(clientID, robot_id, world_id, vrep.simx_opmode_blocking);
    [returnCode, pos] = vrep.simxGetObjectPosition(clientID, robot_id, world_id, vrep.simx_opmode_blocking);
end
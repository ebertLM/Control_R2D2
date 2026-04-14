% Tracking usando un controlador por cinemática inversa.
% La trayectoria se establece en el espacio operacional
clc; clearvars; close all;

%______________ 1. PARÁMETROS DEL ROBOT ___________________
load_parameters

%______________ 2. CONFIGURACION DEL MOVIMIENTO ___________________
%  2.1. Parámetros de la rura geométrica circular
r  = [1;0;0];                 % Vector unitario del eje    
d  = [0.216;   0;   0.300];   % Punto en el eje (puede ser el centro)
po = [0.216; 0.105; 0.300];   % Posicion inicial del circulo 
so = 0;                       % Longitud inicial del circulo
centralAngle = 2*pi;          % Arco a recorrer

%  2.2. Tiempo de muestreo
Ts = 0.005;     % 5ms es optimo en tiempo real
tadd = 1.0;     % 1s de tiempo adicional al finalizar la trayectoria

%  2.3. Parámetros del perfil de velocidad trapezoidal (s)
factor = 0.35;
ds_max = 1.0;
dds_c = 1.2;

%  2.4. Creación del objeto para la planificación de trayectoria
circlePlanner = TrajectoryPlanningCircle(r, d, po, so, centralAngle, ...
                                         Ts, tadd, factor, ds_max, ...
                                         dds_c);

%______________ 3. PARAMETROS DEL ALGORITMO DE CINEMATICA INVERSA __________
%  3.1. Constantes de sintonización
% Kp = diag([100, 100, 100]);
% Kd = diag([20, 20, 20]);
Kp = diag([225, 225, 225]);
Kd = diag([30, 30, 30]);

%  3.2. Objetos: cinemática, dinámica y el control
cin = RobotCinematica(ROBOT_DATA);
dyn = RobotDinamica(ROBOT_DATA);

%  3.3. Objeto: control por cinemática inversa
CCI_R2D2 = CIK_Robot3DOF(Kp, Kd, ROBOT_DATA, cin, dyn);

%  3.4. Configurar los límites de los torques
CCI_R2D2.set_outputMaxTorque();

%  3.5. Configurar los límites de los voltajes de control
CCI_R2D2.set_outputMaxVoltage();

%______________ 4. OBJETOS GRÁFICOS __________
%  4.1. Objeto para la gráfica
Plotter = RobotVisualization(ROBOT_DATA);
ax = Plotter.drawAxesOfSimpleBox();
%ax = Plotter.drawInitialAxes();

%______________ 5. CONFIGURACION INICIAL DEL ROBOT __________
%  5.1. Configuración inicial de las juntas
q1 = 0; q2 =  pi/2; q3 = -pi/2;
q = [q1;q2;q3];

%  5.2. Configuración inicial del punto de trabajo del robot
xo = cin.get_posXYZ(q,"pt"); % Cinemática directa del punto final

%  5.3. Velocidad inicial de las juntas
dq1 = 0; dq2 = 0; dq3 = 0;
dq = [dq1;dq2;dq3];

%  5.4. Aceleracion inicial de las juntas
ddq1 = 0; ddq2 = 0; ddq3 = 0;
ddq = [ddq1;ddq2;ddq3];

%______________ 6. DEFINICIÓN DE MATRICES PARA GUARDAR VALORES __________
tf = circlePlanner.get_tf();
td = tf + tadd;

MM = floor(td/Ts);

Q   = zeros(3,MM);      % Posición angular del punto de trabajo
QD  = zeros(3,MM);      % Posición angular deseada del punto de trabajo
dQ  = zeros(3,MM);      % Velocidad angular del punto de trabajo 
ddQ = zeros(3,MM);      % Aceleración angular del Punto de trabajo
errXYZ = zeros(3,MM);   % Error en el espacio operacional del punto de trabajo
PD = zeros(3,MM);       % Posicion deseada del punto de trabajo
dPD = zeros(3,MM);      % Velocidad deseada del punto de trabajo
ddPD = zeros(3,MM);     % Aceleración deseada del punto de trabajo
VA  = zeros(3,MM);      % Voltaje de armadura
VC  = zeros(3, MM);     % Voltaje de control
TM = zeros(3,MM);      % Torque en el eje del rotor
TAU_L  = zeros(3,MM);      % Torque en las juntas
XYZ = zeros(3,MM);      % Posición en el espacio operacional del punto de trabajo
S   = zeros(1,MM);      % Distancia dada por la ley temporal s(t)
dS  = zeros(1,MM);      % Velocidad de s(t)
ddS = zeros(1,MM);      % Aceleración de s(t)

%pause
for k=1:MM 
    %______________ 7. OBTENCIÓN DE LA TRAYECTORIA __________
    % Comprueba que q1,q2,q3 estén dentro de los límites.
    cin.verifyAngleQ(q);

    t = k*Ts;
    % perfil trapezoidal
    [s, ds, dds] = circlePlanner.profile(t); 

    % Posicion, velocidad y aceleracion
    [pd, dpd, ddpd] = circlePlanner.trajectory(s, ds, dds);  
    
    qd = cin.get_posQ(pd,"pt");
    
    PD(:,k)  = pd; % Coordenadas xyz deseadas del punto de trabajo
    % dPD(:,k) = dpd; 
    % ddPD(:,k) = ddpd; 
    
    %  GUARDAMOS LAS SENALES
    Q(:,k)   = q;  
    QD(:,k)  = qd;
    dQ(:,k)  = dq; 
    S(k)     = s;  
    dS(k)    = ds;
    ddS(k)   = dds;
    
    %______________ 8. CONTROL POR CINEMÁTICA INVERSA __________
    % B.1. CINEMATICA OPERACIONAL
    xyz_pt = cin.get_posXYZ(q,"pt"); % Punto de trabajo actual
    XYZ(:,k) = xyz_pt; % Guardar el punto de trabajo actual
    errXYZ(:,k) = pd - xyz_pt; % Guardar el error del punto de trabajo actual
    
    [u, y] = CCI_R2D2.get_torque(pd, dpd, ddpd, q, dq);
    [Vc,Va] = CCI_R2D2.get_voltage_uC(u);
    
    VA(:,k) = Va;
    VC(:,k) = Vc; 
    %-----------------------------------------------------%
    % D. CALCULO DE LOS TORQUES
    %-----------------------------------------------------%
    % Recalcular el torque de control con los voltajes limitados.
    % Esto sólo es para la simulación y su cálculo de la dinámica directa
    ddq_y = y;
    [tau_l, ~, Tm] = CCI_R2D2.voltage2torque(Vc,dq,ddq_y);
    TM(:,k) = Tm;
    TAU_L(:,k) = tau_l;
    
    %-----------------------------------------------------%
    % E. RESPUESTA DINAMICA DEL MANIPULADOR
    %-----------------------------------------------------%
    % E.1. CALCULO DE LA ACELERACION
    ddq = dyn.dinamica_directa(tau_l,q,dq);
    ddQ(:,k) = ddq;
    % E.2. CALCULO DE LA VELOCIDAD Y CONFIGURACION
    dq = dq + Ts*ddq;
    q  = q + Ts*dq;
    
    %-------------------------------------------------------------%
    % F. PLOTEAMOS EL ROBOT
    %-------------------------------------------------------------%
    if(rem(k-1,5)==0)
        % Plotter.drawRobotLinks(q);
        Plotter.drawTrajectory(ax,xyz_pt);
        pause(0.05)
    end
end    

%_______________8. GUARDAR VALORES DE SIMULACION____________
% Crear la carpeta si no existe
if ~exist('Resultados_Laboratorio', 'dir')
    mkdir('Resultados_Laboratorio');
end

% 1. Definición del nombre base (Ruta y nombre del archivo)
nombre_base = fullfile('Resultados_Laboratorio', 'TrayecCirculoMenor_Sim');

% 2. Guardar las tres matrices en el archivo .mat
save([nombre_base '.mat'], 'Q', 'dQ', 'ddQ', 'XYZ', 'TAU_L', 'VC');

%______________ 9. SCRIPT DE GRÁFICOS __________
Graphics
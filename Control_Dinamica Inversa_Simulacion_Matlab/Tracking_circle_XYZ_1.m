% Tracking usando un controlador por cinemática inversa.
% La trayectoria se establece en el espacio operacional
clc; clearvars; close all;

%______________ A. PARÁMETROS DEL ROBOT ___________________
load_parameters  % ROBOT_DATA

%______________ B. CONFIGURACION DEL MOVIMIENTO ___________________
%  B.1. Parámetros de la rura geométrica circular
r  = [1;0;0];                 % Vector unitario del eje    
d  = [0.210;   0;   0.300];   % Punto en el eje (puede ser el centro)
po = [0.149; 0.149; 0.300];   % Posicion inicial del circulo

%  B.2. Tiempo de muestreo
Ts = 0.005;     % 5ms es optimo en tiempo real
tadd = 1.0;     % 1s de tiempo adicional al finalizar la trayectoria

%  B.3. Parámetros del perfil de velocidad trapezoidal (s)
so = 0;                       % Longitud inicial del circulo
centralAngle = 2*pi;          % Arco a recorrer
factor = 0.35;
ds_max = 1.0;
dds_c = 1.2;

%  B.4. Creación del objeto para la planificación de trayectoria
circlePlanner = TrajectoryPlanningCircle(r, d, po, so, centralAngle, ...
                                         Ts, tadd, factor, ds_max, ...
                                         dds_c);

%______________ C. PARAMETROS DEL ALGORITMO DE CINEMATICA INVERSA __________
%  C.1. Constantes de sintonización
Kp = diag([225, 225, 225]);
Kd = diag([30, 30, 30]);

%  C.2. Objetos: cinemática, dinámica y el control
cin = RobotCinematica(ROBOT_DATA);
dyn = RobotDinamica(ROBOT_DATA);

%  C.3. Objeto: control por cinemática inversa
CCI_R2D2 = CIK_Robot3DOF(Kp, Kd, ROBOT_DATA, cin, dyn);

%  C.4. Configurar los límites de los torques
CCI_R2D2.set_outputMaxTorque();

%  C.5. Configurar los límites de los voltajes de control
CCI_R2D2.set_outputMaxVoltage();

%______________ D. OBJETOS GRÁFICOS __________
%  D.1. Objeto para la gráfica
Plotter = RobotVisualization(ROBOT_DATA);
ax = Plotter.drawAxesOfSimpleBox();
%ax = Plotter.drawInitialAxes();

%______________ E. CONFIGURACION INICIAL DEL ROBOT __________
%  E.1. Configuración inicial de las juntas
q1 = 0; q2 =  pi/2; q3 = -pi/2;
q = [q1;q2;q3];

%  E.2. Posicion XYZ inicial del punto de trabajo del robot
xo = cin.get_posXYZ(q,"pt");

%  E.3. Velocidad inicial de las juntas
dq1 = 0; dq2 = 0; dq3 = 0;
dq = [dq1;dq2;dq3];

%  E.4. Aceleracion inicial de las juntas
ddq1 = 0; ddq2 = 0; ddq3 = 0;
ddq = [ddq1;ddq2;ddq3];

%______________ F. DEFINICIÓN DE MATRICES PARA GUARDAR VALORES __________
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
TM = zeros(3,MM);       % Torque en el eje del rotor
TAU_L  = zeros(3,MM);   % Torque en las juntas
XYZ = zeros(3,MM);      % Posición en el espacio operacional del punto de trabajo
S   = zeros(1,MM);      % Distancia dada por la ley temporal s(t)
dS  = zeros(1,MM);      % Velocidad de s(t)
ddS = zeros(1,MM);      % Aceleración de s(t)

%pause
for k=1:MM 
    %______________ G. OBTENCIÓN DE LA TRAYECTORIA __________
    % Comprueba que q1,q2,q3 estén dentro de los límites.
    q = [q(1);q(2);q(3)];
    cin.verifyAngleQ(q);

    t = k*Ts;
    % Perfil trapezoidal
    %   profile: Ver Ecuación (91)
    [s, ds, dds] = circlePlanner.profile(t); 

    % Posicion, velocidad y aceleracion
    %   trajectory: Ver Ecuaciones (85), (88) y (89)
    [pd, dpd, ddpd] = circlePlanner.trajectory(s, ds, dds);  
    
    % Cinemática inversa
    %   get_posQ: Ver Ecuaciones (105), (106) y (107)
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
    
    %______________ H. CONTROL POR CINEMÁTICA INVERSA __________
    % H.1. Cinematica operacional
    xyz_pt = cin.get_posXYZ(q,"pt"); % Punto de trabajo actual
    XYZ(:,k) = xyz_pt; % Guardar el punto de trabajo actual
    errXYZ(:,k) = pd - xyz_pt; % Guardar el error del punto de trabajo actual
    
    % H.2. Calculo del torque de control
    %   y: Ley de control auxiliar .... Ver la Ecuacion (62)   
    %   y = Ja_inv*(ddpd_ef + this.Kd*de + this.Kp*e - dJa*dq);
    %   Se cumple: y = ddq .... Ver la Ecuacion (60)   
    [u, y] = CCI_R2D2.get_torque(pd, dpd, ddpd, q, dq);

    % H.3. Calculo del voltaje de control
    %   get_voltage_uC: Ver Ecuacion (76)
    [Vc,Va] = CCI_R2D2.get_voltage_uC(u);
    
    VA(:,k) = Va;
    VC(:,k) = Vc; 

    %______________ I. DE VOLTAJE A TORQUE (Simulación) __________
    % Recalcular el torque de control con los voltajes limitados.
    % Esto sólo es para la simulación y su cálculo de la dinámica directa
    %   voltage2torque: Ver Ecuacion (81)
    ddq_y = y;
    [tau_l, ~, Tm] = CCI_R2D2.voltage2torque(Vc,dq,ddq_y);
    TM(:,k) = Tm;
    TAU_L(:,k) = tau_l;
    
    %______________ J. RESPUESTA DINAMICA DEL MANIPULADOR __________
    % J.1. Cálculo de la aceleración
    %   dinamica_directa: Ver Ecuacion (41)
    ddq = dyn.dinamica_directa(tau_l,q,dq);
    ddQ(:,k) = ddq;

    % J.2. Cálculo de la velocidad y posición angular según
    %      la discretización de Euler
    dq = dq + Ts*ddq;
    q  = q + Ts*dq;
    
    %______________ K. TRAYECTORIA EN EL ESPACIO OPERACIONAL __________
    if(rem(k-1,5)==0)
        % Plotter.drawRobotLinks(q);
        Plotter.drawTrajectory(ax,xyz_pt);
        pause(0.05)
    end
end    

%_______________L. GUARDAR VALORES DE SIMULACION____________
% Crear la carpeta si no existe
if ~exist('Resultados_Laboratorio', 'dir')
    mkdir('Resultados_Laboratorio');
end

% 1. Definición del nombre base (Ruta y nombre del archivo)
nombre_base = fullfile('Resultados_Laboratorio', 'Data');

% 2. Guardar las tres matrices en el archivo .mat
save([nombre_base '.mat'], 'Q','VC');

%______________ M. SCRIPT DE GRÁFICOS __________
Graphics
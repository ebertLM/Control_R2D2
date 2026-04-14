% Tracking usando un controlador por cinemática inversa.
% La trayectoria se establece en el espacio operacional
clc; clearvars; close all;

%______________ A. PARÁMETROS DEL ROBOT ___________________
load_parameters

%______________ B. CONFIGURACION DEL MOVIMIENTO ___________________
%  B.1. Parámetros de la rura geométrica circular
so = 0;                 % Longitud inicial del rectangulo
p1 = [0.149;  0.110; 0.200];  % Primer punto del rectángulo
p2 = [0.149;  0.110; 0.400];  % Segundo punto del rectángulo
p3 = [0.149; -0.110; 0.400];  % Tercer punto del rectángulo
p4 = [0.149; -0.110; 0.200];  % Cuarto punto del rectángulo

%  B.2. Tiempo de muestreo
Ts = 0.005;     % 5ms es optimo en tiempo real
tadd = 1.0;     % 1s de tiempo adicional al finalizar la trayectoria

%  B.3. Parámetros del perfil de velocidad trapezoidal (s)
factor = 0.35;
ds_max = 1.0;
dds_c = 1.2;

%  B.4. Creación del objeto para la planificación de trayectoria
rectanglePlanner = TrajectoryPlanningRectangle(p1, p2, p3, p4, so, Ts, tadd, ...
                                               factor, ds_max, dds_c);

%  B.5. Obtención de los tiempos de la trayectoria
tf1 = rectanglePlanner.get_tf1();
tf2 = rectanglePlanner.get_tf2();
tf3 = rectanglePlanner.get_tf3();
tf4 = rectanglePlanner.get_tf4();
tf_total = rectanglePlanner.get_totalTime();
td = tf_total + tadd;

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
% ax = Plotter.drawInitialAxes();

%______________ E. CONFIGURACION INICIAL DEL ROBOT __________
%  E.1. Configuración inicial de las juntas
q1 = 0; q2 =  pi/2; q3 = -pi/2;
q = [q1;q2;q3];

%  E.2. Configuración inicial del punto de trabajo del robot
xo = cin.get_posXYZ(q,"pt"); % Cinemática directa del punto final

%  E.3. Velocidad inicial de las juntas
dq1 = 0; dq2 = 0; dq3 = 0;
dq = [dq1;dq2;dq3];

%  E.4. Aceleracion inicial de las juntas
ddq1 = 0; ddq2 = 0; ddq3 = 0;
ddq = [ddq1;ddq2;ddq3];

%______________ F. DEFINICIÓN DE MATRICES PARA GUARDAR VALORES __________
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

% pause
for k=1:MM 
    %______________ G. OBTENCIÓN DE LA TRAYECTORIA __________
    % Comprueba que q1,q2,q3 estén dentro de los límites.
    cin.verifyAngleQ(q);

    % Cáculo del perfil trapezoidal y trayectoria planificada
    t = k*Ts;
    if t <= tf1
        currentSide = "lado1";
    elseif t <= tf1 + tf2
        t = t - tf1;
        currentSide = "lado2";
    elseif t <= tf1 + tf2 + tf3
        t = t - (tf1 + tf2);
        currentSide = "lado3";
    elseif t <= tf_total
        t = t - (tf1 + tf2 + tf3);
        currentSide = "lado4";
    else
        % Tiempo adicional al finalizar la trayectoria
        t = tf_total;
        currentSide = "lado4";
    end

    [s,ds,dds]    = rectanglePlanner.profile(t,currentSide);
    [pd,dpd,ddpd] = rectanglePlanner.trajectory(s,ds,dds,currentSide);

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
    
    % H.2. Calculo del torque y voltaje de control
    [u, y] = CCI_R2D2.get_torque(pd, dpd, ddpd, q, dq);
    [Vc,Va] = CCI_R2D2.get_voltage_uC(u); 
    
    VA(:,k) = Va;
    VC(:,k) = Vc;

    %______________ I. DE VOLTAJE A TORQUE (Simulación) __________
    % Recalcular el torque de control con los voltajes limitados.
    % Esto sólo es para la simulación y su cálculo de la dinámica directa
    ddq_y = y;
    [tau_l, ~, Tm] = CCI_R2D2.voltage2torque(Vc,dq,ddq_y);
    TM(:,k) = Tm;
    TAU_L(:,k) = tau_l;
    
    %______________ J. RESPUESTA DINAMICA DEL MANIPULADOR __________
    % J.1. CALCULO DE LA ACELERACION
    ddq = dyn.dinamica_directa(tau_l,q,dq);
    ddQ(:,k) = ddq;
    % J.2. CALCULO DE LA VELOCIDAD Y CONFIGURACION
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
nombre_base = fullfile('Resultados_Laboratorio', 'TrayecRectanguloMayor_Sim');

% 2. Guardar las tres matrices en el archivo .mat
save([nombre_base '.mat'], 'Q', 'dQ', 'ddQ', 'XYZ', 'TAU_L', 'VC');

%______________ M. SCRIPT DE GRÁFICOS __________
Graphics
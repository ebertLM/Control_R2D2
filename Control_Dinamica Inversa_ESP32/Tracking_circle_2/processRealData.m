% ___________ PARÁMETROS CONSTANTES _____________
Gv1 = ROBOT_DATA.Gv1;
Gv2 = ROBOT_DATA.Gv2;
Gv3 = ROBOT_DATA.Gv3;
Kt1 = ROBOT_DATA.Kt1;
Kt2 = ROBOT_DATA.Kt2;
Kt3 = ROBOT_DATA.Kt3;

% ___________ INICIALIZACIÓN DE VARIABLES _____________
dQ     = zeros(3,lengthColumn); % velocidad angular
ddQ    = zeros(3,lengthColumn); % aceleración angular
XYZ    = zeros(3,lengthColumn); % posición en el espacio de trabajo
dXYZ   = zeros(3,lengthColumn); % velocidad en el espacio de trabajo
errXYZ = zeros(3,lengthColumn); % error en el espacio de trabajo
VA     = zeros(3,lengthColumn); % voltaje de armadura
TT     = zeros(3,lengthColumn); % torque de control en las juntas
TTm    = zeros(3,lengthColumn); % torque en el eje del rotor
IC     = zeros(3,lengthColumn); % corriente de control

% ___________ TRAYECTORIA CIRCULAR _____________
%  1. Parámetros del círculo
r  = [1;0;0];              % Vector unitario del eje central del circulo
d  = [0.216;0;0.300];     % Punto en el eje (puede ser el centro)
po = [0.216;0.105;0.300];   % Posicion inicial del circulo en XYZ
so = 0;                    % Posicion inicial del arco 
centralAngle = 2*pi;       % Arco a recorrer

%  2. Parámetros del perfil trapezoidal en 's'
factor  = 0.35; 
ds_max  = 1.0;  
dds_c = 1.2;  

% 3. Parámetros temporales
Ts   = 0.005; % Tiempo de muestreo
tadd = 1.0;   %Tiempo adicional para que el robot se pare

% 4. Objetos para el cálculo de la trayectoria planificada
cin           = RobotCinematica(ROBOT_DATA);
circlePlanner = TrajectoryPlanningCircle(cin, r, d, po, so, centralAngle, Ts, ...
                                         tadd, factor, ds_max, dds_c);

% 5. Posición, velocidad y aceleración deseada 
%    en el espacio de trabajo y articular del punto de trabajo
[PD, dPD, ddPD, QD, dQD, ddQD] = circlePlanner.PosVelAcc();

%___________ VELOCIDAD Y ACELERACIÓN ANGULAR ______________
alpha_dQ = 0.34; % Parámetro de filtro EMA para dQ
alpha_ddQ = 0.10; % Parámetro de filtro EMA para dQ
q_old  = [0; pi/2; -pi/2];  % Posición angular inicial (Home)
dq_old = [0; 0; 0];         % Velocidad angular inicial
dQ_filt = zeros(3,lengthColumn);     % Reserva de memoria dQ_filt 
ddQ_filt = zeros(3,lengthColumn);    % Reserva de memoria ddQ_filt 

% 1. Filtro EMA para la velocidad angular dQ
Q_aug = [q_old,Q];                   % Posición angular inicial agregada   
dQ_unfiltered = diff(Q_aug,1,2)/Ts;  % Velocidad angular por diferenciación

dQ_filt(:,1) = zeros(3,1);           % dQ_filt inicial
for k = 2:lengthColumn               
    dQ_filt(:,k) = alpha_dQ*dQ_unfiltered(:,k) + (1-alpha_dQ)*dQ_filt(:,k-1);
end
dQ = dQ_filt;                        % Guardamos la velocidad filtrada

% 2. Filtro EMA para la aceleración angular ddQ
dQ_aug = [dq_old,dQ];                 % Velocidad angular inicial agregada 
ddQ_unfiltered = diff(dQ_aug,1,2)/Ts; % Aceleración angular por diferenciación

ddQ_filt(:,1) = zeros(3,1);           % ddQ_filt inicial
for k = 2:lengthColumn
    ddQ_filt(:,k) = alpha_ddQ*ddQ_unfiltered(:,k) + (1-alpha_ddQ)*ddQ_filt(:,k-1);
end
ddQ = ddQ_filt;                      % Guardamos la aceleración filtrada

%_______ POCIÓN Y VELOCIDAD EN XYZ DEL PUNTO DE TRABAJO_________
for i=1:lengthColumn
    % Posicion del punto de trabajo
    XYZ(:,i) = cin.get_posXYZ(Q(:,i),"pt");

    q_now = Q(:,i);
    dq_now = dQ(:,i);
    % Velocidad del punto de trabajo    
    dXYZ(:,i) = cin.get_Ja(q_now,"pt")*dq_now;
end

%___________ ERRORES ______________
% 1. Error de la posición angular
errQ = QD - Q;

% 2. Error del punto de trabajo en XYZ
errXYZ = PD - XYZ; 

%__________________ VOLTAJE DE ARMADURA _____________________
% VC: Voltaje de control
VA(1,:) = VC(1,:) * Gv1;
VA(2,:) = VC(2,:) * Gv2;
VA(3,:) = VC(3,:) * Gv3;

%__________________ TORQUE DE CONTROL _____________________
%  3.1. Constantes de sintonización
Kp = diag([100, 100, 100]);
Kd = diag([20, 20, 20]);

%  3.2. Objeto dinámica
dyn = RobotDinamica(ROBOT_DATA);

%  3.2. Objeto: control por cinemática inversa
CCI_R2D2 = CIK_Robot3DOF(ROBOT_DATA);

%  3.3. Torque en las juntas
for i=1:lengthColumn
    [T, ~, Tm] = CCI_R2D2.voltage2torque(VC(:,i),dQ(:,i),ddQ(:,i));
    TTm(:,i) = Tm;  % Torque en el eje del rotor
    TT(:,i) = T;    % Torque en las juntas 
end

%_____________ Corriente de control ________________
%  3.3. Corriente de armadura: Ia = TTm/Kt
IC(1,:) = TTm(1,:)/Kt1;
IC(2,:) = TTm(2,:)/Kt2;
IC(3,:) = TTm(3,:)/Kt3;
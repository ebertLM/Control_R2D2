%--------------------------------------------------%
% A. PARAMETROS DE LOS ESLABONES Y LA PIEZA DE TRABAJO
% ------------------------------------%
% Parámetros del eslabón 1
ROBOT_DATA.d1 = 157.75e-3;       %Distancia entre ejes X0 y X1 (m)
ROBOT_DATA.L1y = 23.38e-3;       %Centro de masa en Y1 (m)
ROBOT_DATA.L1z = 13.27e-3;       %Centro de masa en Z1 (m)
ROBOT_DATA.I1xx = 531947.50e-9;  %Momentos de inercia en su eje local (kg/m^2)
ROBOT_DATA.I1yy = 587232.79e-9;
ROBOT_DATA.I1zz = 395302.05e-9;
ROBOT_DATA.I1yz = 91308.90e-9;
ROBOT_DATA.m1 = 333.79e-3;        %Masa del eslabon 1 (kg)

% Parámetros del eslabón 2
ROBOT_DATA.a2 = 143.61e-3;        %Distancia entre ejes X1 y X2 (m)
ROBOT_DATA.L2x = 58.32e-3;        %Centro de masa en X2 (m)
ROBOT_DATA.L2z = 122.02e-3;       %Centro de masa en Z2 (m)
ROBOT_DATA.I2xx = 794765.96e-9;   %Momentos de inercia en su eje local (kg/m^2)
ROBOT_DATA.I2yy = 2100894.02e-9;
ROBOT_DATA.I2zz = 1407822.82e-9;
ROBOT_DATA.I2xz = -140711.29e-9;
ROBOT_DATA.m2 = 329.53e-3;        %Masa del eslabon 2 (kg)

% Parámetros del eslabón 3
ROBOT_DATA.a3 = 151.95e-3;        %Distancia entre ejes X2 y X3 (m)
ROBOT_DATA.L3x = 94.37e-3;        %Centro de masa en X3 (m)
ROBOT_DATA.L3z = 13.34e-3;        %Centro de masa en Z3 (m)
ROBOT_DATA.I3xx = 119122.40e-9;   %Momentos de inercia en su eje local(kg/m^2)
ROBOT_DATA.I3yy = 528238.71e-9; 
ROBOT_DATA.I3zz = 457951.78e-9; 
ROBOT_DATA.I3xz = -119909.83e-9; 
ROBOT_DATA.m3 = 155.26e-3;        %Masa del eslabon 3 (kg)

% Parámetros del efector final
ROBOT_DATA.a4 = 44.95e-3;         %Distancia entre ejes X3 y X4 (m)

%--------------------------------------------------%
% B. PARAMETROS DE LOS MOTORES 
%  Motor 1: CHR GM37-3429,12V,r90  110rpm, 11.51rad/s no load 
%  Motor 2: CHR GM37-3429,12V,r90 
%  Motor 3: GA25 37,12V,r74.8       115rpm,12.04rad/s no load
%  - Estamos considerando que Jm y Bm ya consideran
%   la dinamica de los engranajes
%  - Notese que estos dos parametros tambien contienen la dinamica
%   de los engranajes ya que la toma de datos se hizo con el motor
%   completo, es decir:
%       Jm = Jm + Je/(n^2)
%       Bm = Bm + Be/(n^2)
%--------------------------------------------------% 
%****Datos del motor 1****
ROBOT_DATA.Ra1 = 7.1554;        % Resistencia de armadura (ohms)
ROBOT_DATA.La1 = 4.01e-3;       % Inductancia de armadura (Henrio)
ROBOT_DATA.Kt1 = 7.252e-3;      % Constante de torque (V*s/rad)
ROBOT_DATA.Kb1 = 7.252e-3;      % Constante contraelectromotriz (V*s/rad)
ROBOT_DATA.Jm1 = 5.112e-7;      % Momento de inercia (kg*m^2)
ROBOT_DATA.Bm1 = 3.889e-6;      % Coeficinete de friccion viscosa (kg*m^2/s)
ROBOT_DATA.Im1x = 24.03777e-6;  % Momento de inercia Ix = Iy 
ROBOT_DATA.Im1z = ROBOT_DATA.Jm1;
ROBOT_DATA.mm1 = 93.93e-3;        % Masa (kg) Masa total:196g 
ROBOT_DATA.n1 = 90;             % Factor de reducción del engranaje
ROBOT_DATA.pm1y = 6.68e-3;      % Centro de masa en el eje Y  
ROBOT_DATA.pm1z = 81.30e-3;     % Centro de masa en el eje Z  

%****Datos del motor 2****
ROBOT_DATA.Ra2 = 7.1554;        % Resistencia de armadura (ohms)
ROBOT_DATA.La2 = 4.01e-3;       % Inductancia de armadura (Henrio)
ROBOT_DATA.Kt2 = 7.252e-3;      % Constante de torque (V*s/rad)
ROBOT_DATA.Kb2 = 7.252e-3;      % Constante contraelectromotriz (V*s/rad)
ROBOT_DATA.Jm2 = 5.112e-7;      % Momento de inercia (kg*m^2)
ROBOT_DATA.Bm2 = 3.889e-6;      % Coeficinete de friccion viscosa (kg*m^2/s)
ROBOT_DATA.Im2x = 24.03777e-6;  % Momento de inercia Ix 
ROBOT_DATA.Im2z = ROBOT_DATA.Jm2;
ROBOT_DATA.mm2 = 93.93e-3;      % Masa (kg) Masa total:196g 
ROBOT_DATA.n2 = 90;             % Factor de reducción del engranaje
ROBOT_DATA.pm2y = 6.68e-3;      % Centro de masa en el eje Y 
ROBOT_DATA.pm2z = 19.70e-3;     % Centro de masa en el eje Z 

%****Datos del motor 3****
ROBOT_DATA.Ra3 = 8.0462;        % Resistencia de armadura (ohms)
ROBOT_DATA.La3 = 5.37e-3;       % Inductancia de armadura (Henrio)
ROBOT_DATA.Kt3 = 5.424e-3;      % Constante de torque (V*s/rad)
ROBOT_DATA.Kb3 = 5.424e-3;      % Constante contraelectromotriz (V*s/rad)
ROBOT_DATA.Jm3 = 3.258e-7;      % Momento de inercia (kg*m^2)
ROBOT_DATA.Bm3 = 2.552e-6;      % Coeficiente de friccion viscosa (kg*m^2/s)
ROBOT_DATA.Im3x = 11366.73e-9;  % Momento de inercia Ix 
% ROBOT_DATA.Im3z = 0.67549e-6;
ROBOT_DATA.Im3z = ROBOT_DATA.Jm3;
ROBOT_DATA.mm3 = 57.60e-3;      % Masa del rotor (kg), Masa total:96g 
ROBOT_DATA.n3 = 74.8;           % Factor de reducción del engranaje
ROBOT_DATA.pm3z = 89.17e-3;     % Centro de masa en el eje Z 

%--------------------------------------------------%
% C. FACTOR DE TRANSMISION TOTAL
%--------------------------------------------------%
ROBOT_DATA.kr1 = ROBOT_DATA.n1*1;         % Solo hay caja de reduccion 
ROBOT_DATA.kr2 = ROBOT_DATA.n2*1;         % del propio motor
ROBOT_DATA.kr3 = ROBOT_DATA.n3*1;

%--------------------------------------------------%
% D. CONSTANTES DE FRICCION
%--------------------------------------------------%
ROBOT_DATA.Fc1 = 0.0135;          % Factor de Coulomb del motor 1
ROBOT_DATA.Fc2 = 0.115;           % Factor de Coulomb del motor 2
ROBOT_DATA.Fc3 = 0.040;           % Factor de Coulomb del motor 3
ROBOT_DATA.Fv1 = 0.0035;          % Constante Viscosa del motor 1
ROBOT_DATA.Fv2 = 0.004;           % Constante Viscosa del motor 2
ROBOT_DATA.Fv3 = 0.020;           % Constante Viscosa del motor 3
ROBOT_DATA.eps_fric = 0.05;       % Epsilon de la Friccion de Coulomb

%--------------------------------------------------%
% E. GANANCIAS DE LOS AMPLIFICADORES
%--------------------------------------------------%
ROBOT_DATA.Vc1_max = 3.3;                % Vc maximo motor 1
ROBOT_DATA.Vc2_max = 3.3;                % Vc maximo motor 2
ROBOT_DATA.Vc3_max = 3.3;                % Vc maximo motor 3
ROBOT_DATA.Gv1 = 12/ROBOT_DATA.Vc1_max;  % Amplificador motor 1
ROBOT_DATA.Gv2 = 12/ROBOT_DATA.Vc2_max;  % Amplificador motor 2
ROBOT_DATA.Gv3 = 12/ROBOT_DATA.Vc3_max;  % Amplificador motor 3

%--------------------------------------------------%
% F. LIMITES DE LAS JUNTAS
%--------------------------------------------------%
ROBOT_DATA.q1_max =  2*pi/3;  % Primera junta
ROBOT_DATA.q1_min = -2*pi/3;
ROBOT_DATA.q2_max =  5*pi/6;  % Segunda junta
ROBOT_DATA.q2_min =    0;
ROBOT_DATA.q3_max =   0;      % Tercera junta
ROBOT_DATA.q3_min =  -5*pi/6; 
% Velocidades y aceleraciones
% Motor 1,2: CHR GM37-3429,12V,r90  110rpm,11.51rad/s no load 
%  Motor 3: GA25 37,12V,r74.8       115rpm,12.04rad/s no load
ROBOT_DATA.dq1_max =  9;   % Velocidad máxima Motor 1 (rad/s)
ROBOT_DATA.dq2_max =  9;   % Velocidad máxima Motor 2 (rad/s)   
ROBOT_DATA.dq3_max =  10;  % Velocidad máxima Motor 3 (rad/s)
ROBOT_DATA.ddq1_max = 40;  % Aceleración máxima Motor 1 (rad/s)
ROBOT_DATA.ddq2_max = 40;  % Aceleración máxima Motor 2 (rad/s)
ROBOT_DATA.ddq3_max = 40;  % Aceleración máxima Motor 3 (rad/s)

%--------------------------------------------------%
% G. TORQUES MAXIMOS
%--------------------------------------------------%
ROBOT_DATA.T1_max = 3.5;     % Motor 1
ROBOT_DATA.T2_max = 3.5;     % Motor 2
ROBOT_DATA.T3_max = 0.98;    % Motor 3
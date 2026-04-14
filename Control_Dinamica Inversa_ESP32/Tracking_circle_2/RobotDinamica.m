classdef RobotDinamica < handle
  % Cálculo de las matrices dinámicas: D, C, G y Fv

  properties (Access = private)
    ROBOT_DATA  % struct con parámetros del robot
    g_ = 9.81;               % aceleracion de la gravedad
  end

  % methods
  %   function this = RobotDinamica(data)
  %     this.ROBOT_DATA = data;
  %   end
  % 
  %   function D = get_D(this, q)
  %     D = this.matriz_D(q);
  %   end
  % 
  %   function C = get_C(this, q, dq)
  %     C = this.matriz_C(q, dq);
  %   end
  % 
  %   function G = get_G(this, q)
  %     G = this.matriz_G(q);
  %   end
  % 
  %   function Fv = get_Fv(this)
  %     Fv = this.matriz_Fv();
  %   end
  % end

  methods
    function this = RobotDinamica(data)
      this.ROBOT_DATA = data;  
    end

    function D = get_D(this, q)
        %--------------------------------------------------------%
        % 1. PARAMETROS DEL ROBOT
        %--------------------------------------------------------%     
        a2 = this.ROBOT_DATA.a2;
        a3 = this.ROBOT_DATA.a3;
        m1 = this.ROBOT_DATA.m1;         %Masas de los eslabones
        m2 = this.ROBOT_DATA.m2;
        m3 = this.ROBOT_DATA.m3;
        %L1y = ROBOT_DATA.L1y;       %Centros de masas de los eslabones
        L1z = this.ROBOT_DATA.L1z;
        L2x = this.ROBOT_DATA.L2x;
        L2z = this.ROBOT_DATA.L2z;
        L3x = this.ROBOT_DATA.L3x;
        L3z = this.ROBOT_DATA.L3z;
        %mm1 = this.ROBOT_DATA.mm1;       %Masas de los motores
        mm2 = this.ROBOT_DATA.mm2;
        mm3 = this.ROBOT_DATA.mm3;
        %pm1y = this.ROBOT_DATA.pm1y;     %Centros de masas de los motores
        %pm1z = this.ROBOT_DATA.pm1z;
        %pm2y = this.ROBOT_DATA.pm2y;
        pm2z = this.ROBOT_DATA.pm2z;
        pm3z = this.ROBOT_DATA.pm3z;
        kr1 = this.ROBOT_DATA.kr1;       %Factores de trasnmisiòn de motores
        kr2 = this.ROBOT_DATA.kr2;
        kr3 = this.ROBOT_DATA.kr3;
        %I1xx = this.ROBOT_DATA.I1xx;     %Momentos inercia - eslabòn 1
        I1yy = this.ROBOT_DATA.I1yy;
        %I1zz = this.ROBOT_DATA.I1zz;
        %I1yz = this.ROBOT_DATA.I1yz;
        I2xx = this.ROBOT_DATA.I2xx;     %Momentos inercia - eslabòn 2
        I2yy = this.ROBOT_DATA.I2yy;
        I2zz = this.ROBOT_DATA.I2zz;
        I2xz = this.ROBOT_DATA.I2xz;
        I3xx = this.ROBOT_DATA.I3xx;     %Momentos inercia - eslabòn 3
        I3yy = this.ROBOT_DATA.I3yy;
        I3zz = this.ROBOT_DATA.I3zz;
        I3xz = this.ROBOT_DATA.I3xz;
        Im1z = this.ROBOT_DATA.Im1z;     %Momentos de inercia - motor 1
        %Im1x = this.ROBOT_DATA.Im1x;
        Im2z = this.ROBOT_DATA.Im2z;     %Momentos de inercia - motor 2
        Im2x = this.ROBOT_DATA.Im2x;
        Im3z = this.ROBOT_DATA.Im3z;     %Momentos de inercia - motor 3
        Im3x = this.ROBOT_DATA.Im3x;
        
        %--------------------------------------------------%
        % 2. LEEMOS ENTRADAS
        %--------------------------------------------------%
        %q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        
        %--------------------------------------------------%
        % 3. CONSTANTES Y VARIABLES
        %--------------------------------------------------%
        La2 = a2 - L2x;
        La3 = a3 - L3x;
        c2 = cos(q2);
        s2 = sin(q2);
        c3 = cos(q3);
        c23 = cos(q2+q3);
        s23 = sin(q2+q3);
        
        %--------------------------------------------------%
        % 5. MATRIZ D -> MATRIZ DE INERCIAS
        %--------------------------------------------------%
        D11 = I1yy + I2yy + I3yy + Im2x  +  Im3x  + ...
              m1*L1z^2  +  m2*(La2^2*c2^2+L2z^2) + ...
              m3*(L3z^2+(La3*c23+a2*c2)^2)  +  mm2*pm2z^2  +...
              mm3*(pm3z^2+a2^2*c2^2)  + ...
              (I2xx-I2yy)*s2^2  +  (I3xx-I3yy)*s23^2  +  Im1z*kr1^2;
        D12 = I3xz*s23  +  I2xz*s2  -  m2*L2z*La2*s2  -  m3*L3z*(La3*s23+a2*s2)  - ...
              mm3*a2*pm3z*s2;
        D13 = (I3xz-m3*L3z*La3)*s23;
        
        D21 = D12;
        D22 = I2zz  +  I3zz  +  Im3z  +  Im2z*kr2^2  +  ...
              mm3*a2^2  +  m2*La2^2  + m3*(La3^2+a2^2+2*a2*La3*c3);
        D23 = I3zz  +  Im3z*kr3  +  m3*La3*(La3+a2*c3);
        
        D31 = D13;
        D32 = D23;
        D33 = I3zz  +  Im3z*kr3^2  +  m3*La3^2;
        
        D = [D11 D12 D13;
             D21 D22 D23;
             D31 D32 D33]; 
    end

    function C = get_C(this, q, dq)
        %--------------------------------------------------%
        % 1. LECTURA DE LOS PARÁMETROS DEL ROBOT
        %--------------------------------------------------%
        a2 = this.ROBOT_DATA.a2;
        a3 = this.ROBOT_DATA.a3;
        m2 = this.ROBOT_DATA.m2;
        m3 = this.ROBOT_DATA.m3;
        L2x = this.ROBOT_DATA.L2x;
        L2z = this.ROBOT_DATA.L2z;
        L3x = this.ROBOT_DATA.L3x;
        L3z = this.ROBOT_DATA.L3z;
        mm3 = this.ROBOT_DATA.mm3;
        pm3z = this.ROBOT_DATA.pm3z;
        I2xx = this.ROBOT_DATA.I2xx;     %Momentos inercia - eslabòn 2
        I2yy = this.ROBOT_DATA.I2yy;
        I2xz = this.ROBOT_DATA.I2xz;
        I3xx = this.ROBOT_DATA.I3xx;     %Momentos inercia - eslabòn 3
        I3yy = this.ROBOT_DATA.I3yy;
        I3xz = this.ROBOT_DATA.I3xz;
        
        %--------------------------------------------------%
        % 2. LEEMOS ENTRADAS
        %--------------------------------------------------%
        %q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        dq1 = dq(1);
        dq2 = dq(2);
        dq3 = dq(3);
        
        %--------------------------------------------------%
        % 3. CONSTANTES Y VARIABLES
        %--------------------------------------------------%
        La2 = a2  -  L2x;
        La3 = a3  -  L3x;
        k1 = (I3xx-I3yy-m3*La3^2)/2;
        k2 = I2xx  -  I2yy  -  m2*La2^2  -  a2^2*(m3+mm3);
        k3 = m3*a2*La3;
        k4 = I3xz  -  m3*L3z*La3;
        var1 = sin(2*q2+2*q3);
        var2 = sin(2*q2+q3);
        var3 = sin(2*q2);
        c2 = cos(q2);
        s3 = sin(q3);
        c23 = cos(q2+q3);
        %--------------------------------------------------%
        % 4. MATRIZ C -> MATRIZ DE CORIOLIS
        %--------------------------------------------------%
        C11 = dq2*(var1*k1-var2*k3+var3*k2/2)  +  dq3*(var1*k1-k3*(var2+s3)/2);
        C12 = dq1*(var1*k1-var2*k3+var3*k2/2)  +  ...
              dq2*(k4*c23+(I2xz-m3*L3z*a2-m2*L2z*La2-a2*mm3*pm3z)*c2)  + ...
              dq3*k4*c23;
        C13 = dq1*(var1*k1-k3*(var2+s3)/2)  +  dq2*k4*c23  +  dq3*k4*c23;
        
        C21 = -dq1*(var1*k1-var2*k3+var3*k2/2);
        C22 = -dq3*k3*s3;
        C23 = -(dq2+dq3)*k3*s3;
        
        C31 = -dq1*(var1*k1-k3*(var2+s3)/2);
        C32 = dq2*k3*s3;
        C33 = 0;
        
        C = [C11 C12 C13;
             C21 C22 C23;
             C31 C32 C33];
    end

    function G = get_G(this, q)
        %--------------------------------------------------------%
        % 1. LECTURA DE LOS PARÁMETROS DEL ROBOT
        %--------------------------------------------------------%
        a2 = this.ROBOT_DATA.a2;
        a3 = this.ROBOT_DATA.a3;
        L2x = this.ROBOT_DATA.L2x;
        L3x = this.ROBOT_DATA.L3x;
        m2  = this.ROBOT_DATA.m2;
        m3  = this.ROBOT_DATA.m3;
        mm3 = this.ROBOT_DATA.mm3;
        g = this.g_;
        
        %--------------------------------------------------%
        % 2. LECTURA DE ENTRADAS
        %--------------------------------------------------%
        %q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        
        %--------------------------------------------------%
        % 3. CONSTANTES Y VARIABLES
        %--------------------------------------------------%
        La2 = a2 - L2x;
        La3 = a3 - L3x;
        % g = 9.81;               % aceleracion de la gravedad
        c2 = cos(q2);
        c23 = cos(q2+q3);
        
        %--------------------------------------------------%
        % 4. VECTOR DE GRAVEDAD 
        %--------------------------------------------------%
        G1 = 0;
        G2 = g*m3*(a2*c2+La3*c23)  +  g*m2*La2*c2  +  g*mm3*a2*c2;
        G3 = g*m3*La3*c23;
        G = [G1;G2;G3];
    end

    function Fv = get_Fv(this)
        %--------------------------------------------------%
        % 3. CONSTANTES Y VARIABLES
        %--------------------------------------------------%
        Bm1 = this.ROBOT_DATA.Bm1;
        Bm2 = this.ROBOT_DATA.Bm2;
        Bm3 = this.ROBOT_DATA.Bm3;
        kr1 = this.ROBOT_DATA.kr1;
        kr2 = this.ROBOT_DATA.kr2;
        kr3 = this.ROBOT_DATA.kr3;
        
        %--------------------------------------------------%
        % 4. VECTOR DE GRAVEDAD 
        %--------------------------------------------------%
        Fv = diag([Bm1*kr1^2, Bm2*kr2^2, Bm3*kr3^2]);
    end

    function ddq = dinamica_directa(this, u, q, dq)
        % Funcion para hallar la dinamica directa del manipulador
        %
        % SALIDA
        %  ddq: Vector de aceleraciones [ddq1;ddq2;...;ddqn]
        %
        % ENTRADAS
        %   u: Torque [u1;u2;...;un]
        %   q: Vector de configuracion [q1;q2;...;qn]
        %  dq: Vector de velocidades [dq1;dq2;...;dqn]
        
        
        %--------------------------------------------------------%
        % 2. CALCULO DE LAS MATRICES
        %--------------------------------------------------------%
        %  2.1. MATRIZ DE INERCIA "D"
        D = this.get_D(q);
        %  2.2. MATRIZ DE CORIOLIS "C"
        C = this.get_C(q, dq);
        %  2.3. VECTOR DE GRAVEDAD "g"
        G = this.get_G(q);        
        %  2.4. FRICCION
        Fv = this.get_Fv;
        
        %--------------------------------------------------------%
        % 3. RESPUESTA DEL ROBOT (Hallamos la 
        %    aceleracion debido al torque)
        %--------------------------------------------------------%
        ddq = (D^(-1))*(u - C*dq - G - Fv*dq);
        
        %--------------------------------------------------------%
        % 4. LIMITES EN LA ACELERACION
        %--------------------------------------------------------%
        ddq = this.limitAccelQ(ddq);
    end    
  end

  methods (Access = private)
    function ddq_lim = limitAccelQ(this, ddq)
      ddq_max = [this.ROBOT_DATA.ddq1_max; 
                 this.ROBOT_DATA.ddq2_max; 
                 this.ROBOT_DATA.ddq3_max];
      ddq_min = -ddq_max;      
      ddq_lim = min(max(ddq, ddq_min), ddq_max);
    end    
  end  
end

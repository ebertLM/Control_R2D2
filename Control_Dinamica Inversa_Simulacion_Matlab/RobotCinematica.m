classdef RobotCinematica < handle
  % Cinemática directa, inversa y Jacobianos

  properties
    ROBOT_DATA  % struct con parámetros del robot
  end

  methods
    function this = RobotCinematica(data)
        this.ROBOT_DATA = data;  
    end

    function pva = pt2ef(this, pd_pt, dpd_pt, ddpd_pt)
        % Funcion para hallar la posición, velocidad y aceleración del 
        % efector final del robot de 3GDL a partir de la posición, 
        % velocidad y aceleración del punto final de la pieza de trabajo.
        %
        % ENTRADAS:
        % pd_pt: Posición deseada de la pieza de trabajo [p1;p2;p3]
        % dpd_pt: Velocidad deseada de la pieza de trabajo [dp1;dp2;dp3]
        % ddpd_pt: Aceleración deseada de la pieza de trabajo [ddp1;ddp2;ddp3]
        %
        % SALIDAS:
        % pd_ef: Posición deseada del efector final [p1;p2;p3]
        % dpd_ef: Velocidad deseada del efector final [dp1;dp2;dp3]
        % ddpd_ef: Aceleración deseada del efector final [ddp1;ddp2;ddp3] 
        
        % Se halla la posición del efector final del robot 3GDL a partir de:
        qd = this.get_posQ(pd_pt,"pt");
        
        pd_ef = this.get_posXYZ(qd,"ef");
        
        % Se halla la velocidad del efector final del robot 3GDL a partir de:
        % V_ef = Ja_ef * dqd;            donde:
        % dqd = inv(Ja_pt) * V_pt;
        dqd = this.get_Ja_inv(qd,"pt") * dpd_pt;
        dpd_ef = this.get_Ja(qd,"ef") * dqd;
        
        % Se halla la aceleración del efector final del robot 3GDL a partir de:
        % Acc = dJa * dqd + Ja * ddq;            
        ddqd = this.get_Ja_inv(qd,"pt") * (ddpd_pt - this.get_dJa(qd, dqd,"pt") * dqd);
        ddpd_ef = this.get_dJa(qd, dqd,"ef") * dqd + this.get_Ja(qd,"ef") * ddqd; 

        pva = struct('pd',pd_ef, 'dpd',dpd_ef, 'ddpd',ddpd_ef,...
                     'qd',qd,    'dqd',dqd,    'ddqd',ddqd);
    end

    function q = get_posQ(this, XYZ, punto_final)
        % Cinematica inversa para el punto final de la 
        % pieza de trabajo
        % Configuración codo abajo
        % SALIDAS
        %     q: Configuracion de las juntas
        %  FLAG: 0(Todo OK), 1(Error)
        FLAG = 0;
        
        %--------------------------------------------------------%
        % 1. LONGITUDES DEL BRAZO
        %--------------------------------------------------------%
        d1 = this.ROBOT_DATA.d1;
        a2 = this.ROBOT_DATA.a2;
        a3 = this.ROBOT_DATA.a3;
        a4 = this.ROBOT_DATA.a4;

        % —————— Lectura de la entrada ——————
        x = XYZ(1); y = XYZ(2); z = XYZ(3); 

        %--------------------------------------------------------%
        % 2. PUNTO FINAL DESEADO
        %--------------------------------------------------------%            
        if strcmp(punto_final,"ef")
            l = a3;
        elseif strcmp(punto_final,"pt")
            l = a3 + a4;
        end
        
        %--------------------------------------------------------%
        % 2. CINEMATICA INVERSA
        %--------------------------------------------------------%
        %  3.1. EMPEZAMOS CON "q1=theta1"
        q1 = atan2(y,x);
        %  3.2. SEGUIMOS CON "q3=theta3"
        %   -> Hallamos "D"
        s  = z - d1; 
        r2 = x^2 + y^2;
        r  = sqrt(r2); 
        D  = (r2 + s^2 - (a2^2 + l^2))/(2*a2*l);
        if(abs(D)>1)
            FLAG = 1;
            RobotCinematica.Error_WorkSpace(FLAG);
            q1 = q1;
            q2 = NaN; % -100
            q3 = NaN; % -100
            return
        end
        %   -> Hallamos "q3"
        q3 = atan2(-sqrt(1-D*D), D);
        %  3.3. SEGUIMOS CON "q2=theta2"
        alfa = atan2(s,r);
        beta = atan2(l*sin(q3), a2+l*cos(q3));
        q2   = alfa - beta;
        q = [q1;q2;q3];
    end

    function XYZ = get_posXYZ(this, q, punto_final)
        % get_posXYZ Cinematica directa operacional
        % ef: Efector final del robot 3GDL
        % pt: Pieza de trabajo (punto final)
        
        %--------------------------------------------------------%
        % 1. LEEMOS PARAMETROS
        %--------------------------------------------------------%
        d1 = this.ROBOT_DATA.d1;
        a2 = this.ROBOT_DATA.a2;
        a3 = this.ROBOT_DATA.a3;
        a4 = this.ROBOT_DATA.a4;
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        
        %--------------------------------------------------------%
        % 2. PUNTO FINAL DESEADO
        %--------------------------------------------------------%            
        if strcmp(punto_final,"ef")
            l = a3;
        elseif strcmp(punto_final,"pt")
            l = a3 + a4;
        end            
        
        %--------------------------------------------------------%
        % 3. CINEMATICA OPERACIONAL
        %--------------------------------------------------------%
        x = cos(q1)*(a2*cos(q2) + l*cos(q2+q3));
        y = sin(q1)*(a2*cos(q2) + l*cos(q2+q3));
        z = d1 + a2*sin(q2) + l*sin(q2+q3);
        XYZ  = [x;y;z];
    end

    function Ja = get_Ja(this, q, punto_final)
        % Jacobiano analitico
        
        %--------------------------------------------------------%
        % 1. LEEMOS PARAMETROS
        %--------------------------------------------------------%
        a2 = this.ROBOT_DATA.a2;
        a3 = this.ROBOT_DATA.a3;
        a4 = this.ROBOT_DATA.a4;
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        
        %--------------------------------------------------------%
        % 2. PUNTO FINAL DESEADO
        %--------------------------------------------------------%            
        if strcmp(punto_final,"ef")
            l = a3;
        elseif strcmp(punto_final,"pt")
            l = a3 + a4;
        end 
        
        %--------------------------------------------------------%
        % 3. JACOBIANO ANALITICO
        %--------------------------------------------------------%
        s1 = sin(q1);       s2 = sin(q2);
        c1 = cos(q1);       c2 = cos(q2);
        s23  = sin(q2+q3);  c23 = cos(q2+q3);
        
        Ja = [-s1*(l*c23 + a2*c2), -c1*(l*s23 + a2*s2), -l*s23*c1;
               c1*(l*c23 + a2*c2), -s1*(l*s23 + a2*s2), -l*s23*s1;
                        0,            l*c23 + a2*c2,       l*c23]; 
    end

    function dJa = get_dJa(this, q, dq, punto_final)
        % Derivada del Jacobiano analitico
        
        %--------------------------------------------------------%
        % 1. LEEMOS PARAMETROS
        %--------------------------------------------------------%
        a2 = this.ROBOT_DATA.a2;
        a3 = this.ROBOT_DATA.a3;   
        a4 = this.ROBOT_DATA.a4;
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        dq1 = dq(1);
        dq2 = dq(2);
        dq3 = dq(3);
        
        %--------------------------------------------------------%
        % 2. PUNTO FINAL DESEADO
        %--------------------------------------------------------%            
        if strcmp(punto_final,"ef")
            l = a3;
        elseif strcmp(punto_final,"pt")
            l = a3 + a4;
        end 
        
        %--------------------------------------------------------%
        % 3. DERIVADA DEL JACOBIANO ANALITICO
        %--------------------------------------------------------%
        %  3.1. VARIABLES
        s1   = sin(q1);      c1 = cos(q1);
        s2   = sin(q2);      c2 = cos(q2);
        s23  = sin(q2+q3);  c23 = cos(q2+q3);
        aux1 = a2*s2 + l*s23;
        aux2 = a2*c2 + l*c23;
        aux3 = l*s23*s1;
        aux4 = l*s23*c1;
        aux5 = l*c23*c1;
        aux6 = l*c23*s1;
        %  3.2. DERIVADA 
        dJa      = zeros(3,3);
        dJa(1,1) = -c1*aux2*dq1 + s1*aux1*dq2 + aux3*dq3;
        dJa(1,2) =  s1*aux1*dq1 - c1*aux2*dq2 - aux5*dq3;
        dJa(1,3) =  aux3*dq1 - aux5*dq2 - aux5*dq3;
        dJa(2,1) = -s1*aux2*dq1 - c1*aux1*dq2 - aux4*dq3;
        dJa(2,2) = -c1*aux1*dq1 - s1*aux2*dq2 - aux6*dq3;
        dJa(2,3) = -aux4*dq1 - aux6*dq2 - aux6*dq3;
        dJa(3,1) =  0;
        dJa(3,2) = -aux1*dq2 - l*s23*dq3;
        dJa(3,3) = -l*s23*(dq2 + dq3);
    end

    function Ja_inv = get_Ja_inv(this, q, punto_final)
        %GET_JA_INV   Inversa del Jacobiano analítico con chequeo de singularidad
        %
        %   Ja_inv = obj.get_Ja_inv(q, punto_final)
        %
        %   q             : vector [q1; q2; q3] en radianes
        %   punto_final   : 'ef' o 'pt'
        %   Ja_inv        : matriz inversa 3×3 del Jacobiano analítico
        %   FLAG: 0(Todo OK), 1(Error)
        
        % 1) Parámetros geométricos
        a2 = this.ROBOT_DATA.a2;
        a3 = this.ROBOT_DATA.a3;
        a4 = this.ROBOT_DATA.a4;
        
        % 2) Componentes articulares
        q1 = q(1);
        q2 = q(2);
        q3 = q(3);
        
        % 3) Chequeo de singularidad: determinante cercano a cero
        threshold = 1e-3;
        FLAG = 0;

        %detA = sin(q3) * (a2*cos(q2) + (a3+a4)*cos(q2+q3));
        detA = sin(q3) * (a2*cos(q2) + a3*cos(q2+q3));
        if abs(detA) < threshold
            FLAG = 1;
            RobotCinematica.Error_Singularity(detA, threshold, FLAG);
        end
        
        % 4) Longitud l según punto_final
        if strcmp(punto_final, 'pt')
        l = a3 + a4;
        else
        l = a3;
        end
        
        % 5) Cálculos auxiliares
        aux1 = l*cos(q2 + q3) + a2*cos(q2);
        aux2 = sin(q2 + q3) / (a2*sin(q3));
        aux3 = a2 * l * sin(q3);

        % 6) Construcción de la inversa        
        Ja_inv = [...
            -sin(q1)/aux1                   , cos(q1)/aux1                   , 0; ...
             cos(q2+q3)*cos(q1)/(a2*sin(q3)), cos(q2+q3)*sin(q1)/(a2*sin(q3)), aux2; ...
            -cos(q1)*aux1/aux3              , -sin(q1)*aux1/aux3             , -sin(q2)/(l*sin(q3))-aux2 ...
                 ];   
    end

    function verifyAngleQ(this, q)
        %   VERIFYANGLEQ Comprueba que q1,q2,q3 estén dentro de los límites.
        %   RobotCinematica.verifyAngleQ(q, limits)
        %   q      : [3×1] vector de ángulos (rad)
        %   limits : struct con campos 
        %            q1_min, q1_max, q2_min, q2_max, q3_min, q3_max
        
        % Lectura de los ángulos
        q1 = q(1);  q2 = q(2);  q3 = q(3);

        % Obtencion de los angulos límites
        q1_min_ = this.ROBOT_DATA.q1_min;
        q1_max_ = this.ROBOT_DATA.q1_max;
        q2_min_ = this.ROBOT_DATA.q2_min;
        q2_max_ = this.ROBOT_DATA.q2_max;
        q3_min_ = this.ROBOT_DATA.q3_min;
        q3_max_ = this.ROBOT_DATA.q3_max;

        ok = true;
        
        if q1 < q1_min_ || q1 > q1_max_
            fprintf('q1 fuera de rango válido (%.3f–%.3f)\n', ...
                    q1_min_, q1_max_);
            ok = false;
        end
        if q2 < q2_min_ || q2 > q2_max_
            fprintf('q2 fuera de rango válido (%.3f–%.3f)\n', ...
                    q2_min_, q2_max_);
            ok = false;
        end
        if q3 < q3_min_ || q3 > q3_max_
            fprintf('q3 fuera de rango válido (%.3f–%.3f)\n', ...
                    q3_min_, q3_max_);
            ok = false;
        end
        
        if ~ok
            error('Ángulo(s) fuera de rango: deteniendo ejecución.');
        end
    end    
  end

  methods (Static)
    function Error_WorkSpace(flag)
        if flag == 1
            error("Punto fuera del espacio de trabajo, trayectoria circular no realizable");
        end
    end 
    
    function Error_Singularity(detA, threshold, flag)
        if flag == 1
            error('Singularidad detectada: det(Jac) ≈ 0 (|%.3e| < %.3e)', detA, threshold);
        end
    end 
  end

end

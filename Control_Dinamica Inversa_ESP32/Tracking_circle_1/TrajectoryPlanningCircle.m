classdef TrajectoryPlanningCircle < handle
    % Planificación de una trayectoria circular en el espacio operativo
    properties (SetAccess = private)
        % Objeto de la clase 'RobotCinematica'
        cin RobotCinematica
        
        % Parámetros de configuración (inmutables tras la construcción)
        Ts           % tiempo de muestreo
        tadd         % tiempo adicional para reposo
        factor       % factor del perfil trapezoidal
        ds_max       % velocidad máxima del perfil
        dds_max      % aceleración máxima del perfil
        ds_c         % velocidad de crucero del perfil
        
        % Parámetros de la curva (calculados al crear el objeto)
        r            % eje unitario
        d            % punto de paso
        po           % posición inicial en el círculo
        centralAngle         % ángulo central a recorrer
        c            % centro del círculo
        rho          % radio
        R            % matriz de rotación base→mundo
        
        % Perfil trapezoidal completo
        so           % longitud inicial del arco (m)
        sf           % longitud total del arco (m)
        tf           % tiempo de navegación (s)
        tc           % tiempo de crucero (s)
    end
    
    methods
        function this = TrajectoryPlanningCircle(robotCinematicObj, r, d, po, so, centralAngle, ...
                                                 Ts, tadd, factor, ds_max, dds_max)
            % Constructor: almacena parámetros y genera toda la planificación
            this.cin = robotCinematicObj;
            this.Ts      = Ts;
            this.tadd    = tadd;
            this.factor  = factor;
            this.ds_max  = ds_max;
            this.dds_max = dds_max;
            
            % normalizamos eje y guardamos resto
            this.r    = r / norm(r);
            this.d    = d;
            this.po   = po;
            this.so   = so;
            this.centralAngle = centralAngle;
            
            % calculamos la geometría y perfil trapezoidal
            this.computeParameters();
        end
        
        function [PD, dPD, ddPD, QD, dQD, ddQD] = PosVelAcc(this)
            % Devuelve:
            %   PD: 3×N matriz de puntos {x;y;z}
            %   QD: 3×N matriz de configuraciones articulares {q1;q2;q3}
            
            td      = this.tf + this.tadd;
            
            nPoints = floor(td/this.Ts);
            PD = zeros(3, nPoints);
            dPD = zeros(3, nPoints);
            ddPD = zeros(3, nPoints);
            QD = zeros(3, nPoints);
            dQD = zeros(3, nPoints);
            ddQD = zeros(3, nPoints);
            
            for k = 1:nPoints
                t = k*this.Ts;

                % perfil trapezoidal
                [s, ds, dds] = this.profile(t);

                % Posicion, velocidad y aceleracion en XYZ
                [pd,dpd,ddpd] = this.trajectory(s, ds, dds); 

                % Posicion, velocidad y aceleracion en Q
                pvaQ = this.cin.pt2ef(pd, dpd, ddpd);

                % Guardamos
                PD(:,k) = pd;
                dPD(:,k) = dpd;
                ddPD(:,k) = ddpd; 
                QD(:,k) = pvaQ.qd;
                dQD(:,k) = pvaQ.dqd;
                ddQD(:,k) = pvaQ.ddqd;                  
            end
        end
    end
    
    % methods (Access = private)
    methods
        function computeParameters(this)
            % —————— Geometría del círculo ——————
            Y = this.po - this.d;
            if abs(dot(this.r,Y)) >= norm(Y)
                error('Parámetros del círculo inválidos');
            end
            %  Centro y radio del círculo
            this.c   = this.d + dot(Y,this.r)*this.r;
            this.rho = norm(this.po - this.c);
            this.sf  = this.centralAngle * this.rho;

            %  Matriz de rotación: sistema local → sistema inercial
            xp = (this.po - this.c)/norm(this.po - this.c);
            zp = this.r;
            yp = cross(zp, xp);
            this.R = [xp, yp, zp];
            
            % —————— Parámetros del perfil trapezoidal ——————
            dds_max_ = this.dds_max;        

            delta   = this.sf - this.so;
            ds_c_ = this.factor * this.ds_max;
            ds_c_ = sign(delta) * abs(ds_c_);

            if sign(dds_max_) ~= sign(delta)
                dds_max_ = -dds_max_;
            end

            tc_ = ds_c_ / dds_max_;
            tf_ = tc_ + delta / ds_c_;
            ds_r = abs(delta) / tf_;
            
            % ajustes por validez
            if abs(ds_c_) <= ds_r
                ds_c_ = ds_r;
                tc_  = ds_c_ / dds_max_;
                tf_  = tc_ + delta / ds_c_;
                fprintf('Velocidad de crucero muy pequeña. Corrigiendo dqc=%2.4f\n',ds_c_)
            elseif abs(ds_c_) > 2*ds_r
                ds_c_ = 2*ds_r;
                tc_  = ds_c_ / dds_max_;
                tf_  = tc_ + delta / ds_c_;
                fprintf('Velocidad de crucero muy grande. Corrigiendo dqc=%2.4f\n',ds_c_)
            end
            
            fprintf('TIEMPO DE MOVIMIENTO PLANIFICADO=%2.4f\n',tf_);
            % actualizamos los valores miembros
            this.tc   = tc_;
            this.tf = tf_;
            this.dds_max = dds_max_;
            this.ds_c = ds_c_;
        end


        function [s, ds, dds] = profile(this, t)
            % —————— Perfil trapezoidal en el tiempo ——————
            tc_  = this.tc; 
            tf_  = this.tf;
            so_  = this.so;
            dds_max_ = this.dds_max;

            if t <= tc_
                s   = so_ + 0.5 * dds_max_ * t^2;
                ds  = dds_max_ * t;
                dds = dds_max_;
            elseif t <= (tf_ - tc_)
                s   = so_ + dds_max_ * tc_ * (t - 0.5*tc_);
                ds  = dds_max_ * tc_;
                dds = 0;
            elseif t <= tf_
                dt  = tf_ - t;
                s   = this.sf - 0.5*dds_max_*dt^2;
                ds  = dds_max_ * dt;
                dds = -dds_max_;
            else
                s   = this.sf;
                ds  = 0;
                dds = 0;
            end
        end
        
        function [pd,dpd,ddpd] = trajectory(this, s, ds, dds)
            % —————— Trayectoria circular en el tiempo ——————
            sn     = s/this.rho;
            cos_sn = cos(sn);
            sin_sn = sin(sn);

            % Posición deseada 
            pp     = [this.rho*cos_sn; this.rho*sin_sn; 0];         
            pd     = this.c + this.R * pp;

            % Velocidad deseada
            dpd_local = [-ds*sin_sn; ds*cos_sn; 0];
            dpd = this.R*dpd_local;

            % Aceleración deseada
            ddpd_local = [-ds^2*cos_sn/this.rho - dds*sin_sn; ...
                          -ds^2*sin_sn/this.rho + dds*cos_sn;  
                           0];
            ddpd = this.R*ddpd_local;
        end

        function tf = get_tf(this)
            tf = this.tf;
        end
    end
end

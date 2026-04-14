classdef TrajectoryPlanningRectangle < handle
    % Planificación de una trayectoria circular en el espacio operativo
    properties (SetAccess = private)
        % Parámetros de configuración (inmutables tras la construcción)
        Ts           % tiempo de muestreo
        tadd         % tiempo adicional para reposo
        factor       % factor del perfil trapezoidal
        ds_max       % velocidad máxima del perfil
        dds_max      % aceleración máxima del perfil
       
        % Parámetros de configuración (mutables tras la construcción)
        dds_max1     % aceleración máxima del perfil-lado 1
        ds_c1        % velocidad de crucero del perfil-lado 1
        dds_max2     % aceleración máxima del perfil-lado 2
        ds_c2        % velocidad de crucero del perfil-lado 2
        dds_max3     % aceleración máxima del perfil-lado 3
        ds_c3        % velocidad de crucero del perfil-lado 3  
        dds_max4     % aceleración máxima del perfil-lado 4
        ds_c4        % velocidad de crucero del perfil-lado 4        
        
        % Parámetros de las rectas (calculados al crear el objeto)
        p1          % primer punto del rectangulo
        p2          % segundo punto del rectangulo
        p3          % tercer punto del rectangulo
        p4          % cuarto punto del rectangulo
        
        % Perfil trapezoidal completo
        so            % longitud inicial de los 4 lados (m)

        sf1           % longitud total del lado 1 (m)
        sf2           % longitud total del lado 1 (m)
        sf3           % longitud total del lado 1 (m)
        sf4           % longitud total del lado 1 (m)
        
        tf1           % tiempo de navegación del primer lado
        tf2           % tiempo de navegación del segundo lado
        tf3           % tiempo de navegación del tercer lado
        tf4           % tiempo de navegación del cuarto lado

        tc1           % tiempo de crucero del primer lado        
        tc2           % tiempo de crucero del segundo lado        
        tc3           % tiempo de crucero del tercer lado          
        tc4           % tiempo de crucero del cuarto lado        

    end
    
    methods
        function this = TrajectoryPlanningRectangle(p1_, p2_, p3_, p4_, so, Ts,...
                                                    tadd, factor, ds_max_, dds_max_)
            % Constructor: almacena parámetros y genera toda la planificación
            this.Ts      = Ts;
            this.tadd    = tadd;
            this.factor  = factor;
            this.ds_max  = ds_max_;
            this.dds_max = dds_max_;
            
            % normalizamos eje y guardamos resto
            this.p1    = p1_;
            this.p2    = p2_;
            this.p3    = p3_;
            this.p4    = p4_;
            this.so   = so;
            
            % calculamos geometría y perfil trapezoidal
            this.computeParameters();
        end
        
        function [PD, dPD, ddPD] = PosVelAcc(this)
            % Devuelve:
            %   PD: 3×N matriz de puntos {x;y;z}
            %   QD: 3×N matriz de configuraciones articulares {q1;q2;q3}
            tf_total = this.get_totalTime();
            td       = tf_total + this.tadd;

            nPoints = floor(td/this.Ts);
            PD = zeros(3, nPoints);
            dPD = zeros(3, nPoints);
            ddPD = zeros(3, nPoints);
            
            for k = 1:nPoints
                t = k * this.Ts;

                if t <= this.tf1
                    currentSide = "lado1";
                elseif t <= this.tf1 + this.tf2
                    t = t - this.tf1;
                    currentSide = "lado2";
                elseif t <= this.tf1 + this.tf2 + this.tf3
                    t = t - (this.tf1 + this.tf2);
                    currentSide = "lado3";
                elseif t <= tf_total
                    t = t - (this.tf1 + this.tf2 + this.tf3);
                    currentSide = "lado4";
                else
                    % Tiempo adicional al finalizar la trayectoria
                    t = tf_total;
                    currentSide = "lado4";
                end           
                [s,ds,dds]    = rectanglePlanner.profile(t,currentSide);
                [pd,dpd,ddpd] = rectanglePlanner.trajectory(s,ds,dds,currentSide); 

                % Guardamos
                PD(:,k) = pd;
                dPD(:,k) = dpd;
                ddPD(:,k) = ddpd; 
            end
        end

        function computeParameters(this)
            % —————— Geometría del rectángulo ——————
            % 2.1. VARIABLES
            % Longitud inicial y final del primer lado
            Do1 = this.so;
            Df1 = norm(this.p2 - this.p1);     % Longitud del segmento
            % Longitud inicial y final del segundo lado
            Do2 = this.so;
            Df2 = norm(this.p3 - this.p2);     % Longitud del segmento
            % Longitud inicial y final del tercer lado
            Do3 = this.so;
            Df3 = norm(this.p4 - this.p3);     % Longitud del segmento
            % Longitud inicial y final del cuarto lado
            Do4 = this.so;
            Df4 = norm(this.p1 - this.p4);     % Longitud del segmento
            
            % —————— Parámetros del perfil trapezoidal ——————
            % Calculamos los parámetros del perfil trapezoidal
            [ds_c1_,dds_max1_,tf1_,tc1_] = trapez_parameters(this, Do1,Df1);
            [ds_c2_,dds_max2_,tf2_,tc2_] = trapez_parameters(this, Do2,Df2);
            [ds_c3_,dds_max3_,tf3_,tc3_] = trapez_parameters(this, Do3,Df3);
            [ds_c4_,dds_max4_,tf4_,tc4_] = trapez_parameters(this, Do4,Df4);

            % Actualizamos los valores miembros
            this.ds_c1=ds_c1_; this.dds_max1=dds_max1_; this.tf1=tf1_; this.tc1=tc1_; this.sf1=Df1;
            this.ds_c2=ds_c2_; this.dds_max2=dds_max2_; this.tf2=tf2_; this.tc2=tc2_; this.sf2=Df2;
            this.ds_c3=ds_c3_; this.dds_max3=dds_max3_; this.tf3=tf3_; this.tc3=tc3_; this.sf3=Df3;
            this.ds_c4=ds_c4_; this.dds_max4=dds_max4_; this.tf4=tf4_; this.tc4=tc4_; this.sf4=Df4;

            fprintf('TIEMPO DE MOVIMIENTO PLANIFICADO=%2.4f\n',tf1_+tf2_+tf3_+tf4_);
        end

        function [ds_c_,dds_max_,tf_,tc_] = trapez_parameters(this, Do,Df)
            % —————— Parámetros del perfil trapezoidal ——————
            dds_max_ = this.dds_max;        

            delta   = Df - Do;
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
        end 


        function [s, ds, dds] = profile(this, t, side)
            % ——— Identificación paramétrica del perfil trapezoidal ————
            if strcmp(side,"lado1")
                tc_      = this.tc1;
                tf_      = this.tf1;
                sf_      = this.sf1;
                dds_max_ = this.dds_max1;      
            elseif strcmp(side,"lado2")
                tc_      = this.tc2;
                tf_      = this.tf2;
                sf_      = this.sf2;
                dds_max_ = this.dds_max2;  
            elseif strcmp(side,"lado3")
                tc_      = this.tc3;
                tf_      = this.tf3;
                sf_      = this.sf3;
                dds_max_ = this.dds_max3;
            elseif strcmp(side,"lado4")
                tc_      = this.tc4;
                tf_      = this.tf4;
                sf_      = this.sf4;
                dds_max_ = this.dds_max4;  
            else
                error('profile:unknownSide', 'Unknown side "%s".', side);                
            end 
            so_  = this.so;

            % —————— Perfil trapezoidal en el tiempo ——————
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
                s   = sf_ - 0.5*dds_max_*dt^2;
                ds  = dds_max_ * dt;
                dds = -dds_max_;
            else
                s   = sf_;
                ds  = 0;
                dds = 0;
            end
        end
        
        function [pd,dpd,ddpd] = trajectory(this, s, ds, dds, side)
            if strcmp(side,"lado1")
                po  = this.p1; 
                pf  = this.p2;  
                sf  = this.sf1;     
            elseif strcmp(side,"lado2")
                po  = this.p2; 
                pf  = this.p3;  
                sf  = this.sf2; 
            elseif strcmp(side,"lado3")
                po  = this.p3; 
                pf  = this.p4;  
                sf  = this.sf3; 
            elseif strcmp(side,"lado4")
                po  = this.p4; 
                pf  = this.p1;  
                sf  = this.sf4; 
            else
                error('profile:unknownSide', 'Unknown side "%s".', side);                
            end

            % —————— Trayectoria lineal en el tiempo ——————
            % Posición deseada 
            pd   = po + (pf - po)*(s/sf); 

            % Velocidad deseada
            dpd  = (pf - po)*(ds/sf);

            % Aceleración deseada
            ddpd = (pf - po)*(dds/sf);            
        end

        function tf1 = get_tf1(this)
            tf1 = this.tf1;
        end
        function tf2 = get_tf2(this)
            tf2 = this.tf2;
        end        
        function tf3 = get_tf3(this)
            tf3 = this.tf3;
        end 
        function tf4 = get_tf4(this)
            tf4 = this.tf4;
        end       
        function tf = get_totalTime(this)
            tf = this.tf1+this.tf2+this.tf3+this.tf4;
        end         
    end
end

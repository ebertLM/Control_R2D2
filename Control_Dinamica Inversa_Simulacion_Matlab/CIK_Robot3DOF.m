classdef CIK_Robot3DOF < handle
  % Computa el torque de control (inverse dynamics + PD)
  properties (Access = private)
    Kp            % ganancia proporcional (3×3)
    Kd            % ganancia derivativa   (3×3)
    ROBOT_DATA    % parámetros del robot
    Cinematica    % objeto RobotCinematica
    Dinamica      % objeto RobotDinamica

    % Límites de voltaje
    outMaxVolt    % [v1; v2; v3]
    outMinVolt

    % Límites de torque
    outMaxTorque  % [t1; t2; t3]
    outMinTorque
  end

  methods
    function this = CIK_Robot3DOF(Kp, Kd, data, cin, dyn)
      this.Kp         = Kp;
      this.Kd         = Kd;
      this.ROBOT_DATA = data;
      this.Cinematica = cin;
      this.Dinamica   = dyn;

      % inicializa límites a ±∞
      this.outMaxVolt   = inf(3,1);
      this.outMinVolt   = -inf(3,1);
      this.outMaxTorque = inf(3,1);
      this.outMinTorque = -inf(3,1);
    end

    function set_outputMaxVoltage(this)
      this.outMaxVolt = [this.ROBOT_DATA.Vc1_max; 
                         this.ROBOT_DATA.Vc2_max; 
                         this.ROBOT_DATA.Vc3_max];
      this.outMinVolt = -this.outMaxVolt;
    end

    function set_outputMaxTorque(this)
      this.outMaxTorque = [this.ROBOT_DATA.T1_max; 
                           this.ROBOT_DATA.T2_max; 
                           this.ROBOT_DATA.T3_max];
      this.outMinTorque = -this.outMaxTorque;
    end

    function [u_c, y] = get_torque(this, pd_pt, dpd_pt, ddpd_pt, q, dq)
      % 1) obtener pd,dpd,ddpd en efector final
      pva = this.Cinematica.pt2ef(pd_pt, dpd_pt, ddpd_pt);
      pd_ef  = pva.pd;
      dpd_ef = pva.dpd;
      ddpd_ef= pva.ddpd;
      % qd = pva.qd;
      % dqd = pva.dqd;
      % ddqd = pva.ddqd;

      % 2) Operaciondes del Jacobiano
      Ja      = this.Cinematica.get_Ja(q,'ef');
      Ja_inv  = this.Cinematica.get_Ja_inv(q,'ef');
      dJa = this.Cinematica.get_dJa(q,dq,'ef');

      % 3) Posición y velocidad actual del efector final
      p_ef  = this.Cinematica.get_posXYZ(q,'ef');
      dp_ef = Ja * dq;

      % 4) Errores
      e  = pd_ef  - p_ef;
      de = dpd_ef - dp_ef;

      % 5) Ley de control auxiliar "y"
      %    Se cumple: y = ddq
      y = Ja_inv*(ddpd_ef + this.Kd*de + this.Kp*e - dJa*dq);

      % 6) dinámica directa: u = D·u0 + C·dq + Fv·dq + G
      D  = this.Dinamica.get_D(q);
      C     = this.Dinamica.get_C(q,dq);
      G     = this.Dinamica.get_G(q);
      B_aug = this.Dinamica.get_B_aug();
      Frict = this.Dinamica.get_Frict(dq);
      Jref_rotor = this.Dinamica.get_Jref_rotor();

      % Torque de control sin la dinámica del motor 
      u = D*y + C*dq + Frict + G;

      % 7) Limitamos el torque de control 
      u_lim = this.limitTorque(u);

      % Torque de control con la dinámica del motor  
      u_c = u_lim + B_aug*dq + Jref_rotor*y;
    end

    function [Vc,Va] = get_voltage_uC(this, u)
      % Entrada:
      % u: Torque de control
      % Salida:
      % Vc: Voltaje de control
      % Va: Voltaje de armadura
      % **** Descripción:******
      % Halla Vc y Va a partir de "u"

      % Extraer constantes
      Ra1 = this.ROBOT_DATA.Ra1;
      Ra2 = this.ROBOT_DATA.Ra2;
      Ra3 = this.ROBOT_DATA.Ra3;
      Kt1 = this.ROBOT_DATA.Kt1;
      Kt2 = this.ROBOT_DATA.Kt2;
      Kt3 = this.ROBOT_DATA.Kt3;
      kr1 = this.ROBOT_DATA.kr1;
      kr2 = this.ROBOT_DATA.kr2;
      kr3 = this.ROBOT_DATA.kr3;
      Gv1 = this.ROBOT_DATA.Gv1;
      Gv2 = this.ROBOT_DATA.Gv2;
      Gv3 = this.ROBOT_DATA.Gv3;
      Gv  = diag([Gv1;Gv2;Gv3]);

      % Hallar Vc
      % u = Vc_gain * Vc -> Vc=Vc_gain^-1 * u 
      Vc = [u(1)*Ra1/(kr1*Kt1*Gv1);...
            u(2)*Ra2/(kr2*Kt2*Gv2);...
            u(3)*Ra3/(kr3*Kt3*Gv3)];

      % Saturar el Vc
      Vc = this.limitVoltage(Vc);

      % Voltaje de armadura con el Vc limitado
      Va = Gv * Vc; 
    end

    function [tau_l, Tms, Tm] = voltage2torque(this, Vc, dq, ddq)
      % Funcion para hallar el torque/fuerza en funcion 
      % del voltaje
      %
      % SALIDAS
      %   T: Torque/fuerza en la junta del robot
      % Tms: Torque en el eje de salida del motor
      %  Tm: Torque generado por el motor
      % ENTRADAS
      %   Vc: Voltaje de control del microcontrolador
      %   dq: Velocidades de las juntas
      %   ddq: Aceleraciones de las juntas

      % Extraer constantes
      Ra1 = this.ROBOT_DATA.Ra1;
      Ra2 = this.ROBOT_DATA.Ra2;
      Ra3 = this.ROBOT_DATA.Ra3;
      Kt1 = this.ROBOT_DATA.Kt1;
      Kt2 = this.ROBOT_DATA.Kt2;
      Kt3 = this.ROBOT_DATA.Kt3;
      Kb1 = this.ROBOT_DATA.Kb1;
      Kb2 = this.ROBOT_DATA.Kb2;
      Kb3 = this.ROBOT_DATA.Kb3;
      kr1 = this.ROBOT_DATA.kr1;
      kr2 = this.ROBOT_DATA.kr2;
      kr3 = this.ROBOT_DATA.kr3;
      Kr  = diag([kr1;kr2;kr3]);      
      Bm1 = this.ROBOT_DATA.Bm1;
      Bm2 = this.ROBOT_DATA.Bm2;
      Bm3 = this.ROBOT_DATA.Bm3;
      Jm1 = this.ROBOT_DATA.Jm1;      % Momento de inercia (kg*m^2)
      Jm2 = this.ROBOT_DATA.Jm2;
      Jm3 = this.ROBOT_DATA.Jm3;
      Gv1 = this.ROBOT_DATA.Gv1;
      Gv2 = this.ROBOT_DATA.Gv2;
      Gv3 = this.ROBOT_DATA.Gv3;

      %--------------------------------------------------%
      % 2. HALLAMOS LOS TORQUES
      %--------------------------------------------------%
      %  2.1. TORQUE EN EL EJE DE ROTOR: 
      %       Tm = Kt*(Gv*Vc-Kb*Kr*dq)/Ra
      Tm = [Kt1*(Gv1*Vc(1)-Kb1*kr1*dq(1))/Ra1;...
            Kt2*(Gv2*Vc(2)-Kb2*kr2*dq(2))/Ra2;...
            Kt3*(Gv3*Vc(3)-Kb3*kr3*dq(3))/Ra3];

      %  2.2. TORQUE EN EL EJE DE SALIDA DEL MOTOR: 
      %       Tms = Kr*Tm - Kr*Jm*Kr*ddq - Kr*Bm*Kr*dq 
      %       donde  si: tau_Jm = Kr*Jm*Kr*ddq 
      %                   tau_Bm = Kr*Bm*Kr*dq
      %       entonces:
      %       Tms = Kr*Tm - tau_Jm - tau_Bm
      tau_Jm = [kr1^2*Jm1*ddq(1);...
                kr2^2*Jm2*ddq(2);...
                kr3^2*Jm3*ddq(3)];
      tau_Bm = [kr1^2*Bm1*dq(1);...
                kr2^2*Bm2*dq(2);...
                kr3^2*Bm3*dq(3)];    
      Tms = Kr*Tm - tau_Jm - tau_Bm;
      %  2.3. TORQUE EN LA JUNTA DEL ESLABÓN 
      %       Debido a la acople directo entre el eje la 
      %       salida del motor y la junta, entonces:
      %       tau_l = Tms
      tau_l = Tms;
      %  2.4. límites de torque
      tau_l = this.limitTorque(tau_l); 
      Tms = tau_l;
    end
  end

  methods (Access = private)
    function u_lim = limitTorque(this, u)
      u_lim = min(max(u, this.outMinTorque), this.outMaxTorque);
    end

    function Vc_lim = limitVoltage(this, Vc)
      Vc_lim = min(max(Vc, this.outMinVolt), this.outMaxVolt);
    end   
  end

end

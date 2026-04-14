classdef CIK_Robot3DOF < handle
  % Computa el torque de control (inverse dynamics + PD)
  properties (Access = private)
    ROBOT_DATA    % parámetros del robot

    % Límites de torque
    outMaxTorque  % [t1; t2; t3]
    outMinTorque
  end

  methods
    function this = CIK_Robot3DOF(data)
      this.ROBOT_DATA = data;

      % Limita los valores de los torques realizable
      this.set_outputMaxTorque()
    end

    function set_outputMaxTorque(this)
      T1_max = this.ROBOT_DATA.T1_max;
      T2_max = this.ROBOT_DATA.T2_max;
      T3_max = this.ROBOT_DATA.T3_max;

      this.outMaxTorque = [T1_max; T2_max; T3_max];
      this.outMinTorque = -this.outMaxTorque;
    end

    function [T, Tms, Tm] = voltage2torque(this, Vc, dq, ddq)
      % Funcion para hallar el torque/fuerza en funcion 
      % del voltaje, velocidad y aceleración angular
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
      %  2.1. TORQUE EN EL EJE DE ROTOR: Tm = Kt*Ia
      %       Tm = Kt*(Gv*Vc-Kb*Kr*dq)/Ra
      Tm = [Kt1*(Gv1*Vc(1)-Kb1*kr1*dq(1))/Ra1;...
            Kt2*(Gv2*Vc(2)-Kb2*kr2*dq(2))/Ra2;...
            Kt3*(Gv3*Vc(3)-Kb3*kr3*dq(3))/Ra3];

      %  2.2. TORQUE EN EL EJE DE SALIDA DEL MOTOR: Tms 
      %       tau_Jm = Kr*Jm*Kr*ddq
      %       tau_Bm = Kr*Bm*Kr*dq
      %       Tms = Kr*Tm - tau_Jm - tau_Bm 
      tau_Jm = [kr1^2*Jm1*ddq(1);...
                kr2^2*Jm2*ddq(2);...
                kr3^2*Jm3*ddq(3)];
      tau_Bm = [kr1^2*Bm1*dq(1);...
                kr2^2*Bm2*dq(2);...
                kr3^2*Bm3*dq(3)];    
      Tms = Kr*Tm - tau_Jm - tau_Bm;

      %  2.3. TORQUE EN LA JUNTA DEL ESLABÓN 
      %       No hay elementos de transmisión entre el motor y la junta
      T = Tms;
      %  2.4. Límites de torque
      T = this.limitTorque(T);        
    end 
  end

  methods (Access = private)
    function u_lim = limitTorque(this, u)
      u_lim = min(max(u, this.outMinTorque), this.outMaxTorque);
    end
  end
end

#ifndef CIK_ROBOT3DOF_H
#define CIK_ROBOT3DOF_H

#include <BasicLinearAlgebra.h>
#include "RobotDinamica.h"
#include "RobotCinematica.h"
#include "parameters.h"

using namespace BLA;

class CIK_Robot3DOF {
private:
    // CONSTANTES DE SINTONIZACIÓN
    BLA::Matrix<3, 3> Kp, Kd;

    // OBJETOS DE LA DINÁMICA Y CINEMÁTICA DEL MOTOR
    RobotDinamica Dinamica;
    RobotCinematica Cinematica;

    // LIMITES DE VOLTAJE
    float outMaxVoltage_M1 = 0.0f; // Motor 1
    float outMinVoltage_M1 = 0.0f;
    float outMaxVoltage_M2 = 0.0f; // Motor 2
    float outMinVoltage_M2 = 0.0f;
    float outMaxVoltage_M3 = 0.0f; // Motor 3
    float outMinVoltage_M3 = 0.0f;

    // LIMITES DE TORQUE
    float outMaxTorque_M1 = 0.0f; // Motor 1
    float outMinTorque_M1 = 0.0f;
    float outMaxTorque_M2 = 0.0f; // Motor 2
    float outMinTorque_M2 = 0.0f;
    float outMaxTorque_M3 = 0.0f; // Motor 3
    float outMinTorque_M3 = 0.0f;

public:
    CIK_Robot3DOF(const BLA::Matrix<3, 3>& Kp_, const BLA::Matrix<3, 3>& Kd_)
    : Kp(Kp_), Kd(Kd_), Dinamica(), Cinematica() { }

    void set_outputMaxVoltage(const float &outMaxVoltage_M1_, 
                              const float &outMaxVoltage_M2_, 
                              const float &outMaxVoltage_M3_)
    {
      outMaxVoltage_M1 =  outMaxVoltage_M1_;
      outMinVoltage_M1 = -outMaxVoltage_M1_;
      outMaxVoltage_M2 =  outMaxVoltage_M2_;
      outMinVoltage_M2 = -outMaxVoltage_M2_;
      outMaxVoltage_M3 =  outMaxVoltage_M3_;
      outMinVoltage_M3 = -outMaxVoltage_M3_;   
    } 

    void set_outputMaxTorque(const float &outMaxTorque_M1_, 
                             const float &outMaxTorque_M2_, 
                             const float &outMaxTorque_M3_)
    {
      outMaxTorque_M1 =  outMaxTorque_M1_;
      outMinTorque_M1 = -outMaxTorque_M1_;
      outMaxTorque_M2 =  outMaxTorque_M2_;
      outMinTorque_M2 = -outMaxTorque_M2_;
      outMaxTorque_M3 =  outMaxTorque_M3_;
      outMinTorque_M3 = -outMaxTorque_M3_;    
    }

    // Calculo del torque por el algoritmo de cinemática inversa(Torque computado) para el 
    // tracking en el espacio Operacional(XYZ) 
    BLA::Matrix<3, 1> get_torque(const BLA::Matrix<3, 1> &pd_pt,
                                 const BLA::Matrix<3, 1> &dpd_pt,
                                 const BLA::Matrix<3, 1> &ddpd_pt,                                             
                                 const BLA::Matrix<3, 1> &q,
                                 const BLA::Matrix<3, 1> &dq)      
    {
        //Obtener la cinemática del efector final
        RobotCinematica::PosVelAccel pva_ef = Cinematica.pt2ef(pd_pt, dpd_pt, ddpd_pt);       
        auto pd_ef    = pva_ef.pd;
        auto dpd_ef   = pva_ef.dpd;
        auto ddpd_ef  = pva_ef.ddpd;

        // Operaciondes del Jacobiano
        auto Ja     = Cinematica.get_Ja(q, "ef");
        auto Ja_inv = Cinematica.get_Ja_inv(q, "ef");
        auto dJa    = Cinematica.get_dJa(q, dq, "ef");

        // Posición y velocidad actual del efector final
        auto p_ef   = Cinematica.get_posXYZ(q, "ef"); 
        auto dp_ef  = Ja*dq; 

        // Errores
        auto e  = pd_ef - p_ef;
        auto de = dpd_ef - dp_ef;

        // Ley de control auxiliar: y
        auto y = Ja_inv*(ddpd_ef + Kd*de + Kp*e - dJa*dq);

        // Calcular el torque de control.                         
        auto D      = Dinamica.get_D(q); 
        auto C      = Dinamica.get_C(q, dq);
        auto G      = Dinamica.get_G(q);
        auto Frict  = Dinamica.get_Frict(dq);

        auto B_aug      = Dinamica.get_B_aug();
        auto Jref_rotor = Dinamica.get_Jref_rotor();

        // Torque de control sin la dinámica del motor          
        auto u = D*y + C*dq + Frict + G;

        // Limitamos el torque de control     
        limitTorque(u);

        // Torque de control con la dinámica del motor  
        auto u_lim = u + B_aug*dq + Jref_rotor*y;

        return u_lim;
    }

/*
    // Calculo del torque por el algoritmo de cinemática inversa(Torque computado) para el 
    // tracking en el espacio Operacional(XYZ) 
    BLA::Matrix<3, 1> get_torque(const BLA::Matrix<3, 1> &pd_pt,
                                 const BLA::Matrix<3, 1> &dpd_pt,
                                 const BLA::Matrix<3, 1> &ddpd_pt,                                             
                                 const BLA::Matrix<3, 1> &q,
                                 const BLA::Matrix<3, 1> &dq)      
    {
        RobotCinematica::PosVelAccel pva_ef = Cinematica.pt2ef(pd_pt, dpd_pt, ddpd_pt);
        //Obtener pd, dpd, ddpd del efector final
        auto pd_ef = pva_ef.pd;
        auto dpd_ef = pva_ef.dpd;
        auto ddpd_ef = pva_ef.ddpd;

        // Operaciondes del Jacobiano
        auto Ja = Cinematica.get_Ja(q, "ef");
        auto Ja_inv = Cinematica.get_Ja_inv(q, "ef");
        auto dJa = Cinematica.get_dJa(q, dq, "ef");

        // Posición y velocidad actual del efector final
        auto p_ef = Cinematica.get_posXYZ(q, "ef"); 
        auto dp_ef = Ja*dq; 

        // Errores
        auto e = pd_ef - p_ef;
        auto de = dpd_ef - dp_ef;

        // Ley de control auxiliar: y
        auto y = Ja_inv*(ddpd_ef + Kd*de + Kp*e - dJa*dq);

        // Calcular el torque de control.    
        auto D_aug = Dinamica.get_D_aug(q);
        auto C = Dinamica.get_C(q, dq);
        auto B_aug = Dinamica.get_B_aug();
        auto Frict = Dinamica.get_Frict(dq);
        auto G = Dinamica.get_G(q);

        auto u = D_aug*y + C*dq + B_aug*dq + Frict + G;

        // LIMITAMOS LOS VOLTAJES DE CONTROL 
        limitTorque(u);
        return u;
    }
*/

    BLA::Matrix<3, 1> get_voltage_uC(const BLA::Matrix<3, 1>& u)
    {
      // VOLTAJES DE CONTROL
      // Motor 1
      float Vc1 = (Ra1/(kr1*Kt1*Gv1))*u(0,0);  // Voltaje de control motor 1
      // Motor 2
      float Vc2 = (Ra2/(kr2*Kt2*Gv2))*u(1,0);  // Voltaje de control motor 2
      // Motor 3
      float Vc3 = (Ra3/(kr3*Kt3*Gv3))*u(2,0);  // Voltaje de control motor 3
      
      // Limitamos el voltaje de control para el microcontrolador de cada junta
      limitVoltage(Vc1, Vc2, Vc3);
      Matrix<3, 1> Vc = {Vc1, Vc2, Vc3};
      return Vc;
    }    

    // Limitamos el torque de control para cada motor
    void limitTorque(BLA::Matrix<3, 1> &u)
    {
      // u(0); // Torque de la junta 1
      // u(1); // Torque de la junta 2
      // u(2); // Torque de la junta 3
      u(0) = setLimit(u(0), outMinTorque_M1, outMaxTorque_M1);
      u(1) = setLimit(u(1), outMinTorque_M2, outMaxTorque_M2);
      u(2) = setLimit(u(2), outMinTorque_M3, outMaxTorque_M3);
    }  

    // Limitamos el voltaje de control del microcontrolador 
    // para cada motor
    void limitVoltage(float &Vc1, float &Vc2, float &Vc3)
    {
      Vc1 = setLimit(Vc1, outMinVoltage_M1, outMaxVoltage_M1);
      Vc2 = setLimit(Vc2, outMinVoltage_M2, outMaxVoltage_M2);
      Vc3 = setLimit(Vc3, outMinVoltage_M3, outMaxVoltage_M3);
    }

    // Función para limitar un float a [low_value, high_value]
    static inline float setLimit(float x, float low, float high) 
    {
      return (x < low) ? low : ((x > high) ? high : x);
    }
};

#endif
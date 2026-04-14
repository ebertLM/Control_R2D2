#ifndef ROBOTCINEMATICA_H
#define ROBOTCINEMATICA_H

#include <BasicLinearAlgebra.h>
#include "parameters.h"
#include <cstring>               // para strcmp
#include <math.h>
#include <esp_system.h>          // para ESP.restart()

using namespace BLA;

class RobotCinematica {
public:

    struct PosVelAccel
    {
      BLA::Matrix<3,1> pd, dpd, ddpd;
    };

    // Cálculo de la cinemática directa para una posicion angular "q"
    // Función para calcular las coordenadas XYZ del efector final
    // a partir de las posiciones angulares q1, q2, q3
    BLA::Matrix<3, 1> get_posXYZ(const BLA::Matrix<3, 1> &q, const char* punto_final) const
    {
      // Leemos la entrada de las posiciones en Q
      float q1 = q(0), q2 = q(1), q3 = q(2);
      float s1 = sinf(q1), c1 = cosf(q1);
      float s2 = sinf(q2), c2 = cosf(q2);
      float s23 = sinf(q2+q3), c23 = cosf(q2+q3);

      float l = (std::strcmp(punto_final, "pt") == 0 ? a3 + a4 : a3);

      Matrix<3, 1> posXYZ = {
                              c1*(l*c23 + a2*c2),
                              s1*(l*c23 + a2*c2),
                              d1 + l*s23 + a2*s2
                            };      
      return posXYZ;
    }

    // Función para calcular la matriz Jacobiano analítico
    BLA::Matrix<3, 3> get_Ja(const BLA::Matrix<3, 1> &q, const char* punto_final) const
    {
      // Leemos la entrada de las posiciones en Q
      float q1 = q(0), q2 = q(1), q3 = q(2);

      float s1 = sinf(q1), c1 = cosf(q1);
      float s2 = sinf(q2), c2 = cosf(q2);
      float s23 = sinf(q2+q3), c23 = cosf(q2+q3);

      float l = (std::strcmp(punto_final, "pt") == 0 ? a3 + a4 : a3);

      Matrix<3, 3> Ja;
      // Matriz Jacobiano analítico
      Ja(0, 0) = -s1*(l*c23 + a2*c2);
      Ja(0, 1) = -c1*(l*s23 + a2*s2);
      Ja(0, 2) = -l*s23*c1;
      Ja(1, 0) = c1*(l*c23 + a2*c2);
      Ja(1, 1) = -s1*(l*s23 + a2*s2);
      Ja(1, 2) = -l*s23*s1;
      Ja(2, 0) = 0;
      Ja(2, 1) = l*c23 + a2*c2;
      Ja(2, 2) = l*c23;
      return Ja;
    }    

    // Función para calcular la matriz de la derivada del Jacobiano analítico
    BLA::Matrix<3, 3> get_dJa(const BLA::Matrix<3, 1> &q, 
                              const BLA::Matrix<3, 1> &dq, 
                              const char* punto_final) const
    {
      // Leemos la entrada de las posiciones en Q
      float q1 = q(0), q2 = q(1), q3 = q(2),
            dq1 = dq(0), dq2 = dq(1), dq3 = dq(2);

      float l = (std::strcmp(punto_final, "pt") == 0 ? a3 + a4 : a3);

      float s1 = sinf(q1), c1 = cosf(q1),
            s2 = sinf(q2), c2 = cosf(q2),
            s23 = sinf(q2+q3), c23 = cosf(q2+q3);

      float aux1 = l*s23 + a2*s2,
            aux2 = l*c23 + a2*c2,
            aux3 = l*s23*s1,
            aux4 = l*s23*c1,
            aux5 = l*c23*c1,
            aux6 = l*c23*s1;  

      // 2. CALCULO DEL JACOBIANO 
      Matrix<3, 3> dJa;
      dJa(0, 0) = dq2*s1*aux1 - dq1*c1*aux2 + aux3*dq3;
      dJa(0, 1) = dq1*s1*aux1 - dq2*c1*aux2 - aux5*dq3;
      dJa(0, 2) = aux3*dq1 - aux5*dq3 - aux5*dq2;
      dJa(1, 0) = -dq1*s1*aux2 - dq2*c1*aux1 - aux4*dq3;
      dJa(1, 1) = -dq2*s1*aux2 - dq1*c1*aux1 - aux6*dq3;
      dJa(1, 2) = -aux4*dq1 - aux6*dq2 - aux6*dq3;
      dJa(2, 0) = 0;
      dJa(2, 1) = -dq2*aux1 - l*dq3*s23;
      dJa(2, 2) = -l*s23*(dq2 + dq3);
      return dJa;
    }

    BLA::Matrix<3,3> get_Ja_inv(const BLA::Matrix<3,1> &q, const char* punto_final) const
    {
      float threshold = 1e-5;
      // Leemos la entrada de las posiciones en Q
      float q1 = q(0), q2 = q(1), q3 = q(2);

      // Calculamos el determinante de la matriz
      int error = 0;

      // Verificamos si el determinante es casi cero, proximo a un punto singular
      float det_A = sinf(q3) * (a2*cosf(q2) + (a3+a4)*cosf(q2+q3));
      if (fabsf(det_A) < threshold) 
      {
        error = 1;
        Error_Singularity(error);
      } 
      else 
      {
        float l = (std::strcmp(punto_final, "pt") == 0 ? a3 + a4 : a3);

        // 3) auxiliares
        float aux1 = l*cosf(q2 + q3) + a2*cosf(q2),
              aux2 = sinf(q2 + q3) / (a2*sinf(q3)),
              aux3 = a2*l*sinf(q3);

        // 4) construir la inversa
        Matrix<3,3> A_inv;
        A_inv(0,0) = -sinf(q1) / aux1;
        A_inv(0,1) =  cosf(q1) / aux1;
        A_inv(0,2) =  0.0f;

        A_inv(1,0) =  cosf(q2+q3)*cosf(q1) / (a2*sinf(q3));
        A_inv(1,1) =  cosf(q2+q3)*sinf(q1) / (a2*sinf(q3));
        A_inv(1,2) =  aux2;

        A_inv(2,0) = -cosf(q1) * aux1 / aux3;
        A_inv(2,1) = -sinf(q1) * aux1 / aux3;
        A_inv(2,2) = -sinf(q2) / (l*sinf(q3)) - aux2;

        return A_inv;
      }
    }

    //get_Q: Calcula la cinemática inversa de un punto pd dentro
    //       del espacio de trabajo
    //Retorna error: si error = 0, NO HAY NINGÚN ERROR
    //               si error = 1, ERROR ENCONTRADO
    BLA::Matrix<3,1> get_posQ(const Matrix<3,1> &pd, const char* punto_final) const
    {
      float x = pd(0), y = pd(1), z = pd(2);
      
      // 1) Selecciona l según punto_final
      float l = (std::strcmp(punto_final, "pt") == 0 ? a3 + a4 : a3);

      // 2) q1
      float q1 = atan2f(y, x);

      // 3) calcular D para q3
      float s   = z - d1,
            r2  = x*x + y*y,
            D   = (r2 + s*s - (a2*a2 + l*l)) / (2.0f * a2 * l);

      if (fabsf(D) > 1.0f) {
        int error = 1;
        Error_WorkSpace(error); //Manejo del error
      }

      // 4) q3 (elige la solución “codo abajo”)
      float q3 = atan2f(-sqrtf(1.0f - D*D), D);

      // 5) q2
      float alfa = atan2f(s, sqrtf(r2)),
            beta = atan2f(l * sinf(q3), a2 + l * cosf(q3)),
            q2   = alfa - beta;

      // 6) devolver resultados
      BLA::Matrix<3,1> q = {q1, q2, q3};
      return q;
    }

    PosVelAccel pt2ef(const BLA::Matrix<3, 1> &pd_pt,
                      const BLA::Matrix<3, 1> &dpd_pt,
                      const BLA::Matrix<3, 1> &ddpd_pt) const
    {
      PosVelAccel pva;

      Matrix<3,1> qd = get_posQ(pd_pt, "pt"); // qd guarda su valor por referencia

      BLA::Matrix<3, 1> pd_ef = get_posXYZ(qd, "ef");

      BLA::Matrix<3, 1> dqd = get_Ja_inv(qd, "pt") * dpd_pt;
      BLA::Matrix<3, 1> dpd_ef = get_Ja(qd, "ef") * dqd;

      BLA::Matrix<3, 1> ddqd = get_Ja_inv(qd, "pt") * (ddpd_pt - get_dJa(qd, dqd, "pt")*dqd);
      BLA::Matrix<3, 1> ddpd_ef = get_dJa(qd, dqd, "ef")*dqd + get_Ja(qd, "ef")*ddqd;

      pva.pd = pd_ef;
      pva.dpd = dpd_ef;
      pva.ddpd = ddpd_ef;
      return pva;
    }

    static void verifyAngleQ(const Matrix<3,1>& q)
    {
      // Leemos la entrada de las posiciones en Q
      float q1 = q(0), q2 = q(1), q3 = q(2);

      // Verificar rangos
      bool ok = true;
      if (q1 < q1_min || q1 > q1_max) {
        //Serial.println("q1 fuera de rango válido...");
        //delay(100);        // Espera a que el mensaje se envíe
        ok = false;
      }
      if (q2 < q2_min || q2 > q2_max) {
        //Serial.println("q2 fuera de rango válido...");
        //delay(100);        // Espera a que el mensaje se envíe
        ok = false;
      }
      if (q3 < q3_min || q3 > q3_max) {
        //Serial.println("q3 fuera de rango válido...");
        //delay(100);        // Espera a que el mensaje se envíe
        ok = false;
      }

      // Si cualquiera falla, reinicia
      if (!ok) {
        Serial.println("Reiniciando...");
        delay(2);        // Espera a que el mensaje se envíe
        ESP.restart();     // Reinicia el ESP32
      }
    }
    
    // Manejo de error para puntos fuera del espacio de trabajo
    void Error_WorkSpace(int &error) const
    {
      if (error == 1) {
        // Error detectado: Punto singular y reinicia el ESP32
        Serial.println("Punto fuera del espacio de trabajo, reiniciando...");
        delay(2);        // Espera a que el mensaje se envíe
        ESP.restart();
      }
      // Si error == 0, no hacemos nada y el programa sigue normalmente
    }

    // Manejo de error para puntos singulares
    void Error_Singularity(int &error) const
    {
      if (error == 1) {
        // Error detectado: Punto singular y reinicia el ESP32
        Serial.println("Punto singular, reiniciando...");
        delay(2);        // Espera a que el mensaje se envíe
        ESP.restart();
      }
      // Si error == 0, no hacemos nada y el programa sigue normalmente
    }    
};

#endif
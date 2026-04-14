#ifndef ROBOTDINAMICA_H
#define ROBOTDINAMICA_H

#include <BasicLinearAlgebra.h>
#include "parameters.h"
#include <math.h>

using namespace BLA;

class RobotDinamica {

public:
    // Función para calcular la matriz D
    BLA::Matrix<3, 3> get_D(const BLA::Matrix<3, 1> &q) 
    {
      // Variables auxiliare
      float q2 = q(1), q3 = q(2);

      float La2x = a2 - L2x,
            La3x = a3 - L3x;
      float s2 = sinf(q2), c2 = cosf(q2);
      float c3 = cosf(q3);
      float s23 = sinf(q2+q3), c23 = cosf(q2+q3);

      Matrix<3, 3> D;
      // Cálculo de la matriz D
      D(0, 0) = I1yy + I2yy + I3yy + m2*(La2x*La2x*c2*c2 + L2z*L2z) + m1*L1z*L1z + 
                m3*(L3z*L3z + (La3x*c23  + a2*c2)*(La3x*c23 + a2*c2)) + 
                s23*s23*(I3xx - I3yy) + s2*s2*(I2xx - I2yy) + Im2x + Im3x + 
                mm3*(pm3z*pm3z + a2*a2*c2*c2) + Im1z*kr1*kr1 + mm2*pm2z*pm2z;
      D(0, 1) = I3xz*s23 + I2xz*s2 - L3z*m3*(La3x*s23 + a2*s2) - L2z*m2*s2*La2x - mm3*a2*pm3z*s2;
      D(0, 2) = s23*(I3xz - m3*L3z*La3x);
      D(1, 0) = D(0, 1);
      D(1, 1) = I2zz + I3zz + m3*(La3x*La3x + a2*a2 + 2*a2*La3x*c3) + m2*La2x*La2x + mm3*a2*a2 + Im2z*kr2*kr2 + Im3z;
      D(1, 2) = I3zz + m3*La3x*(La3x + a2*c3) + Im3z*kr3;
      D(2, 0) = D(0, 2);
      D(2, 1) = D(1, 2);
      D(2, 2) = I3zz + m3*La3x*La3x + Im3z*kr3*kr3;
      return D;
    }

   // Función para calcular la matriz D_aug
    BLA::Matrix<3, 3> get_D_aug(const BLA::Matrix<3, 1> &q) 
    {
      Matrix<3, 3> Jref = {
                          Jm1*kr1*kr1, 0, 0,
                          0, Jm2*kr2*kr2, 0,
                          0, 0, Jm3*kr3*kr3
                          };

      // Cálculo de la matriz D_aug
      Matrix<3, 3> D_aug = get_D(q) + Jref;
      return D_aug;
    } 

    // Función para calcular la matriz C
    BLA::Matrix<3, 3> get_C(const BLA::Matrix<3, 1> &q, const BLA::Matrix<3, 1> &dq) 
    {
      // Variables auxiliares
      float q2 = q(1), q3 = q(2);
      float dq1 = dq(0), dq2 = dq(1), dq3 = dq(2);

      float La2x = a2 - L2x,
            La3x = a3 - L3x;
      float h1 = (I3xx - I3yy - m3*La3x*La3x)/2,
            h2 = I2xx - I2yy - m2*La2x*La2x - a2*a2*(m3 + mm3),
            h3 = m3*a2*La3x,
            h4 = I3xz - m3*L3z*La3x;
      float var1 = sinf(2*q2 + 2*q3),
            var2 = sinf(2*q2 + q3),
            var3 = sinf(2*q2);
      float c2 = cosf(q2);
      float s3 = sinf(q3);
      float c23 = cosf(q2 + q3);

      Matrix<3, 3> C;
      // Cálculo de la matriz C
      C(0, 0) = dq2*(var1*h1 - var2*h3 + var3*h2/2) + dq3*(var1*h1 - h3*(var2 + s3)/2);
      C(0, 1) = dq1*(var1*h1 - var2*h3 + var3*h2/2) + 
                dq2*(h4*c23 + (I2xz - m3*L3z*a2 - m2*L2z*La2x - a2*mm3*pm3z)*c2) + 
                dq3*h4*c23;
      C(0, 2) = dq1*(var1*h1 - h3*(var2 + s3)/2) + dq2*h4*c23 + dq3*h4*c23;
      C(1, 0) = -dq1*(var1*h1 - var2*h3 + var3*h2/2);
      C(1, 1) = -dq3*h3*s3;
      C(1, 2) = -(dq2 + dq3)*h3*s3;
      C(2, 0) = -dq1*(var1*h1 - h3*(var2 + s3)/2);
      C(2, 1) = dq2*h3*s3;
      C(2, 2) = 0;
      return C;
    }

    // Función para calcular la matriz G
    BLA::Matrix<3, 1> get_G(const BLA::Matrix<3, 1> &q)
    {
      // Variables auxiliares
      float q2 = q(1), q3 = q(2);

      float La2x = a2 - L2x,
            La3x = a3 - L3x;
      float c2 = cosf(q2);
      float c23 = cosf(q2 + q3);

      // Cálculo de la matriz G
      Matrix<3, 1> G = {
                        0,
                        g*m3*(a2*c2 + La3x*c23)  +  g*m2*La2x*c2  +  g*mm3*a2*c2,
                        g*m3*La3x*c23
                       };
      return G;
    }

    BLA::Matrix<3, 1> get_Frict(const BLA::Matrix<3, 1> &dq)
    {
      float x1, x2, x3;
      x1 = dq(0)/eps_fric;
      float Fr1 = Fc1*tanhf(x1) + Fv1*dq(0);

      x2 = dq(1)/eps_fric;
      float Fr2 = Fc2*tanhf(x2) + Fv2*dq(1);

      x3 = dq(2)/eps_fric;
      float Fr3 = Fc3*tanhf(x3) + Fv3*dq(2);

      // Matriz Jacobiano analítico
      Matrix<3, 1> Frict = {
                            Fr1,
                            Fr2,
                            Fr3
                           };
      return Frict;
    }

    BLA::Matrix<3, 3> get_B_aug()
    {
      // Matriz friccion viscosa del motor
      Matrix<3, 3> B_aug = {
                          kr1*kr1*((Kt1*Kb1/Ra1) + Bm1), 0, 0,
                          0, kr2*kr2*((Kt2*Kb2/Ra2) + Bm2), 0,
                          0, 0, kr3*kr3*((Kt3*Kb3/Ra3) + Bm3)
                        };
      return B_aug;
    }  

    BLA::Matrix<3, 3> get_Jref_rotor()
    {
      Matrix<3, 3> Jref = {
                          Jm1*kr1*kr1, 0, 0,
                          0, Jm2*kr2*kr2, 0,
                          0, 0, Jm3*kr3*kr3
                          };                        
      return Jref;
    }  
};

#endif
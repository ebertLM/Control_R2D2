////////////////////////////////
// parameters.h
#ifndef PARAMETERS_H
#define PARAMETERS_H

// Parametros generales
constexpr float g = 9.81f;               // aceleracion de la gravedad

// Parámetros del eslabón 1
constexpr float d1 = 157.75e-3f,
                L1y = 23.38e-3f,
                L1z = 13.27e-3f,
                I1xx = 531947.50e-9f,
                I1yy = 587232.79e-9f,
                I1zz = 395302.05e-9f,
                I1yz = 91308.90e-9f,
                m1 = 333.79e-3f;

// Parámetros del eslabón 2
constexpr float a2 = 143.61e-3f,
                L2x = 58.32e-3f,
                L2z = 122.02e-3f,
                I2xx = 794765.96e-9f,
                I2yy = 2100894.02e-9f,
                I2zz = 1407822.82e-9f,
                I2xz = -140711.29e-9f,
                m2 = 329.53e-3f;

//Parámetros del eslabón 3
constexpr float a3 = 151.95e-3f,
                L3x = 94.37e-3f,
                L3z = 13.34e-3f,
                I3xx = 119122.40e-9f,
                I3yy = 528238.71e-9f,
                I3zz = 457951.78e-9f,
                I3xz = -119909.83e-9f,
                m3 = 155.26e-3f;

//Parámetros de la pieza de trabajo
constexpr float a4 = 44.95e-3f;

//Parámetros del motor 1
constexpr float Ra1 = 7.1554f,
                La1 = 4.01e-3f,
                Kt1 = 7.252e-3f,
                Kb1 = 7.252e-3f,
                Jm1 = 5.112e-7f,
                Bm1 = 3.889e-6f,
                Im1x = 24.03777e-6f,
                Im1z = Jm1, // Jm1
                mm1 = 93.93e-3f,
                n1 = 90.0f,
                pm1y = 6.68e-3f,
                pm1z = 81.30e-3f;

//Parámetros del motor 2
constexpr float Ra2 = 7.1554f,
                La2 = 4.01e-3f,
                Kt2 = 7.252e-3f,
                Kb2 = 7.252e-3f,
                Jm2 = 5.112e-7f,
                Bm2 = 3.889e-6f,
                Im2x = 24.03777e-6f,
                Im2z = Jm2, // Jm2
                mm2 = 93.93e-3f,
                n2 = 90.0f,
                pm2y = 6.68e-3f,
                pm2z = 19.70e-3f;

//Parámetros del motor 3
constexpr float Ra3 = 8.0462f,
                La3 = 5.37e-3f,
                Kt3 = 5.424e-3f,
                Kb3 = 5.424e-3f,
                Jm3 = 3.258e-7f,
                Bm3 = 2.552e-6f,
                Im3x = 11366.73e-9f,
                Im3z = Jm3, // Jm3
                mm3 = 57.60e-3f,
                n3 = 74.8f,
                pm3z = 89.17e-3f;

//Factores de Transmisión total
constexpr float kr1 = n1*1.0f,
                kr2 = n2*1.0f,
                kr3 = n3*1.0f;
//Ganancias de los amplificadores
constexpr float Vc1_max = 3.3f,
                Vc2_max = 3.3f,
                Vc3_max = 3.3f;
constexpr float Gv1 = 12.0f/Vc1_max,
                Gv2 = 12.0f/Vc2_max,
                Gv3 = 12.0f/Vc3_max;

//Constantes de friccion
// constexpr float Fc1 = 0.12f,
//                 Fc2 = 0.12f,
//                 Fc3 = 0.04f;             
// constexpr float Fv1 = 0.06f,
//                 Fv2 = 0.06f,
//                 Fv3 = 0.02f;     
constexpr float Fc1 = 0.0135f,
                Fc2 = 0.0160f,
                Fc3 = 0.0400f;             
constexpr float Fv1 = 0.0035f,
                Fv2 = 0.0020f,
                Fv3 = 0.0200f;                  
constexpr float eps_fric = 0.06f;

//Límites angulares de las juntas(rad)
constexpr float q1_max = 2*PI/3, 
                q1_min = -2*PI/3,
                q2_max = 5*PI/6,
                q2_min = 0.0f,
                q3_max = 0.0f,
                q3_min = -5*PI/6;

//Límites de velocidad de las juntas
constexpr float dq1_max = 9.0f, // rad/seg
                dq2_max = 9.0f,
                dq3_max = 10.0f;

//Límites de aceleración de las juntas
constexpr float dddq1_max = 40.0f, // rad/seg
                ddq2_max = 40.0f,
                ddq3_max = 40.0f;

//Torques máximos
constexpr float T1_max = 3.5f,
                T2_max = 3.5f,
                T3_max = 0.98f; 

#endif



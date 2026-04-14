#ifndef TRAJECTORYPLANNING_H
#define TRAJECTORYPLANNING_H

#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Arduino.h>

using namespace BLA;

class TrajectoryPlanning {
private:
  // Parámetros de la ruta circular
  Matrix<3,1> r_, d_, po_, c_;
  Matrix<3,3> R_;
  float rho_; 

  // Parámetros del perfil trapezoidal 
  float so_, sf_, tc_, tf_, centralAngle_;
  float factor_, ds_max_, dds_max_, ds_c_;
  int   err_;

  // Funciones helpers útiles
  static float dot(const Matrix<3,1>& a, const Matrix<3,1>& b) {
    return a(0)*b(0) + a(1)*b(1) + a(2)*b(2);
  }
  static float norm(const Matrix<3,1>& v) {
    return sqrtf(dot(v,v));
  }
  static int sign(float x) {
    return (x>0) - (x<0);
  }
  static Matrix<3,1> cross(const Matrix<3,1> &a, const Matrix<3,1> &b) {
    Matrix<3,1> r;
    r(0) = a(1)*b(2) - a(2)*b(1);
    r(1) = a(2)*b(0) - a(0)*b(2);
    r(2) = a(0)*b(1) - a(1)*b(0);
    return r;
  }
  // Si la norma es mayor a 0, devuelve el vector normalizado
  // si la norma es 0, devuelve el mismo valor
  static Matrix<3,1> normalize(const Matrix<3,1>& v) {
    float n = norm(v);
    return n>0 ? v*(1.0f/n) : v;
  }


public:
  //  Structs para resultados 
  struct TrapezoidalProfile {
    float s, ds, dds;
  };
  struct CircleTrajectory {
    Matrix<3,1> pd, dpd, ddpd;
  };

  //  Constructor 
  TrajectoryPlanning(
    const Matrix<3,1>& r,
    const Matrix<3,1>& d,
    const Matrix<3,1>& po,
    float so,
    float centralAngle,
    float factor,
    float ds_max,
    float dds_max
  ) : r_(normalize(r)),
      d_(d),
      po_(po),
      so_(so),
      centralAngle_(centralAngle),
      factor_(factor),
      ds_max_(ds_max),
      dds_max_(dds_max),
      err_(0)
  {
    computeParameters();
  }

  //  Recalcula c_, R_, rho_, sf_, tc_, tf_   
  void computeParameters() {
    // 1) centro y radio
    auto Y = po_ - d_;
    if (fabsf(dot(r_,Y)) >= norm(Y)) {
      Serial.println("Error parámetros del círculo");
      while(true);  // detiene
    }
    c_   = d_ + dot(Y,r_)*r_;
    rho_ = norm(po_ - c_);
    sf_  = centralAngle_*rho_;

    // 2) matriz de rotación
    auto xp = normalize(po_ - c_);
    auto zp = r_;
    auto yp = cross(zp, xp);

    // Asignamos las columnas de la matriz R_
    R_(0,0) = xp(0); R_(0,1) = yp(0); R_(0,2) = zp(0); 
    R_(1,0) = xp(1); R_(1,1) = yp(1); R_(1,2) = zp(1); 
    R_(2,0) = xp(2); R_(2,1) = yp(2); R_(2,2) = zp(2); 

    // 3) tiempos trapezoidal
    // resolve factor, ds_max_, dds_max_ → tc_, tf_
    float delta = sf_ - so_;
    ds_c_ = factor_*ds_max_;
    ds_c_ = sign(delta)*fabsf(ds_c_);

    if (sign(dds_max_) != sign(delta)) dds_max_ = -dds_max_;

    tc_ = ds_c_/ dds_max_;
    tf_ = tc_ + delta/ds_c_;
    float ds_r = fabsf(delta)/tf_;
    err_ = 0;

    if (fabsf(ds_c_) <= ds_r) {
      //Se corrige la velocidad de crucero muy pequeña
      err_ = 1;
      ds_c_ = ds_r;
      tc_ = ds_c_/ dds_max_;
      tf_ = tc_ + delta/ds_c_;
    } 
    else if (fabsf(ds_c_) > 2*ds_r) {
      //Se corrige la velocidad de crucero muy grande
      err_ = 2;
      ds_c_ = 2*ds_r;
      tc_ = ds_c_/ dds_max_;
      tf_ = tc_ + delta/ds_c_;
    }
  }

  //  Perfil trapezoidal en t ∈ [0, tf_] 
  TrapezoidalProfile profile(float t) const {
    TrapezoidalProfile p;
    if (t <= tc_) {
      p.s   = so_ + 0.5f*dds_max_*t*t;
      p.ds  =    dds_max_*t;
      p.dds =    dds_max_;
    }
    else if (t <= (tf_ - tc_)) {
      p.s   = so_ + dds_max_*tc_*(t-0.5f*tc_);
      p.ds  = dds_max_*tc_;
      p.dds = 0;
    }
    else if (t <= tf_) {
      float dt = tf_ - t;
      p.s   = sf_ - 0.5f*dds_max_*dt*dt;
      p.ds  = dds_max_*dt;
      p.dds = -dds_max_;
    }
    else {
      p.s = sf_; p.ds = 0; p.dds = 0;
    }
    return p;
  }

  //  Trayectoria circular para un perfil dado 
  CircleTrajectory trajectory(const TrapezoidalProfile &p) const {
    CircleTrajectory tr;
    float sn     = p.s/rho_;
    float cos_sn = cosf(sn), sin_sn = sinf(sn);

    Matrix<3,1> pp     = { rho_*cos_sn, rho_*sin_sn, 0 };
    tr.pd   = c_ + R_*pp;
    Matrix<3,1> dpd_local   = { -p.ds*sin_sn, p.ds*cos_sn, 0 };
    tr.dpd  = R_*dpd_local;
    Matrix<3,1> ddpd_local   = {
      -p.ds*p.ds*cos_sn/rho_ - p.dds*sin_sn,
      -p.ds*p.ds*sin_sn/rho_ + p.dds*cos_sn,
       0
    };
    tr.ddpd = R_*ddpd_local;
    return tr;
  }

  //  Accesores 
  float      getSf()   const { return sf_; }
  float      getTf()   const { return tf_; }
  float      getTc()   const { return tc_; }
  float      getRho()  const { return rho_; }
  float      getError()  const { return err_; }
  Matrix<3,1> getCenter()   const { return c_; }
  Matrix<3,3> getRotation() const { return R_; }
};

#endif // TRAJECTORYPLANNING_H

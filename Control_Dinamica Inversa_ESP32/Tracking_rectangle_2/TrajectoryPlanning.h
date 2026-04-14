#ifndef TRAJECTORYPLANNING_H
#define TRAJECTORYPLANNING_H

#include <BasicLinearAlgebra.h>
#include <cstring>               // para strcmp
#include <ElementStorage.h>
#include <Arduino.h>

using namespace BLA;

class TrajectoryPlanning {
private:
  // Puntos para la ruta del rectángulo
  Matrix<3,1> p1_, p2_, p3_, p4_;

  //  Parámetros del perfil trapezoidal 
  float ds_c1_, dds_max1_, tf1_, tc1_, sf1_;
  float ds_c2_, dds_max2_, tf2_, tc2_, sf2_;
  float ds_c3_, dds_max3_, tf3_, tc3_, sf3_;
  float ds_c4_, dds_max4_, tf4_, tc4_, sf4_;
  float Ts_, tadd_, factor_, ds_max_, dds_max_, so_;

  // Funciones helpers útiles
  static float dot(const Matrix<3,1> &a, const Matrix<3,1> &b) {
    return a(0)*b(0) + a(1)*b(1) + a(2)*b(2);
  }
  static float norm(const Matrix<3,1>& v) {
    return sqrtf(dot(v,v));
  }
  static int sign(float x) {
    return (x>0) - (x<0);
  }

  //  Structs para resultados 
  struct SideProfile {
    float ds_c;
    float dds_max;
    float tf;
    float tc;
  };

  // compute ds_c,dds_max,tf,tc for one side
  SideProfile trapez_parameters(float Do, float Df) const {
    SideProfile sp;
    float dds_max = dds_max_;
    float delta = Df - Do;
    float ds_c  = factor_ * ds_max_;
    ds_c = sign(delta) * fabsf(ds_c);

    if (sign(dds_max) != sign(delta)) dds_max = -dds_max;

    float tc = ds_c / dds_max;
    float tf = tc + delta / ds_c;
    float ds_r = fabsf(delta) / tf;

    if (fabsf(ds_c) <= ds_r) {
      ds_c = ds_r;
      tc   = ds_c / dds_max;
      tf   = tc + delta / ds_c;
    } else if (fabsf(ds_c) > 2*ds_r) {
      ds_c = 2*ds_r;
      tc   = ds_c / dds_max;
      tf   = tc + delta / ds_c;
    }
    sp.ds_c = ds_c;
    sp.dds_max = dds_max;
    sp.tf = tf;
    sp.tc = tc;
    return sp;
  }

public:
  // output structs
  struct TrapezoidalProfile {
    float s, ds, dds;
  };
  struct LineTrajectory {
    Matrix<3,1> pd, dpd, ddpd;
  };

  // Constructor
  TrajectoryPlanning(
    const Matrix<3,1>& p1,
    const Matrix<3,1>& p2,
    const Matrix<3,1>& p3,
    const Matrix<3,1>& p4,
    float so,
    float Ts,
    float tadd,
    float factor,
    float ds_max,
    float dds_max
  ) : p1_(p1), p2_(p2), p3_(p3), p4_(p4),
      so_(so), Ts_(Ts), tadd_(tadd),
      factor_(factor), ds_max_(ds_max), dds_max_(dds_max)
  {
    computeParameters();
  }

  // recompute all side parameters
  void computeParameters() 
  {
    // side 1
    float Do1 = so_;
    float Df1 = norm(p2_ - p1_);
    auto sp1 = trapez_parameters(Do1, Df1);
    ds_c1_   = sp1.ds_c;    dds_max1_ = sp1.dds_max;
    tf1_     = sp1.tf;      tc1_       = sp1.tc; sf1_ = Df1;

    // side 2
    float Do2 = so_;
    float Df2 = norm(p3_ - p2_);
    auto sp2 = trapez_parameters(Do2, Df2);
    ds_c2_   = sp2.ds_c;    dds_max2_ = sp2.dds_max;
    tf2_     = sp2.tf;      tc2_       = sp2.tc; sf2_ = Df2;

    // side 3
    float Do3 = so_;
    float Df3 = norm(p4_ - p3_);
    auto sp3 = trapez_parameters(Do3, Df3);
    ds_c3_   = sp3.ds_c;    dds_max3_ = sp3.dds_max;
    tf3_     = sp3.tf;      tc3_       = sp3.tc; sf3_ = Df3;

    // side 4
    float Do4 = so_;
    float Df4 = norm(p1_ - p4_);
    auto sp4 = trapez_parameters(Do4, Df4);
    ds_c4_   = sp4.ds_c;    dds_max4_ = sp4.dds_max;
    tf4_     = sp4.tf;      tc4_       = sp4.tc; sf4_ = Df4;
  }

  // trapezoidal profile at time t for given side (1–4)
  TrapezoidalProfile profile(float t, const char* side) const 
  {
    TrapezoidalProfile p;
    float so = so_, sf=0, tc=0, tf=0, dds_max=0;

    if      (strcmp(side,"lado1")==0) sf=sf1_, tc=tc1_, tf=tf1_, dds_max=dds_max1_;
    else if (strcmp(side,"lado2")==0) sf=sf2_, tc=tc2_, tf=tf2_, dds_max=dds_max2_;
    else if (strcmp(side,"lado3")==0) sf=sf3_, tc=tc3_, tf=tf3_, dds_max=dds_max3_;
    else if (strcmp(side,"lado4")==0) sf=sf4_, tc=tc4_, tf=tf4_, dds_max=dds_max4_;

    
    if (t <= tc) {
      p.s   = so + 0.5f * dds_max * t * t;
      p.ds  = dds_max * t;
      p.dds = dds_max;
    }
    else if (t <= (tf - tc)) {
      p.s   = so + dds_max * tc * (t - 0.5f * tc);
      p.ds  = dds_max * tc;
      p.dds = 0.0f;
    }
    else if (t <= tf) {
      float dt = tf - t;
      p.s   = sf - 0.5f * dds_max * dt * dt;
      p.ds  = dds_max * dt;
      p.dds = -dds_max;
    }
    else {
      p.s   = sf;
      p.ds  = 0.0f;
      p.dds = 0.0f;
    }
    return p;
  }

  // line segment trajectory for given side
  LineTrajectory trajectory(const TrapezoidalProfile &p, const char* side) const 
  {
    Matrix<3,1> po, pf;
    float sf = 0;

    if      (strcmp(side,"lado1")==0) po = p1_, pf = p2_, sf = sf1_;
    else if (strcmp(side,"lado2")==0) po = p2_, pf = p3_, sf = sf2_;
    else if (strcmp(side,"lado3")==0) po = p3_, pf = p4_, sf = sf3_;
    else if (strcmp(side,"lado4")==0) po = p4_, pf = p1_, sf = sf4_;

    // Lectura de la posicion, velocidad y aceleracion 
    // del perfil trapezoidal
    float s = p.s, ds = p.ds, dds = p.dds;

    LineTrajectory tr;
    // position
    tr.pd   = po + (pf - po) * (s / sf);
    // velocity
    tr.dpd  = (pf - po) * (ds / sf);
    // acceleration
    tr.ddpd = (pf - po) * (dds / sf);
    return tr;
  }

  // getters for times and total
  float getTf1() const { return tf1_; }
  float getTf2() const { return tf2_; }
  float getTf3() const { return tf3_; }
  float getTf4() const { return tf4_; }
  float getTotalTime() const { return tf1_ + tf2_ + tf3_ + tf4_; }
};

#endif // TRAJECTORYPLANNINGRECTANGLE_H

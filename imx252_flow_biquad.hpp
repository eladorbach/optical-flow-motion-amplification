#pragma once
#include <array>
#include <cmath>
// Causal biquad bandpass, unity response at center frequency.
// For equivalent difference-input displacement at ~236fps.
// Signal center is image-measured, not verified physical M1600 frequency.
struct BiquadMotion {
 double b0=0,b1=0,b2=0,a1=0,a2=0,x1=0,x2=0,y1=0,y2=0;
 void configure(double fs=235.98,double fc=10.45,double q=2.2) {
   constexpr double pi=3.14159265358979323846;
   const double omega=2*pi*fc/fs;
   const double alpha=std::sin(omega)/(2*q);
   const double norm=1/(1+alpha);
   b0=alpha*norm;b1=0;b2=-alpha*norm;
   a1=-2*std::cos(omega)*norm;a2=(1-alpha)*norm;
   reset();
 }
 void reset(){x1=x2=y1=y2=0;}
 float step(float x){
   const double y=b0*x+b1*x1+b2*x2-a1*y1-a2*y2;
   x2=x1;x1=x;y2=y1;y1=y;
   return float(std::isfinite(y)?y:0);
 }
};

#include "imx252_flow_biquad.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>
constexpr double PI=3.14159265358979323846, FS=235.98;
double response(double f){
 BiquadMotion b;b.configure(FS,10.45,2.2);
 double cs=0,sn=0;int n=0;
 for(int i=0;i<4096;i++){
   const double z=std::sin(2*PI*f*i/FS);
   const double y=b.step(float(z));
   if(i>=1536){cs+=y*std::cos(2*PI*f*i/FS);sn+=y*std::sin(2*PI*f*i/FS);n++;}
 }
 return 2*std::hypot(cs,sn)/std::max(1,n);
}
int main(){
 const double center=response(10.45),low=response(3.0),middle=response(18),high=response(50),lighting=response(100);
 printf("V2_BANDPASS center_10.45Hz=%.3f pass_18Hz=%.3f reject_3Hz=%.3f reject_50Hz=%.3f reject_100Hz=%.3f\n",
   center,middle,low,high,lighting);
 bool pass=center>.90 && center<1.08 && low<.28 && high<.13 && lighting<.04;
 printf("V2_FILTER_%s: bandpass response\n",pass?"PASS":"FAIL");
 return pass?0:2;
}

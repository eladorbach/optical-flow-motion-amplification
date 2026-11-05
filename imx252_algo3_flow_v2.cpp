// Optical flow motion amplification for a high-speed monochrome camera.
// Tracks two regions relative to a reference patch and filters motion near 10.45 Hz.
// Renders a portrait X11 preview. Gain is not a calibrated displacement ratio.
// Copyright (c) 2026 Elad Orbach.
#include "imx252_flow_biquad.hpp"
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <X11/Xutil.h>
#include <X11/extensions/XShm.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <array>

constexpr int W=1024,H=768,OUT_W=768,OUT_H=1024,PW=256,PH=192,QW=128,QH=96;
constexpr double PI=3.14159265358979323846;
std::atomic<bool> run{true}, bypass{false};
std::atomic<int> amp{15};
std::atomic<int> sideLX{170},sideLY{420},sideRX{480},sideRY{390};
std::atomic<bool> roiDirty{false},flowReset{false};
std::atomic<double> processor_ms{0};
std::atomic<uint64_t> cap_frames{0}, cap_missing{0}, cap_errors{0}, cap_timeouts{0}, ui_frames{0};
std::atomic<double> cam_fps{0}, cap_lag_ms{0}, app_render_ms{0};
std::atomic<int> cam_fps_request{238};
std::atomic<bool> cam_ready{false}, cam_failed{false};
std::string cam_error;

static double monotonic() {
  timespec t{}; clock_gettime(CLOCK_MONOTONIC,&t);return double(t.tv_sec)+double(t.tv_nsec)*1e-9;
}
static bool xioctl(int fd,unsigned long req,void* v){
  int rc;do{rc=ioctl(fd,req,v);}while(rc<0 && errno==EINTR);return rc==0;
}
// renderer uses inline saturating conversion for low latency.
struct Shared {
 std::mutex m;
 std::vector<uint8_t> raw;
 std::array<float,4> motion{};
 uint64_t sequence=0;
 double camera_time=0,captured_host_time=0;
 bool valid=false;
 Shared():raw(W*H){}
} shared;
struct Snap {
 std::vector<uint8_t> raw;
 std::array<float,4> motion{};
 uint64_t seq=0;
 double sensor_time=0,arrived=0;
 bool valid=false;
 Snap():raw(W*H){}
};
struct Cam {
  int fd=-1;
  struct Buf{void* ptr=nullptr;size_t len=0;};
  std::vector<Buf> bufs;
  bool streaming=false,changed_format=false,changed_fps=false;
  v4l2_format orig{};
  v4l2_streamparm origparm{};
  void closeall(){
    if(streaming){int t=V4L2_BUF_TYPE_VIDEO_CAPTURE; xioctl(fd,VIDIOC_STREAMOFF,&t); streaming=false;}
    for(auto &b:bufs) if(b.ptr && b.ptr!=MAP_FAILED) munmap(b.ptr,b.len);
    bufs.clear();
    if(fd>=0) {
      v4l2_requestbuffers r{};r.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;r.memory=V4L2_MEMORY_MMAP;r.count=0; xioctl(fd,VIDIOC_REQBUFS,&r);
      if(changed_format) xioctl(fd,VIDIOC_S_FMT,&orig);
      if(changed_fps) xioctl(fd,VIDIOC_S_PARM,&origparm);
      ::close(fd);fd=-1;
    }
  }
  ~Cam(){closeall();}
  bool openit(std::string &why){
    fd=open("/dev/video0",O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if(fd<0){why=std::string("open /dev/video0: ")+strerror(errno);return false;}
    orig.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;origparm.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(!xioctl(fd,VIDIOC_G_FMT,&orig) || !xioctl(fd,VIDIOC_G_PARM,&origparm)) {
      why="V4L2 cannot read initial capture settings";return false;
    }
    v4l2_format f=orig;f.fmt.pix.width=W;f.fmt.pix.height=H;f.fmt.pix.pixelformat=V4L2_PIX_FMT_GREY;f.fmt.pix.field=V4L2_FIELD_NONE;
    if(!xioctl(fd,VIDIOC_S_FMT,&f)){why=std::string("VIDIOC_S_FMT: ")+strerror(errno);return false;}
    changed_format=true;
    if(f.fmt.pix.width!=W||f.fmt.pix.height!=H||f.fmt.pix.pixelformat!=V4L2_PIX_FMT_GREY) {
      why="Camera rejected 1024x768 GREY";return false;
    }
    v4l2_streamparm prm=origparm;prm.parm.capture.timeperframe.numerator=1000;prm.parm.capture.timeperframe.denominator=238005;
    if(!xioctl(fd,VIDIOC_S_PARM,&prm)){why=std::string("VIDIOC_S_PARM: ")+strerror(errno);return false;}
    changed_fps=true;
    double actual=prm.parm.capture.timeperframe.numerator?double(prm.parm.capture.timeperframe.denominator)/prm.parm.capture.timeperframe.numerator:0;
    if(actual<200) {why="Sensor cannot negotiate >200 fps; got "+std::to_string(actual);return false;}
    cam_fps_request.store((int)std::round(actual));
    printf("CAMERA accepted 1024x768 GREY %.3f fps, bytesperline=%u, buffer size=%u\n",
      actual, f.fmt.pix.bytesperline, f.fmt.pix.sizeimage);
    v4l2_requestbuffers r{};r.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;r.memory=V4L2_MEMORY_MMAP;r.count=8;
    if(!xioctl(fd,VIDIOC_REQBUFS,&r)||r.count<3){why="V4L2 MMAP buffers unavailable";return false;}
    bufs.resize(r.count);
    for(unsigned i=0;i<r.count;i++){
      v4l2_buffer b{};b.type=r.type;b.memory=r.memory;b.index=i;
      if(!xioctl(fd,VIDIOC_QUERYBUF,&b)){why="VIDIOC_QUERYBUF failed";return false;}
      bufs[i].len=b.length;
      bufs[i].ptr=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,b.m.offset);
      if(bufs[i].ptr==MAP_FAILED){why="camera mmap failed";return false;}
      if(!xioctl(fd,VIDIOC_QBUF,&b)){why="VIDIOC_QBUF failed";return false;}
    }
    int t=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(!xioctl(fd,VIDIOC_STREAMON,&t)){why=std::string("VIDIOC_STREAMON: ")+strerror(errno);return false;}
    streaming=true;return true;
  }
};

struct TrackedMotion {
 static constexpr int SIZE=40,STEP=2;
 std::array<std::array<uint8_t,SIZE*SIZE>,3> old{};
 std::array<std::array<float,2>,2> pos{},band{};
 std::array<std::array<BiquadMotion,2>,2> filter{};
 TrackedMotion(){ for(auto &a:filter)for(auto &b:a)b.configure(235.98,10.45,2.2); }
 uint64_t seq=0;
 void reset(){seq=0;for(auto &v:pos)v={0,0};for(auto &v:band)v={0,0};for(auto &a:filter)for(auto &b:a)b.reset();}
 void update(const uint8_t *frame,double dt){
  const int xs[3]={sideLY.load(),sideRY.load(),120};
  const int ys[3]={H-1-sideLX.load(),H-1-sideRX.load(),140};
  float velocity[3][2]{};
  for(int k=0;k<3;k++){
   double xx=0,xy=0,yy=0,tx=0,ty=0;
   for(int j=0;j<SIZE;j++)for(int i=0;i<SIZE;i++){
    int px=std::max(2,std::min(W-3,xs[k]-40+i*STEP));
    int py=std::max(2,std::min(H-3,ys[k]-40+j*STEP));
    int id=j*SIZE+i;
    float v=frame[size_t(py)*W+px],before=old[k][id];
    old[k][id]=uint8_t(v);
    if(seq==0||v>250||v<6)continue;
    float gx=.5f*(float(frame[size_t(py)*W+px+1])-float(frame[size_t(py)*W+px-1]));
    float gy=.5f*(float(frame[size_t(py+1)*W+px])-float(frame[size_t(py-1)*W+px]));
    float it=v-before,weight=std::min(1.f,(gx*gx+gy*gy)/300.f);
    xx+=weight*gx*gx;xy+=weight*gx*gy;yy+=weight*gy*gy;
    tx-=weight*gx*it;ty-=weight*gy*it;
   }
   double det=xx*yy-xy*xy;
   if(det>100000 && xx>500 && yy>500){
    double dx=(tx*yy-ty*xy)/det,dy=(ty*xx-tx*xy)/det;
    if(std::isfinite(dx)&&std::isfinite(dy)&&std::fabs(dx)<3&&std::fabs(dy)<3){
     velocity[k][0]=float(dx);velocity[k][1]=float(dy);
    }
   }
  }
  // Unit-gain 10.45Hz bandpass on integrated displacement, sampled
  // on EVERY incoming frame. Suppresses 50/100Hz illumination and long-term drift.
  for(int g=0;g<2;g++)for(int k=0;k<2;k++){
   const float dv=velocity[g][k]-velocity[2][k];
   pos[g][k]+=std::max(-1.8f,std::min(1.8f,dv));
   // Saturate DC integrator at broad limits; exact DC is rejected by biquad.
   pos[g][k]=std::max(-500.f,std::min(500.f,pos[g][k]));
   band[g][k]=filter[g][k].step(pos[g][k]);
  }
  seq++;
 }
};

static void process_camera() {
  Cam c;
  if(!c.openit(cam_error)){cam_failed=true;run=false;printf("CAMERA ERROR: %s\n",cam_error.c_str());fflush(stdout);return;}
  cam_ready=true;
  TrackedMotion tracker;
  double prior_stamp=0,prior_publish=0,tick_start=monotonic();
  uint32_t priorseq=0;bool have_seq=false;
  uint64_t tick_count=0;
  while(run.load(std::memory_order_relaxed)) {
    pollfd p{c.fd,POLLIN,0};
    int z=poll(&p,1,100);
    if(z==0){cap_timeouts++;continue;}
    if(z<0){if(errno==EINTR) continue;cam_error=strerror(errno);cam_failed=true;break;}
    if(!(p.revents&POLLIN)) {cap_errors++;continue;}
    v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;b.memory=V4L2_MEMORY_MMAP;
    if(!xioctl(c.fd,VIDIOC_DQBUF,&b)){if(errno==EAGAIN)continue;cam_error=strerror(errno);cam_failed=true;break;}
    if(b.index>=c.bufs.size()||b.bytesused<size_t(W*H)) {
      cap_errors++;xioctl(c.fd,VIDIOC_QBUF,&b);continue;
    }
    auto now=monotonic();
    double capturestamp=double(b.timestamp.tv_sec)+double(b.timestamp.tv_usec)*1e-6;
    if(have_seq && b.sequence>priorseq+1)cap_missing+=(uint64_t)(b.sequence-priorseq-1);
    have_seq=true;priorseq=b.sequence;
    if(b.flags & V4L2_BUF_FLAG_ERROR) cap_errors++;
    auto src=(const uint8_t*)c.bufs[b.index].ptr;
    double processingStart=monotonic();
    const double delta=(prior_stamp>0 && capturestamp-prior_stamp>0 && capturestamp-prior_stamp<.05)?capturestamp-prior_stamp:1./238.;
    prior_stamp=capturestamp;
    if(flowReset.exchange(false))tracker.reset();
    tracker.update(src,delta);
    processor_ms.store((monotonic()-processingStart)*1000.,std::memory_order_relaxed);
    cap_frames++;tick_count++;
    double lag=(now-capturestamp)*1000.;
    if(lag>=0 && lag<1000) cap_lag_ms.store(lag,std::memory_order_relaxed);
    // Publish the latest raw frame and filtered displacements at most 60 Hz.
    if(now-prior_publish>=1.0/60.){
      std::lock_guard<std::mutex> lk(shared.m);
      std::memcpy(shared.raw.data(),src,W*H);
      shared.motion={tracker.band[0][0],tracker.band[0][1],tracker.band[1][0],tracker.band[1][1]};
      shared.sequence=cap_frames.load();
      shared.camera_time=capturestamp;
      shared.captured_host_time=now;
      shared.valid=true;
      prior_publish=now;
    }
    if(!xioctl(c.fd,VIDIOC_QBUF,&b)){cam_error="VIDIOC_QBUF failed";cam_failed=true;break;}
    if(now-tick_start>=2.0){
      double fps=tick_count/(now-tick_start);
      cam_fps.store(fps,std::memory_order_relaxed);
      printf("CAMERA FLOW_V2 fps=%.2f total=%llu missed=%llu errors=%llu lag_ms=%.2f process_ms=%.3f render_count=%llu bandLeft=(%.3f,%.3f) bandRight=(%.3f,%.3f)\n",
        fps,(unsigned long long)cap_frames.load(),(unsigned long long)cap_missing.load(),
        (unsigned long long)cap_errors.load(),cap_lag_ms.load(),processor_ms.load(),(unsigned long long)ui_frames.load(),tracker.band[0][0],tracker.band[0][1],tracker.band[1][0],tracker.band[1][1]);
      fflush(stdout);tick_start=now;tick_count=0;
    }
  }
  run=false;
  printf("CAMERA ending frames=%llu missing=%llu errors=%llu timeouts=%llu reason=%s\n",
    (unsigned long long)cap_frames.load(),(unsigned long long)cap_missing.load(),
    (unsigned long long)cap_errors.load(),(unsigned long long)cap_timeouts.load(),
    cam_failed?cam_error.c_str():"normal shutdown");
  fflush(stdout);
}
static void on_signal(int) {run=false;}
int main(int argc,char**argv){
  if(argc>1) {int a=atoi(argv[1]);if(a>=0&&a<=80)amp=a;}
  signal(SIGINT,on_signal);signal(SIGTERM,on_signal);
  setvbuf(stdout,nullptr,_IOLBF,0);
  Display *d=XOpenDisplay(nullptr);
  if(!d){fprintf(stderr,"Cannot open existing X11 display. Set DISPLAY=:1 and valid XAUTHORITY.\n");return 10;}
  int screen=DefaultScreen(d);
  int depth=DefaultDepth(d,screen);
  Visual *visual=DefaultVisual(d,screen);
  if(depth!=24 && depth!=32) {fprintf(stderr,"Unsupported X11 visual depth=%d\n",depth);XCloseDisplay(d);return 11;}
  Window w=XCreateSimpleWindow(d,RootWindow(d,screen),90,65,OUT_W,OUT_H,1,BlackPixel(d,screen),BlackPixel(d,screen));
  XStoreName(d,w,"M1600");
  XSelectInput(d,w,ExposureMask|KeyPressMask|ButtonPressMask|StructureNotifyMask);
  Atom wmDel=XInternAtom(d,"WM_DELETE_WINDOW",False);
  XSetWMProtocols(d,w,&wmDel,1);
  XMapRaised(d,w);
  GC gc=XCreateGC(d,w,0,nullptr);
  XSetForeground(d,gc,WhitePixel(d,screen));
  XFlush(d);
  bool shm_enabled=false;
  XShmSegmentInfo shm{};
  XImage *img=nullptr;
  if(XShmQueryExtension(d)) {
    img=XShmCreateImage(d,visual,depth,ZPixmap,nullptr,&shm,OUT_W,OUT_H);
    if(img && img->bits_per_pixel==32) {
      shm.shmid=shmget(IPC_PRIVATE,size_t(img->bytes_per_line)*img->height,IPC_CREAT|0600);
      if(shm.shmid>=0) {
        shm.shmaddr=(char*)shmat(shm.shmid,nullptr,0);
        if(shm.shmaddr!=(char*)-1) {
          img->data=shm.shmaddr;shm.readOnly=False;
          if(XShmAttach(d,&shm)) {
            shm_enabled=true;
            XSync(d,False);
          }
        }
      }
      if(shm.shmid>=0)shmctl(shm.shmid,IPC_RMID,nullptr); // auto destroy after detach
    }
    if(!shm_enabled) {
      if(img) {
        img->data=nullptr;XDestroyImage(img);
      }
      if(shm.shmaddr && shm.shmaddr!=(char*)-1)shmdt(shm.shmaddr);
      img=nullptr;
    }
  }
  if(!img) {
    char *buf=(char*)calloc(size_t(OUT_W*OUT_H),4);
    if(!buf){fprintf(stderr,"image allocation failed\n");XCloseDisplay(d);return 12;}
    img=XCreateImage(d,visual,depth,ZPixmap,0,buf,OUT_W,OUT_H,32,0);
    if(!img || img->bits_per_pixel!=32) {
      fprintf(stderr,"XImage incompatible 32bpp\n");if(img)XDestroyImage(img);else free(buf);XCloseDisplay(d);return 13;
    }
  }
  printf("DISPLAY: open X11 %s, image %dx%d %dbpp transfer=%s, monitor unchanged\n",
    DisplayString(d),OUT_W,OUT_H,img->bits_per_pixel,shm_enabled?"MIT-SHM":"XPutImage fallback");
  std::thread worker(process_camera);
  Snap snap;
  uint64_t seenSeq=0; int renderCount=0; double renderTick=monotonic();
  double next=monotonic();
  constexpr int RAD=90;
  std::array<std::vector<float>,2> masks;
  for(auto &m:masks)m.resize(size_t(2*RAD+1)*(2*RAD+1));
  for(auto &m:masks){
    for(int dy=-RAD;dy<=RAD;dy++)for(int dx=-RAD;dx<=RAD;dx++){
      float v=std::exp(-(dx*dx+dy*dy)/(2.f*42.f*42.f));
      m[size_t(dy+RAD)*(2*RAD+1)+size_t(dx+RAD)]=v<.018f?0.f:v;
    }
  }
  while(run.load()) {
    while(XPending(d)>0){
      XEvent e{}; XNextEvent(d,&e);
      if(e.type==ClientMessage && (Atom)e.xclient.data.l[0]==wmDel){run=false;break;}
      if(e.type==ButtonPress&&(e.xbutton.button==Button1||e.xbutton.button==Button3)){
        const int x=std::max(0,std::min(OUT_W-1,e.xbutton.x));
        const int y=std::max(0,std::min(OUT_H-1,e.xbutton.y));
        if(e.xbutton.button==Button1){sideLX=x;sideLY=y;}
        else {sideRX=x;sideRY=y;}
        roiDirty=true;flowReset=true;
        printf("FLOW ROI %s (%d,%d)\n",e.xbutton.button==Button1?"left":"right",x,y);
      }
      if(e.type==KeyPress) {
        KeySym k=XLookupKeysym(&e.xkey,0);
        if(k==XK_q || k==XK_Escape){run=false;break;}
        if(k==XK_space){bypass.store(!bypass.load());printf("CONTROL bypass=%d\n",int(bypass.load()));}
        if(k==XK_plus || k==XK_equal){amp.store(std::min(80,amp.load()+5));printf("CONTROL alpha=%d\n",amp.load());}
        if(k==XK_minus){amp.store(std::max(0,amp.load()-5));printf("CONTROL alpha=%d\n",amp.load());}
        if(k==XK_r||k==XK_R){flowReset=true;printf("FLOW V2 phase baseline reset\n");}
      }
    }
    double now=monotonic();
    if(now<next){std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;}
    next=std::max(next+1.0/60., now);
    {
      std::lock_guard<std::mutex> lk(shared.m);
      if(shared.valid && shared.sequence!=seenSeq){
        std::memcpy(snap.raw.data(),shared.raw.data(),W*H);
        snap.motion=shared.motion;
        snap.seq=shared.sequence;
        snap.sensor_time=shared.camera_time;
        snap.arrived=shared.captured_host_time;
        snap.valid=true;
      }
    }
    if(!snap.valid || snap.seq==seenSeq)continue; // never repeat stale output frames
    seenSeq=snap.seq;
    // Tiled clockwise transpose of camera pixels. Source is row-major 1024x768;
    // output is 768x1024. 16x16 tiles keep a small block of source and destination
    // cache-resident rather than making a pathological 1024-byte stride per pixel.
    const bool raw=bypass.load();
    const int gain=amp.load();
    auto *dst=(uint32_t*)img->data;
    constexpr int TILE=16;
    for(int y0=0;y0<H;y0+=TILE)for(int x0=0;x0<W;x0+=TILE){
      for(int x=x0;x<x0+TILE;x++){
        uint32_t* row=dst+size_t(x)*OUT_W;
        for(int y=y0;y<y0+TILE;y++){
          int v=snap.raw[size_t(y)*W+x];
          row[H-1-y]=uint32_t(v)*0x010101u;
        }
      }
    }
    if(!raw && gain>1)for(int g=0;g<2;g++){
      const int cx=g==0?sideLY.load():sideRY.load();
      const int cy=H-1-(g==0?sideLX.load():sideRX.load());
      float sx=float(gain-1)*snap.motion[g*2];
      float sy=float(gain-1)*snap.motion[g*2+1];
      sx=std::max(-18.f,std::min(18.f,sx));
      sy=std::max(-18.f,std::min(18.f,sy));
      for(int dy=-RAD;dy<=RAD;dy++){
        int y=cy+dy;if(y<0||y>=H)continue;
        const float* maskRow=masks[g].data()+size_t(dy+RAD)*(2*RAD+1);
        for(int dx=-RAD;dx<=RAD;dx++){
          int x=cx+dx;if(x<0||x>=W)continue;
          float w=maskRow[dx+RAD];if(w<=0)continue;
          float xx=std::max(0.f,std::min(float(W-2),x-sx));
          float yy=std::max(0.f,std::min(float(H-2),y-sy));
          int ix=int(xx),iy=int(yy);
          float fx=xx-ix,fy=yy-iy;
          const uint8_t* a=snap.raw.data()+size_t(iy)*W+ix;
          float interp=(1.f-fy)*((1.f-fx)*a[0]+fx*a[1])
                        +fy*((1.f-fx)*a[W]+fx*a[W+1]);
          int orig=snap.raw[size_t(y)*W+x];
          int v=std::max(0,std::min(255,int(orig+w*(interp-orig)+.5f)));
          dst[size_t(x)*OUT_W+H-1-y]=uint32_t(v)*0x010101u;
        }
      }
    }
    if(shm_enabled) XShmPutImage(d,w,gc,img,0,0,0,0,OUT_W,OUT_H,False);
    else XPutImage(d,w,gc,img,0,0,0,0,OUT_W,OUT_H);
    if(shm_enabled) XSync(d,False); // image buffer safe to reuse after server read
    else XFlush(d);
    ui_frames++;renderCount++;
    double ss=(monotonic()-snap.sensor_time)*1000.;
    if(ss>=0 && ss<1000)app_render_ms.store(ss,std::memory_order_relaxed);
    now=monotonic();
    if(now-renderTick>=3.0) {
      printf("RENDER FLOW_V2 outputs=%.2f fps input=%.2f fps software_latency_ms=%.2f missing=%llu gain=%d motionL=(%.3f,%.3f) motionR=(%.3f,%.3f)\n",
        renderCount/(now-renderTick),cam_fps.load(),app_render_ms.load(),(unsigned long long)cap_missing.load(),gain,snap.motion[0],snap.motion[1],snap.motion[2],snap.motion[3]);
      fflush(stdout);renderCount=0;renderTick=now;
    }
  }
  run=false;worker.join();
  if(shm_enabled){XShmDetach(d,&shm); XSync(d,False); img->data=nullptr; XDestroyImage(img); shmdt(shm.shmaddr);}
  else XDestroyImage(img);
  XFreeGC(d,gc);XDestroyWindow(d,w);XCloseDisplay(d);
  return cam_failed.load()?20:0;
}

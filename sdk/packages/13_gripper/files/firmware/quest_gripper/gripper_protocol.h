#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#ifdef GRIPPER_FEEDBACK_PIN
#include "position_pi.h"
#endif

#ifndef GRIPPER_MIN_US
#define GRIPPER_MIN_US 1000
#endif
#ifndef GRIPPER_MAX_US
#define GRIPPER_MAX_US 2000
#endif
static_assert(GRIPPER_MIN_US >= 400 && GRIPPER_MIN_US <= 1000 && GRIPPER_MIN_US % 4 == 0,
              "Unsupported Servo library lower endpoint");
static_assert(GRIPPER_MAX_US >= GRIPPER_MIN_US + 20 && GRIPPER_MAX_US <= 2000,
              "Unsupported gripper upper endpoint");

// Shared by the AVR sketch and the host-side protocol verification harness.
template<class Output> class GripperProtocol {
public:
  explicit GripperProtocol(char side, Output& output): side_(side), io_(output) {}
  void off() {
    io_.off(); armed_=false; attached_=false; token_=seq_=0;
#ifdef GRIPPER_FEEDBACK_PIN
    positionMode_=false; pi_.reset();
#endif
  }
  void update(uint32_t now) {
    if (!armed_) return;
    if ((uint32_t)(now-lastCommand_) >= 300) { off(); return; }
    uint32_t dt=now-lastStep_;
#ifdef GRIPPER_FEEDBACK_PIN
    if (positionMode_) {
      if (attached_ && dt>=20) { lastStep_=now; applyPosition(dt); }
      return;
    }
#endif
    if (!attached_ || rate_==0 || dt<20) return;
    lastStep_=now;
    uint32_t budget=dt*rate_+remainder_;
    int step=(int)(budget/1000); remainder_=budget%1000;
    int d=target_-current_;
    if (d>step) d=step; if (d< -step) d=-step;
    if (d) {current_+=d;io_.pulse(current_);}
  }
  void line(char* text, uint32_t now, char* reply, size_t size) {
    update(now);
    char* args[10]; int n=0; char* ctx=0;
    for(char* t=strtok_r(text," ",&ctx);t;t=strtok_r(0," ",&ctx)) {
      if(n==10){reject(reply,size);return;}args[n++]=t;
    }
    if(n==1 && !strcmp(args[0],"ID?")) {off();snprintf(reply,size,"E6G1 %c",side_);return;}
#ifdef GRIPPER_FEEDBACK_PIN
    if(n==1 && !strcmp(args[0],"ADC?") && !armed_) {
      snprintf(reply,size,"ADC %d",io_.positionAdc());return;
    }
#endif
    if(n==1 && !strcmp(args[0],"OFF")) {off();snprintf(reply,size,"OFF");return;}
    uint32_t key=0,a=0,b=0,r=0;
    bool directArm=n==6 && !strcmp(args[0],"ARM");
    bool positionArm=n==10 && !strcmp(args[0],"ARMPI");
    if((directArm || positionArm) && !armed_ && (side_=='L'||side_=='R') &&
        args[1][0]==side_ && !args[1][1] && number(args[2],key) && key &&
        number(args[3],a) && number(args[4],b) &&
        a>=GRIPPER_MIN_US && a<=GRIPPER_MAX_US && b>=GRIPPER_MIN_US && b<=GRIPPER_MAX_US &&
        abs((int)a-(int)b)>=20) {
      if (positionArm) {
#ifdef GRIPPER_FEEDBACK_PIN
        uint32_t v[5];
        for (int i=0;i<5;++i) {
          if (!number(args[5+i],v[i]) || v[i]>32767) {reject(reply,size);return;}
        }
        if (!pi_.configure(a,b,v[0],v[1],v[2],v[3],v[4]) || !pi_.valid(io_.positionAdc())) {
          reject(reply,size);return;
        }
        positionMode_=true;
#else
        reject(reply,size);return;
#endif
      } else if (!number(args[5],r) || !(r==0 || (r>=20 && r<=1000))) {
        reject(reply,size);return;
      }
      token_=key;low_=a<b?a:b;high_=a>b?a:b;open_=b;rate_=r;seq_=0;remainder_=0;
      lastCommand_=lastStep_=now;armed_=true;attached_=false;
      snprintf(reply,size,positionArm?"ARMEDPI %lu":"ARMED %lu",(unsigned long)key);return;
    }
    if(n==4 && !strcmp(args[0],"SET") && armed_ && number(args[1],key) && key==token_ &&
       number(args[2],a) && a>seq_ && number(args[3],b) && b>=low_ && b<=high_ && (attached_ || b==open_)) {
      lastCommand_=now;seq_=a;target_=(int)b;
#ifdef GRIPPER_FEEDBACK_PIN
      if(positionMode_) {
        if(!attached_) lastStep_=now;
        if(!applyPosition(0)) {snprintf(reply,size,"ERR FEEDBACK");return;}
        snprintf(reply,size,"ACKPI %lu %lu %u %d %d",(unsigned long)key,(unsigned long)a,(unsigned)b,current_,adc_);
        return;
      }
#endif
      if(!attached_) {current_=target_;lastStep_=now;io_.start(current_);attached_=true;}
      else if(rate_==0 && current_!=target_) {current_=target_;io_.pulse(current_);}
      snprintf(reply,size,"ACK %lu %lu %u",(unsigned long)key,(unsigned long)a,(unsigned)b);return;
    }
    reject(reply,size);
  }
private:
#ifdef GRIPPER_FEEDBACK_PIN
  JawPositionPI pi_;
  bool positionMode_=false;
  int adc_=-1;
  bool applyPosition(uint32_t elapsedMs) {
    adc_=io_.positionAdc();
    int next;
    if(!pi_.step(target_,adc_,elapsedMs,next)) {off();return false;}
    if(!attached_) {io_.start(next);attached_=true;}
    else if(next!=current_) io_.pulse(next);
    current_=next;
    return true;
  }
#endif
  char side_; Output& io_;bool armed_=false,attached_=false;
  uint32_t token_=0,seq_=0,lastCommand_=0,lastStep_=0,remainder_=0;int low_=0,high_=0,open_=0,rate_=0,current_=0,target_=0;
  void reject(char* out,size_t n){off();snprintf(out,n,"ERR");}
  static bool number(const char* s,uint32_t& v) {
    if(!s||!*s)return false;v=0;
    while(*s) {if(*s<'0'||*s>'9')return false;uint32_t digit=*s++-'0';
      if(v>(UINT32_MAX-digit)/10)return false;v=v*10+digit;}return true;
  }
};

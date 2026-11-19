#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Shared by the AVR sketch and the host-side protocol verification harness.
template<class Output> class GripperProtocol {
public:
  explicit GripperProtocol(char side, Output& output): side_(side), io_(output) {}
  void off() {
    io_.off(); armed_=false; attached_=false; token_=seq_=0;
  }
  void update(uint32_t now) {
    if (!armed_) return;
    if ((uint32_t)(now-lastCommand_) >= 300) { off(); return; }
    uint32_t dt=now-lastStep_;
    if (!attached_ || dt<20) return;
    lastStep_=now;
    uint32_t budget=dt*rate_+remainder_;
    int step=(int)(budget/1000); remainder_=budget%1000;
    int d=target_-current_;
    if (d>step) d=step; if (d< -step) d=-step;
    if (d) {current_+=d;io_.pulse(current_);}
  }
  void line(char* text, uint32_t now, char* reply, size_t size) {
    update(now);
    char* args[8]; int n=0; char* ctx=0;
    for(char* t=strtok_r(text," ",&ctx);t;t=strtok_r(0," ",&ctx)) {
      if(n==8){reject(reply,size);return;}args[n++]=t;
    }
    if(n==1 && !strcmp(args[0],"ID?")) {off();snprintf(reply,size,"E6G1 %c",side_);return;}
    if(n==1 && !strcmp(args[0],"OFF")) {off();snprintf(reply,size,"OFF");return;}
    uint32_t key=0,a=0,b=0,r=0;
    if(n==6 && !strcmp(args[0],"ARM") && !armed_ && (side_=='L'||side_=='R') &&
        args[1][0]==side_ && !args[1][1] && number(args[2],key) && key &&
        number(args[3],a) && number(args[4],b) && number(args[5],r) &&
        a>=1000 && a<=2000 && b>=1000 && b<=2000 && abs((int)a-(int)b)>=20 && r>=20 && r<=1000) {
      token_=key;low_=a<b?a:b;high_=a>b?a:b;open_=b;rate_=r;seq_=0;remainder_=0;
      lastCommand_=lastStep_=now;armed_=true;attached_=false;
      snprintf(reply,size,"ARMED %lu",(unsigned long)key);return;
    }
    if(n==4 && !strcmp(args[0],"SET") && armed_ && number(args[1],key) && key==token_ &&
       number(args[2],a) && a>seq_ && number(args[3],b) && b>=low_ && b<=high_ && (attached_ || b==open_)) {
      lastCommand_=now;seq_=a;target_=(int)b;
      if(!attached_) {current_=target_;lastStep_=now;io_.start(current_);attached_=true;}
      snprintf(reply,size,"ACK %lu %lu %u",(unsigned long)key,(unsigned long)a,(unsigned)b);return;
    }
    reject(reply,size);
  }
private:
  char side_; Output& io_;bool armed_=false,attached_=false;
  uint32_t token_=0,seq_=0,lastCommand_=0,lastStep_=0,remainder_=0;int low_=0,high_=0,open_=0,rate_=0,current_=0,target_=0;
  void reject(char* out,size_t n){off();snprintf(out,n,"ERR");}
  static bool number(const char* s,uint32_t& v) {
    if(!s||!*s)return false;v=0;
    while(*s) {if(*s<'0'||*s>'9')return false;uint32_t digit=*s++-'0';
      if(v>(UINT32_MAX-digit)/10)return false;v=v*10+digit;}return true;
  }
};

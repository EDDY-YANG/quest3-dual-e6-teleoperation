#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

// The sensor measures jaw travel. 0 = closed, 1 = open; either ADC or PWM
// direction may be reversed. Gains are us per unit of normalized position error.
class JawPositionPI {
public:
  bool configure(int closedUs, int openUs, int closedAdc, int openAdc,
                 uint16_t kp, uint16_t ki, int maxTrimUs) {
    const int spanUs = abs(openUs-closedUs);
    if (!spanUs || closedAdc < 1 || closedAdc > 1022 || openAdc < 1 || openAdc > 1022 ||
        closedAdc == openAdc || !kp || maxTrimUs <= 0 || maxTrimUs > spanUs) return false;
    closedUs_=closedUs; openUs_=openUs; closedAdc_=closedAdc; openAdc_=openAdc;
    kp_=kp; ki_=ki; trim_=maxTrimUs;
    reset();
    return true;
  }
  void reset() { integral_=0; previousError_=0; }

  bool valid(int adc) const {
    const int margin=2 + abs(openAdc_-closedAdc_)/50;
    const int low=closedAdc_<openAdc_ ? closedAdc_ : openAdc_;
    const int high=closedAdc_>openAdc_ ? closedAdc_ : openAdc_;
    return adc>0 && adc<1023 && adc>=low-margin && adc<=high+margin;
  }
  bool step(int targetUs, int adc, uint32_t elapsedMs, int &outputUs) {
    if (!valid(adc)) { reset(); return false; }
    // A release replaces any previous closing effort immediately.
    if (targetUs==openUs_ || targetUs==closedUs_) { reset(); outputUs=targetUs; return true; }
    const int span=abs(openUs_-closedUs_);
    const int sign=openUs_>closedUs_ ? 1 : -1;
    const float ff=sign*(targetUs-closedUs_);
    const float measured=float(adc-closedAdc_)/(openAdc_-closedAdc_);
    const float error=ff/span-measured;
    if (error*previousError_<0 || elapsedMs>100) integral_=0;
    if (error!=0) previousError_=error;
    const float low=fmaxf(-trim_,-ff), high=fminf(trim_,span-ff);
    const float prop=kp_*error;
    if (elapsedMs && elapsedMs<=100) {
      // Conditional integration, including the trim limit and hard PWM endpoints.
      const float change=ki_*error*(elapsedMs*.001f);
      const float effort=prop+integral_;
      if (change>0 && effort<high) integral_+=fminf(change,high-effort);
      if (change<0 && effort>low) integral_+=fmaxf(change,low-effort);
      integral_=fmaxf(-trim_,fminf(trim_,integral_));
    }
    const float correction=fmaxf(low,fminf(high,prop+integral_));
    outputUs=closedUs_+sign*(int)lroundf(ff+correction);
    return true;
  }
private:
  int closedUs_=0,openUs_=0,closedAdc_=0,openAdc_=0,trim_=0;
  uint16_t kp_=0,ki_=0;
  float integral_=0,previousError_=0;
};

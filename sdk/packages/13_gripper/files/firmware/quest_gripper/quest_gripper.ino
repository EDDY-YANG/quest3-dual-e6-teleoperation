// Arduino Uno AVR, one board and one positional servo per arm. Set side before upload.
#include <Servo.h>
#include "gripper_protocol.h"
#ifndef GRIPPER_SIDE
#define GRIPPER_SIDE 0  // 1 = left, 2 = right; 0 reports X and refuses to arm.
#endif
const byte signalPin = 9;
const char side = GRIPPER_SIDE == 1 ? 'L' : GRIPPER_SIDE == 2 ? 'R' : 'X';
struct ServoOutput {
  Servo servo;
  void off() { if (servo.attached()) servo.detach(); digitalWrite(signalPin,LOW); }
  void start(int us) {
    noInterrupts();
    servo.attach(signalPin,1000,2000);
    servo.writeMicroseconds(us);
    interrupts();
  }
  void pulse(int us) { servo.writeMicroseconds(us); }
} output;
GripperProtocol<ServoOutput> control(side,output);
char buffer[80];byte used=0;bool discard=false;
void setup() { pinMode(signalPin,OUTPUT);output.off();Serial.begin(115200); }
void loop() {
  control.update(millis());
  // Bounded input processing keeps the independent watchdog and slew limiter serviced.
  for (byte i=0;i<32 && Serial.available();++i) {
    char c=(char)Serial.read();
    if(c=='\r')continue;
    if(c=='\n') {
      if(discard){control.off();Serial.println(F("ERR"));}
      else {buffer[used]=0;char reply[48];control.line(buffer,millis(),reply,sizeof(reply));Serial.println(reply);}
      used=0;discard=false;
    } else if(c<32 || c>126 || used>=sizeof(buffer)-1) {discard=true;control.off();}
    else if(!discard) buffer[used++]=c;
  }
}

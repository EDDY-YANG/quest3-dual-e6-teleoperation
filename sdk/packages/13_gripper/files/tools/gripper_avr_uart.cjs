// Dev-only UART/GPIO instrumentation of compiled Uno Intel HEX. No board access.
const fs=require("node:fs"), readline=require("node:readline");
const {CPU,avrInstruction,AVRIOPort,AVRTimer,AVRUSART,AVRADC,adcConfig,portBConfig,portCConfig,portDConfig,timer0Config,timer1Config,timer2Config,usart0Config}=require(process.env.AVR8JS_PATH||"avr8js");
const flash=new Uint16Array(16384),bytes=new Uint8Array(flash.buffer);
let upper=0;
for(const text of fs.readFileSync(process.argv[2],"utf8").trim().split(/\r?\n/)){
  if(!text.startsWith(":"))throw Error("Invalid HEX");const rec=Buffer.from(text.slice(1),"hex");
  if(rec.reduce((a,b)=>a+b,0)%256 || rec.length!==rec[0]+5)throw Error("HEX checksum");
  const type=rec[3],count=rec[0],offset=rec.readUInt16BE(1);
  if(type===0)bytes.set(rec.subarray(4,4+count),upper+offset);
  else if(type===4)upper=rec.readUInt16BE(4)*65536;
  else if(type===2)upper=rec.readUInt16BE(4)*16;
}
const cpu=new CPU(flash),timers=[timer0Config,timer1Config,timer2Config].map(t=>new AVRTimer(cpu,t));
const portB=new AVRIOPort(cpu,portBConfig),portC=new AVRIOPort(cpu,portCConfig),portD=new AVRIOPort(cpu,portDConfig);
const uart=new AVRUSART(cpu,usart0Config,16e6);let replies=[],samples=[],start=null;
uart.onLineTransmit=line=>replies.push(line.trim());
const adc=new AVRADC(cpu,adcConfig);
const pin=Number(process.argv[3]||9);
if(pin!==2 && pin!==9)throw Error("This UART/GPIO test supports Uno D2 and D9");
const port=pin===2?portD:portB, mask=pin===2?4:2;
port.addListener((v,prev)=>{if((v^prev)&mask){if(v&mask)start=cpu.cycles;else if(start!==null){samples.push({at_us:cpu.cycles/16,width_us:(cpu.cycles-start)/16});start=null;}}});
function advance(ms){const end=cpu.cycles+ms*16000;while(cpu.cycles<end){avrInstruction(cpu);cpu.tick();}}
advance(15);
readline.createInterface({input:process.stdin}).on("line",text=>{
  const msg=JSON.parse(text);replies=[];samples=[];
  if(msg.adc!==undefined) adc.channelValues[0]=msg.adc*adc.avcc/1024;
  if(msg.line!==undefined){
    for(const c of Buffer.from(msg.line+"\n","ascii")){if(!uart.writeByte(c))throw Error("UART RX unavailable");advance(.15);}
    // Wait for the complete transmitted line. Decimal nonce lengths affect UART time.
    const deadline=cpu.cycles+16000*100;
    while(!replies.length && cpu.cycles<deadline)advance(.2);
  }
  if(msg.ms!==undefined)advance(msg.ms);
  console.log(JSON.stringify({reply:replies.join("\n"),pulses:samples,now_us:cpu.cycles/16}));
});

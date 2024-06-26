

#include <EEPROM.h>
// #include <Controllino.h> 
#include <OSCMessage.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <avr/wdt.h>

//######### eNCODER CODE

#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// Define rotary encoder pins
#define ENC_A 2
#define ENC_B 3
#define setto0 4

unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 10;

volatile int counter = 0;
volatile int sw = 0;
volatile int lastsw = 0;
char  ulastsw = "";
int m = 1;
//######### eNCODER CODE end  ####################

void(* resetFunc) (void) = 0;                                      // declare reset fuction at address 0 for software reset 

int count; int addr = 2000; int addrb = 2001; int a; int b;                          // for alive time calculation
int restart; int addrestart = 2002;
int period = 1000;
unsigned long time_now = 0;


//TCCR0B = TCCR0B & B11111000 | B00000001;  // for PWM frequency of 62500 Hz

IPAddress ip(172,16,99,13);                                                             //the Arduino's IP
const unsigned int inPort = 7000;                                                        //Arduino's input Port
IPAddress myDns(172, 16, 99, 1);
IPAddress gateway(172, 16, 99, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress outIp(255,255,255,255);                                                     //destination IP multicast/broadcast
const unsigned int outPort = 11000;                                                     //destination Port
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xED, 0xED};                                     //mac address 
EthernetUDP Udp;
OSCErrorCode error;

//@#$%^@#$^%

int mig_send_int = 856; 

// #@$%@#$%
void setup() {
 //TCCR3B = TCCR3B & B11111000 | B00000001;   // for PWM frequency of 31372.55 Hz
  wdt_enable(WDTO_8S);                                                               // Auto reset after 8 seconds if hang
  Ethernet.begin(mac, ip);
  Udp.begin(inPort);

           a = EEPROM.read(addr);
           b = EEPROM.read(addrb);
           count=((b*255)+a);
           restart = EEPROM.read(addrestart);
           EEPROM.write(2*restart-1, a);EEPROM.write(2*restart, b);                   // store last alive time in eeprom
           a= 0; b=0;
           EEPROM.write(addr, a);EEPROM.write(addrb, b);
           restart = restart + 1 ;
           EEPROM.write(addrestart, restart);                                           
           delay(10);
          
  Serial.begin(115200);
  Serial.println("created by Eventagrate Group");
  Serial.println("www.eventagrate.com");
  Serial.println("Booting .....");
  delay(200);
  // Serial.println("Controllino fan pin  used Do0, Do1, Do2, Do3, Do4, Do5, Do6 ");
  // delay(200); 
  // Serial.println("Send osc signal at ip address : ");
  // Serial.println(ip); Serial.println("Port Number : "); Serial.println(inPort);
  // Serial.println("Send osc signal /fan1 value(0-100)speed for fan1 on "); 
  // Serial.println("Send osc signal /fan1 0(zero) for plug1 off  ");
  // Serial.println("Same for fan2-7 ");
 

/////// ENCODER CODE
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(setto0, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);


  // Start the serial monitor to show output
  // Serial.begin(115200);

 }

void loop() {   
  osc_receive();
  if(millis() > time_now + period){
       time_now = millis();
       alive();
        }

// ################################## Encoder start ##################################

    static int lastCounter = 0;

  // If count has changed print the new value to serial
  if(counter != lastCounter){
    Serial.println(counter);
    posi();
  
    if(lastCounter < counter){
      p1();
      // lastCounter = counter;
    }
     if(lastCounter > counter){
      p2();
      // lastCounter = counter;
    }
      lastCounter = counter;
  }

// ################################## Encoder end ##################################

// ################################## switch start ##################################

sw = digitalRead(setto0);

  if(sw == 1){
   
    Serial.println("on Miguel");
    if (m==1){
    sw_press();
    m=0;
    
    }
   }
   
  if(sw == 0){
 
    if (m==0){
    sw_release();
    m=1;
    }
   }


// ################################## switch end ##################################
     
}



void alive(){                                                         // alive function send counting every second

  Ethernet.begin(mac, ip);
OSCMessage msg("/stats/alive");
  msg.add(count);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  a=a+1;
EEPROM.write(addr, a);
 if(a>254){
   b=b+1; a=0;EEPROM.write(addr, a);EEPROM.write(addrb, b);
   }
  a = EEPROM.read(addr);
  b = EEPROM.read(addrb);
  count=((b*255)+a);
  if (count>9000){
      a=0;     b=0;
EEPROM.write(addr, a);EEPROM.write(addrb, b);
resetFunc(); //call reset
 }
}


// ############################### Encoder code ##################

void p1(){                                                          
  Ethernet.begin(mac, ip);
OSCMessage msg("/change");
  msg.add("1");
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
} 

void p2(){                                                          
  Ethernet.begin(mac, ip);
OSCMessage msg("/change");
  msg.add("-1");
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
} 

void sw_press(){                                                          
  Ethernet.begin(mac, ip);
OSCMessage msg("/sw");
  msg.add("on");
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}  
void sw_release(){                                                          
  Ethernet.begin(mac, ip);
OSCMessage msg("/sw");
  msg.add("off");
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();

}  
void posi(){                                                          
  Ethernet.begin(mac, ip);
OSCMessage msg("/pos");
  msg.add(counter);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  
}  

void mig_send(){
    Ethernet.begin(mac, ip);
  OSCMessage msg("/mig");
  msg.add(mig_send_int);
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  
}


void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
} 



void info()                                                                                 // send info over osc
{
  restart = EEPROM.read(addrestart);
  OSCMessage msg("/stats/info");
  msg.add(restart); 
  msg.add("reset1");
  int low = EEPROM.read(2*(restart-1)-1);
  int high = EEPROM.read(2*(restart-1));
  int last =((high*255)+low);
  msg.add(last);
  msg.add("reset2");
  low = EEPROM.read(2*(restart-1)-3);
  high = EEPROM.read((2*(restart-1))-2);
  last =((high*255)+low);
  msg.add(last);
  msg.add("reset3");
  low = EEPROM.read(2*(restart-1)-5);
  high = EEPROM.read((2*(restart-1))-4);
  last =((high*255)+low);
  msg.add(last);
  // msg.add('fan1_speed');
  
  msg.add("inport 6000, outport 11000");
  Udp.beginPacket(outIp, outPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
  }


void info(OSCMessage &msg) {                                                      // info function
  info();  
 }
void Reset1(OSCMessage &msg) {                                                  // reset function
  resetFunc();
 }

void mig(OSCMessage &msg) {                                                  // reset function
    mig_send_int = msg.getInt(0);

  Serial.print(msg.getInt(0));
   Serial.print("     ");
  Serial.println(mig_send_int + 23);
  mig_send();
 }
 


void osc_receive() {                                                          // osc receive function
  wdt_reset(); 
OSCMessage recMsg;

  int size = Udp.parsePacket();
  if (size > 0)
  {
    while (size--) {
      recMsg.fill(Udp.read());
      //Serial.println('OSCData');
    }
    if (!recMsg.hasError()) {
     recMsg.dispatch("/screen/pos", posi);
     recMsg.dispatch("/stats/reset", Reset1);
     recMsg.dispatch("/screenmig", mig);
      } 
  }
}

// //  ###########  FAN 1 ###########

//  void fan1_on(OSCMessage &msg) {                                                                  //fan 1 on function
//     wdt_reset();
//     fan1_time = 3;  
//   //  fan1_speed = msg.getInt(0);
//     fan1_speed= 150;
//     analogWrite(fan1, fan1_speed);
//      fan1_on_status();
//      Serial.println(fan1_speed);
//          }

// void fan1_off_status(){                                                          
//   Ethernet.begin(mac, ip);
// OSCMessage msg("/fan1");
//   msg.add("off");
//   Udp.beginPacket(outIp, outPort);
//   msg.send(Udp);
//   Udp.endPacket();
//   msg.empty();
// }  


// void fan1_on_status(){                                                          
//   Ethernet.begin(mac, ip);
// OSCMessage msg("/fan1");
//   msg.add("on");
//   Udp.beginPacket(outIp, outPort);
//   msg.send(Udp);
//   Udp.endPacket();
//   msg.empty();
// }  

// // ***********************************************************************************


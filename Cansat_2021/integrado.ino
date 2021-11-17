#include <Adafruit_GPS.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <ThreeWire.h>  
#include <RtcDS1302.h>
#include <EEPROM.h>

#define EEPROM_Servo1low 0
#define EEPROM_Servo1high 1
#define EEPROM_Servo2low 2
#define EEPROM_Servo2high 3
ThreeWire myWire(3,4,2); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);
HardwareSerial mySerial = Serial3;
Adafruit_GPS GPS(&Serial3);
MPU9250 IMU(Wire,0x68);
Adafruit_BMP280 bmp;
Servo Servo1;
Servo Servo2;
RtcDateTime now;

double testpressure[139]={100833,100833,100833,100833,100833,100833,100833,100833,100833,100833,100833,100833,100833,98522,97353,96081,95049,94231,93621,93137,92812,92578,92645,92868,92756,92924,92913,93059,93115,93216,93250,93384,93441,97539,93655,93700,93790,93880,93948,94072,94378,94231,94299,94378,94503,94617,94651,94810,96334,94947,95015,95118,95278,95323,98264,95495,95587,95678,95782,95862,96127,96023,96069,96150,96207,96288,96357,96438,96519,96623,96669,96738,96831,96900,96981,97121,97179,97190,97295,97341,97458,97598,97621,97563,97609,97691,97761,97796,97843,97889,97936,98018,98393,104180,98287,98240,98393,98393,98440,98522,98616,98639,98698,98745,98816,98875,98957,99075,99512,99264,99299,100523,99406,99489,99571,99702,99749,99809,99880,99963,100034,100129,100368,100463,101927,100487,100582,100606,100726,100762,100821,100893,100893,100893,100893,100893,100893,100893};

int auxsimcounter;
unsigned long Time;
int status;
String sentencia,altitude,Pressure,TEMP,SOFTWARE_STATE,CMD_ECHO,groundrx,payload1rx,payload2rx,groundtx,p1rx,p1tx,p2rx,p2tx,realtime,gpsaltitude;
float calibratealtitude;
double voltage,simulated_pressure;
int PACKET_COUNT,SP1_PACKET_COUNT,SP2_PACKET_COUNT;
char MODE,aux;
String SP1_RELEASED,SP2_RELEASED;
int alt,altprom,counter,servopos,servopos2,servo2pos,servo2pos2,maxaltitude;
int altbuffer[3];
bool telemetry,simulation,simenable;
int Servo1low,Servo1high,Servo2low,Servo2high;
///////////XBEE
byte containeraddress[2]={0x01,0x11};
byte payload1address[2]={0x00, 0x11};
byte payload2address[2]={0x00, 0x12};
byte groundaddress[2]={0x00, 0x13};
byte rxbuff[255];
byte rxaux=0;
byte rxchcksum;
String rxdata;
bool rxpacket,newpacket;
//////////




void setup() {
  Serial.begin(9600);
  rxpacket=false;
  Serial2.begin(19200);
  Servo1.attach(9);
  Servo2.attach(8);
  Servo1low=EEPROM.read(EEPROM_Servo1low);
  Servo1high=EEPROM.read(EEPROM_Servo1high);
  Servo2low=EEPROM.read(EEPROM_Servo2low);
  Servo2high=EEPROM.read(EEPROM_Servo2high); 
  Servo1.write(Servo1high);
  Servo2.write(Servo2high);
  counter=0;
SOFTWARE_STATE="BOOT";
 //GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//



//////////////////////////////////////////bmp280
bmp.begin();
  /* Default settings from datasheet. */
bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
calibratealtitude=1;
while (bmp.readAltitude(calibratealtitude)<0) {
calibratealtitude=calibratealtitude+1;
                           }
while (bmp.readAltitude(calibratealtitude)>=0) {
calibratealtitude=calibratealtitude-0.1;
                           }
while (bmp.readAltitude(calibratealtitude)<=0) {
calibratealtitude=calibratealtitude+0.001;
                           }
while (bmp.readAltitude(calibratealtitude)>=0) {
calibratealtitude=calibratealtitude-0.0001;
                             }       
//////////////////////////////////////////bmp280


//IMU
status = IMU.begin();
//

//RTC
Rtc.Begin();

//
Time=0;
telemetry=false;
MODE='F';
SP1_RELEASED="N";
SP2_RELEASED="N";
SP1_PACKET_COUNT=0;
SP2_PACKET_COUNT=0;

SOFTWARE_STATE="LAUNCH_WAIT";
auxsimcounter=0;
maxaltitude=0;
Serial.println("START");
simenable=false;
}



void loop() {





 if (Serial.available() > 0) {
  
groundrx=Serial.readString();
 groundrx.trim();
 while(Serial.available() > 0){
  aux=Serial.read();
        }
 }

  

//CMD,1000,SIM,ENABLE
if ((groundrx=="CMD,1231,SIM,ENABLE")){
  simenable=true;
  CMD_ECHO="SIM_ENABLE";
                                       }






//CMD,1000,SIM,ACTIVATE
if ((groundrx=="CMD,1231,SIM,ACTIVATE") && (simenable==true)){
//      if((simenable==true)){
//        Serial.println("TRUE");
//      } else {
//        Serial.println("FALSE");
//      }
  
  simulated_pressure=101325;
  simulation = true;
  MODE='S';
  CMD_ECHO="SIM_ACTIVATE";
                                                             }


if (groundrx=="XX"){
  Serial.println("YY");
}


//CMD,<TEAM_ID>,SP1X,<ON_OFF>
if ((groundrx=="CMD,1231,SP1X,ON")){
  tx("CMD,1231,SP1X,ON",payload1address);
  CMD_ECHO="SP1X_ON";
                                   }
if ((groundrx=="CMD,1231,SP1X,OFF")){
  tx("CMD,1231,SP1X,ON",payload1address);
  CMD_ECHO="SP1X_OFF";
                                   }

if ((groundrx=="CMD,1231,SP2X,ON")){
  tx("CMD,1231,SP2X,ON",payload2address);
  CMD_ECHO="SP2X_ON";
                                   }
if ((groundrx=="CMD,1231,SP2X,OFF")){
  tx("CMD,1231,SP2X,ON",payload2address);
  CMD_ECHO="SP2X_OFF";
                                   }

    


//CMD,1231,SIMP,<PRESSURE>
if ((groundrx.substring(0,14)=="CMD,1231,SIMP,")){  
  Serial.println(groundrx.substring(14,groundrx.length()));
    simulated_pressure=(groundrx.substring(14,groundrx.length())).toDouble();
    Serial.print("RX PRESSURE");
    Serial.println(String(simulated_pressure));
  }

if (groundrx=="CMD,1231,SIM,DISABLE"){


  simenable=false;
  
  simulation = false;
  MODE='F';
  CMD_ECHO="SIM_DISABLE";
}

if (groundrx=="CMD,1231,CX,ON"){
  telemetry = true;
  CMD_ECHO="CXON";
}


if (groundrx=="CMD,1231,CX,OFF"){
  telemetry = false;
  CMD_ECHO="CXOFF";
}

if (groundrx.substring(0,11)=="CMD,1231,ST"){
                       //CMD,1000,ST,13:35:59
//                       date.Hour(groundrx.substring(12,14));
//                       date.Minute=groundrx.substring(16,17);
//                       date.Second=groundrx.substring(15,17);
                       RtcDateTime now = Rtc.GetDateTime();
                       realtime=printDateTime(now);
                       //05/29/2021 02:21:00
                       //Rtc.SetDateTime(RtcDateTime(2018,1,17,7,53,00));
                       Serial.println(realtime);
                       Serial.println(realtime.substring(6,10)+","+realtime.substring(0,2)+","+realtime.substring(3,5)+","+realtime.substring(11,13)+","+realtime.substring(14,16)+","+realtime.substring(realtime.length()-2,realtime.length()));
                       //String conversiondate;
                       //conversiondate=realtime.substring(6,10).toInt()+","+realtime.substring(0,2).toInt()+","+realtime.substring(3,5).toInt()+","+realtime.substring(11,13).toInt()+","+realtime.substring(14,16).toInt()+","+realtime.substring(realtime.length()-2,realtime.length()).toInt()
                       Serial.println((realtime.substring(6,10)+","+realtime.substring(0,2)+","+realtime.substring(3,5)+","+groundrx.substring(12,14)+","+groundrx.substring(15,17)+","+groundrx.substring(realtime.length()-1,realtime.length()+1)));
                       Rtc.SetDateTime(RtcDateTime(realtime.substring(6,10).toInt(),realtime.substring(0,2).toInt(),+realtime.substring(3,5).toInt(),+groundrx.substring(12,14).toInt(),groundrx.substring(15,17).toInt(),groundrx.substring(realtime.length()-1,realtime.length()).toInt()+1));
                       
CMD_ECHO="ST";
}


//SERVO1,POS,120
if (groundrx.substring(0,11)=="SERVO1,POS,"){
  Servo1.write((groundrx.substring(11,groundrx.length())).toInt());
  Serial.println(groundrx.substring(11,groundrx.length()));
}
if (groundrx.substring(0,11)=="SERVO2,POS,"){
  Servo2.write((groundrx.substring(11,groundrx.length())).toInt());
}


if (groundrx.substring(0,12)=="SERVO1,LSET,"){
  //EEPROM.write(EEPROM_Servo1low,toInt(groundrx.substring(12,groundrx.length())))
  Serial.println(groundrx.substring(12,groundrx.length()));
}
if (groundrx.substring(0,12)=="SERVO1,HSET,"){
  EEPROM.write(EEPROM_Servo1high,(   groundrx.substring( 12,groundrx.length() ).toInt()  ));
}


if (groundrx.substring(0,12)=="SERVO2,LSET,"){
  EEPROM.write(EEPROM_Servo2low,(   groundrx.substring( 12,groundrx.length() ).toInt()  ));
}
if (groundrx.substring(0,12)=="SERVO2,HSET,"){
  EEPROM.write(EEPROM_Servo2high,(   groundrx.substring( 12,groundrx.length() ).toInt()  ));
}

  
if(SOFTWARE_STATE=="LAUNCH_WAIT"){
  
                                  if (altprom >= 10){
                                    SOFTWARE_STATE="ASCENT";
                                                     } 
  
}


if(SOFTWARE_STATE=="ASCENT"){
                                  
                                  if ((altitude.toInt())<=(altprom)){
                                    
                                  }
  
}
if (  ( altprom <=(maxaltitude - 10) ) && SOFTWARE_STATE=="ASCENT"){
  SOFTWARE_STATE="ROCKET_SEPARATION";
  counter=0;
}

if(SOFTWARE_STATE=="ROCKET_SEPARATION"){
                                  if ((altitude.toInt())<=(maxaltitude-10)){
                                   
                                  }
  
}


if ( ( altprom <=(maxaltitude - 20) )&& SOFTWARE_STATE=="ROCKET_SEPARATION"){
  SOFTWARE_STATE="DESCENT";
  counter=0;
}


if (SOFTWARE_STATE=="DESCENT" && (altprom<=500)){
  SOFTWARE_STATE="SP1_RELEASE";
  Servo1.write(Servo1low);
  SP1_RELEASED="R";
}

if (SOFTWARE_STATE=="SP1_RELEASE" && (altprom<=400)){
  SOFTWARE_STATE="SP2_RELEASE";
  Servo2.write(Servo2low);
  SP2_RELEASED="R";
}


if (SOFTWARE_STATE=="SP2_RELEASE" && (altprom<=40)){
  SOFTWARE_STATE="LANDED";
}

  // GPS

   if (Serial3.available() > 0) {

char c = GPS.read();
  if(c == 0)return;
  //Serial.print(c);
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }


    
  }
     
//




  
//if (simulation==true){
//  //altitude=String(simulated_altitude(simulated_pressure,101325));
//  simulated_pressure=testpressure[auxsimcounter];
//  altprom=(altprom+simulated_altitude(simulated_pressure,101325))/2;
//  altitude=String(altprom);
//if (altprom > maxaltitude){
//                          maxaltitude=altprom;
//                          }
//  
//} else{
//        
//        altprom=(int(bmp.readAltitude(calibratealtitude))+altprom)/2; 
//        altitude=String(altprom);
//      }






if (telemetry==true && ((Time+1000)<=millis())){



if (simulation==true){
  altitude=String(simulated_altitude(simulated_pressure,101325));
  //simulated_pressure=testpressure[auxsimcounter];
  //simulated_pressure=
  altbuffer[(auxsimcounter)%4]=simulated_altitude(simulated_pressure,101325);
  //altprom=(altprom+simulated_altitude(simulated_pressure,101325))/2;
  altprom=altbuffer[0]+altbuffer[1]+altbuffer[2]+altbuffer[3];
  altprom=altprom/4;
                                                                       
  
  
  altitude=String(altprom);
if (altprom > maxaltitude){
                          maxaltitude=altprom;
                          }
  
} else{
        
        altprom=(int(bmp.readAltitude(calibratealtitude))+altprom)/2; 
        altitude=String(altprom);
      }










  
Time = millis();
RtcDateTime now = Rtc.GetDateTime();
TEMP=String(bmp.readTemperature());
TEMP.remove(TEMP.length() - 1,TEMP.length());
voltage=(analogRead(A1)*5.0/1023)*2;
realtime=printDateTime(now);
//         <TEAM_ID>,   <MISSION_TIME>,<PACKET_COUNT>,<PACKET_TYPE>,<MODE>,  <SP1_RELEASED>,     <SP2_RELEASED>,     <ALTITUDE>,     <TEMP>,   <VOLTAGE>,           <GPS_TIME>,                                                         <GPS_LATITUDE>,                 <GPS_LONGITUDE>,              <GPS_ALTITUDE>,              <GPS_SATS>,  <SOFTWARE_STATE>,      <SP1_PACKET_COUNT>,               <SP2_PACKET_COUNT>,       <CMD_ECHO>
//          1231,05/31/2021 16:54:36,14,C,F,FALSE,FALSE,0,28.8,2.90,19:53:22,2446.9387,6524.3452,1190.90,5,LAUNCH_WAIT,0,0,CXON
gpsaltitude=String(GPS.altitude);
//gpsaltitude="1190.90";

sentencia="1231,"+realtime.substring(realtime.length()-8,realtime.length())+","+String(PACKET_COUNT)+",C,"+MODE+","+String(SP1_RELEASED)+","+String(SP2_RELEASED)+","+altitude+","+TEMP+","+voltage+","+   String(GPS.hour)+":"+String(GPS.minute)+":"+String(GPS.seconds)+","+String(GPS.latitude, 4)+","+String(GPS.longitude, 4)+","+gpsaltitude.substring(0,gpsaltitude.length()-1)+","+String((int)GPS.satellites)+","+ SOFTWARE_STATE       +","+String(SP1_PACKET_COUNT)+","+String(SP2_PACKET_COUNT)+","+CMD_ECHO;
PACKET_COUNT ++;
Serial.println(sentencia);
if (simulation==true ){
auxsimcounter++;    
                      }


///////////////////////XBEE  TX
tx(sentencia,groundaddress);
///////////////////////////
}

groundrx="";


////////////////////XBE RX

if (Serial2.available() > 0) {
  
  byte rx;
  rx=Serial2.read();
  if (rx==0x7E){
                rxaux=0;
                memset(rxbuff, 0, sizeof(rxbuff));
                rxbuff[rxaux]=rx;
                rxaux++;
                }else{
                  rxbuff[rxaux]=rx;
                  rxaux++; 
                }
  
  if ((rxbuff[2]+4)==rxaux){
                            byte chkaux;
                            chkaux=0;
                                                            for (byte i=3; i<(rxaux-1); i++){
                                                                                            chkaux=chkaux+rxbuff[i];
                                                                                          }
                                                                                           rxchcksum=0xFF- (chkaux);
                       
                                                                             
                                                                              if (  ((rxbuff[rxaux]-1)!=rxchcksum) && (rxaux>=5)){
                                                                              rxdata="";
                                                                              for (byte i=8; i<(rxaux-1); i++){
                                                                              char output;
                                                                              rxdata=rxdata+(char)rxbuff[i];
                                                                                                                } 
                                                                                                                                    } 
                                                              
                           
                           
                           if ( (rxbuff[4]==0x00) && ((rxbuff[5]==0x13)) ){
                            groundrx=rxdata;
                                         }
                           

                           
                           if ( (rxbuff[4]==0x00) && ((rxbuff[5]==0x11)) ){
                            payload1rx=rxdata;
                            SP1_PACKET_COUNT++;
                              String toground,SP_ALTITUDE,SP_TEMP,SP_ROTATION_RATE;
                              int auxsp;
                              auxsp= rxdata.indexOf(',');
                              SP_ALTITUDE=rxdata.substring(0,auxsp);
                              SP_TEMP=rxdata.substring(auxsp+1,rxdata.indexOf(',', auxsp + 1 ));
                              SP_ROTATION_RATE=rxdata.substring(rxdata.indexOf(',', auxsp + 1 )+1,rxdata.length());
                              RtcDateTime now = Rtc.GetDateTime();
                              realtime=printDateTime(now);
                              String PACKET_TYPE;
                              if (SP1_RELEASED=="R"){
                                PACKET_TYPE="S2";
                                                        } else {
                                                          PACKET_TYPE="C";
                                                                }
                                      //<TEAM_ID>,<MISSION_TIME>,                                              <PACKET_COUNT>,          <PACKET_TYPE>,   <SP_ALTITUDE>,<SP_TEMP>,<SP_ROTATION_RATE>
                              //         1231        ,          15:57:29                                        ,,C,500.1,,40,,40
                              toground="1231," + realtime.substring(realtime.length()-8,realtime.length()) + "," + String(SP1_PACKET_COUNT) + "," + PACKET_TYPE + "," + SP_ALTITUDE + "," + SP_TEMP + "," + SP_ROTATION_RATE ;
                              tx(toground,groundaddress);
                                          }
                           if ( (rxbuff[5]==0x00) && ((rxbuff[6]==0x13)) ){
                              payload2rx=rxdata;
                              SP2_PACKET_COUNT++;
                              String toground,SP_ALTITUDE,SP_TEMP,SP_ROTATION_RATE;
                              int auxsp;
                              auxsp= rxdata.indexOf(',');
                              SP_ALTITUDE=rxdata.substring(0,auxsp);
                              SP_TEMP=rxdata.substring(auxsp,rxdata.indexOf(',', auxsp + 1 ));
                              SP_ROTATION_RATE=rxdata.substring(rxdata.indexOf(',', auxsp + 1 ),rxdata.length());
                              RtcDateTime now = Rtc.GetDateTime();
                              realtime=printDateTime(now);
                              String PACKET_TYPE;
                              if (SP1_RELEASED=="R"){
                                PACKET_TYPE="S1";
                                                        } else {
                                                          PACKET_TYPE="C";
                                                                }
                                      //<TEAM_ID>,<MISSION_TIME>,                                              <PACKET_COUNT>,          <PACKET_TYPE>,   <SP_ALTITUDE>,<SP_TEMP>,<SP_ROTATION_RATE>
                              toground="1231,"+realtime.substring(realtime.length()-8,realtime.length())+","+String(SP1_PACKET_COUNT+","+PACKET_TYPE+","+SP_ALTITUDE+","+SP_TEMP+","+SP_ROTATION_RATE);
                              tx(toground,groundaddress);
                                          }

//if ( (rxbuff[4]==0x00) && ((rxbuff[5]==0x12)) ){
//                            groundrx=rxdata;
//                                          }


                           
                           }
                           

   
//                            

   
}


/////////////////////

}



//FUNCTIONS
double simulated_altitude(double P, double P0)
// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
{
  return(44330.0*(1-pow(P/P0,1/5.255)));

}


#define countof(a) (sizeof(a) / sizeof(a[0]))

String printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

  snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
      return datestring;
}

void tx(String datatx,byte dir[2]){

  char bufferout[datatx.length()+9];
  bufferout[0] = 0x7E;
  bufferout[1] = 0x00;
  bufferout[2] = datatx.length()+5; 
  bufferout[3] = 0x01;
  bufferout[4] = 0x01;
  bufferout[5] = dir[0]; 
  bufferout[6] = dir[1];
  bufferout[7] = 0x00;
    char buff[datatx.length()+1];
    datatx.toCharArray(buff,datatx.length()+1);
    for (byte i = 0; i < datatx.length()+1; i++){
    bufferout[i+8]=buff[i];
                           }


  int chkaux;
  chkaux=0;
  for (byte i=3; i<(datatx.length()+8); i++){
                                   chkaux += bufferout[i] & 0xFF;
                                           }
byte chk;
chk=0xFF - chkaux;
  for (byte i = 0; i < (datatx.length()+8); i++){
    Serial2.write(bufferout[i] & 0xFF);
  }
  Serial2.write(chk);
}
     
  

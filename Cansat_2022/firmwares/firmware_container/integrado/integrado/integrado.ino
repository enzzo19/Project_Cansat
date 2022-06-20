#include <Adafruit_GPS.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <ThreeWire.h>  
#include <RtcDS1302.h>
#include <EEPROM.h>

#define EEPROM_Servo1low 0
#define EEPROM_Servo1high 1
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

byte auxsimcounter,payloadspeed;
unsigned long Time,Timep1,Timep2,Timep3,Timep4,Timep5,Timep6;
int status;
String sentencia,altitude,Pressure,TEMP,SOFTWARE_STATE,CMD_ECHO,groundrx,payload1rx,payload2rx,groundtx,p1rx,p1tx,p2rx,p2tx,realtime,gpsaltitude;
float calibratealtitude;
double voltage,simulated_pressure;
int PACKET_COUNT,SP1_PACKET_COUNT;
char MODE,aux;
String SP1_RELEASED;
double altprom,altpayload;
int alt,counter,servopos,servopos2,maxaltitude;
double altbuffer[4];
bool telemetry,simulation,simenable,p1released,descend,newaltitude;
int Servo1low,Servo1high;
///////////XBEE
byte containeraddress[2]={0x00,0x10};
byte payload1address[2]={0x00, 0x11};
byte groundaddress[2]={0x00, 0x13};
byte rxbuff[255];
byte rxaux=0;
byte rxchcksum;
String rxdata;
bool rxpacket,newpacket;
//int SP1_PACKET_COUNT;
//////////




void setup() {
  Serial.begin(9600);
  rxpacket=false;
  Serial2.begin(19200);
  Servo1.attach(9);
  Servo2.attach(8);
  Servo1low=EEPROM.read(EEPROM_Servo1low);
  Servo1high=EEPROM.read(EEPROM_Servo1high);
  Servo1.write(Servo1high);
  payloadspeed=90;
  Servo2.write(payloadspeed);
  newaltitude=false;
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
SP1_PACKET_COUNT=0;

SOFTWARE_STATE="LAUNCH_WAIT";
auxsimcounter=0;
maxaltitude=0;
Serial.println("START");
simenable=false;
p1released=false;
descend=false;
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
if ((groundrx=="CMD,1064,SIM,ENABLE")){
  simenable=true;
  CMD_ECHO="SIM_ENABLE";
                                       }






//CMD,1000,SIM,ACTIVATE
if ((groundrx=="CMD,1064,SIM,ACTIVATE") && (simenable==true)){
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
if ((groundrx=="CMD,1064,SP1X,ON")){
  tx("CMD,1064,SP1X,ON",payload1address);
  CMD_ECHO="SP1X_ON";
  Serial.println("P1ON");
                                   }
if ((groundrx=="CMD,1064,SP1X,OFF")){
  tx("CMD,1064,SP1X,OFF",payload1address);
  CMD_ECHO="SP1X_OFF";
  Serial.println("P1OFF");
                                   }


if ((groundrx=="CMD,1064,CAMERA,ON")){
  tx("CMD,1064,CAMERA,ON",payload1address);
  CMD_ECHO="CAM_ON";
  Serial.println("CAMON");
                                   }    

if ((groundrx=="CMD,1064,CAMERA,OFF")){
  tx("CMD,1064,CAMERA,OFF",payload1address);
  CMD_ECHO="CAM_OFF";
  Serial.println("CAMOFF");
                                   }


//CMD,1231,SIMP,92645
if ((groundrx.substring(0,14)=="CMD,1064,SIMP,")){  
    simulated_pressure=(groundrx.substring(14,groundrx.length())).toDouble();
  Serial.println(String(simulated_pressure));
  
  }

if (groundrx=="CMD,1064,SIM,DISABLE"){


  simenable=false;
  
  simulation = false;
  MODE='F';
  CMD_ECHO="SIM_DISABLE";
}

if (groundrx=="CMD,1064,CX,ON"){
  telemetry = true;
  CMD_ECHO="CX-ON";
}


if (groundrx=="CMD,1064,CX,OFF"){
  telemetry = false;
  CMD_ECHO="CX-OFF";
}
if (groundrx=="CMD,1064,GIMBALL,ON"){
  groundrx="";
  tx("CMD,1231,SP1X,ON",payload1address);
  Serial.println("GIMBALL on");
  
}
if (groundrx.substring(0,11)=="CMD,1064,ST"){
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
  //Serial.println(groundrx.substring(12,groundrx.length()));
  EEPROM.write(EEPROM_Servo1low,(   groundrx.substring( 12,groundrx.length() ).toInt()  ));
}
if (groundrx.substring(0,12)=="SERVO1,HSET,"){
  EEPROM.write(EEPROM_Servo1high,(   groundrx.substring( 12,groundrx.length() ).toInt()  ));
}


  
if(SOFTWARE_STATE=="LAUNCH_WAIT"){
  
                                  if (altprom >= 10.00){
                                    SOFTWARE_STATE="ASCENT";
                                                     } 
  
}


if(SOFTWARE_STATE=="ASCENT"){
                                  
                                  if ((altitude.toInt())<=(altprom)){
                                    
                                  }
  
}
if (  ( altprom <=(maxaltitude - 10.00) ) && SOFTWARE_STATE=="ASCENT"){
  SOFTWARE_STATE="ROCKET_SEPARATION";
  counter=0;
}

if(SOFTWARE_STATE=="ROCKET_SEPARATION"){
                                
  
}


if ( ( altprom <=(maxaltitude - 20.00) )&& SOFTWARE_STATE=="ROCKET_SEPARATION"){
  SOFTWARE_STATE="DESCENT";
  descend=true;
  counter=0;
}


if (SOFTWARE_STATE=="DESCENT" && (altprom<=500.00)){
  SOFTWARE_STATE="SECOND_PARACHUTE_RELEASE";
  Serial.println(String(Servo1low));
  Servo1.write(Servo1low);
  //SP1_RELEASED="R";
  //p1released=true;
  //Timep1=millis()+300000;
  //tx("CMD,1231,SP1X,ON",payload1address);
}
if ((p1released==true) && (millis()==Timep1)){
  tx("CMD,1064,SP1X,OFF",payload1address);
  Serial.println("OFF SP1");
}


if (SOFTWARE_STATE=="SECOND_PARACHUTE_RELEASE" && (altprom<=480.00)){
  tx("CMD,1064,GIMBALL,ON",payload1address);
  tx("CMD,1064,CAMERA,ON",payload1address);
}
if (SOFTWARE_STATE=="SECOND_PARACHUTE_RELEASE" && (altprom<=410.00)){
  tx("CMD,1064,CAMERA,ON",payload1address);
  Serial.println("XXXPASAAAAXXX");
}
if (SOFTWARE_STATE=="SECOND_PARACHUTE_RELEASE" && (altprom<=400.00)){
  SOFTWARE_STATE="PAYLOAD_RELEASE";
  Servo2.write(180);
  SP1_RELEASED="R";
  p1released=true;
  Timep1=millis();
  Timep2=Timep1+300000;
  Timep3=Timep1+20000;
  Timep4=Timep1;
  Timep5=Timep1;
  tx("CMD,1064,SP1X,ON",payload1address);
  
  Serial.println("WER");
}
if ((p1released==true) && (millis()==Timep2)){
  //tx("CMD,1064,SP1X,OFF",payload1address);
  Serial.println("OFF SP1");
}






if ( (SOFTWARE_STATE=="PAYLOAD_RELEASE") && (altprom<=10.00) ){
  SOFTWARE_STATE="LANDED";
  Serial.println(String(altprom));
  //telemetry=false;
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

 // altitude=String(simulated_altitude(simulated_pressure,101325));
  simulated_pressure=testpressure[auxsimcounter];
  
  //if (descend==true){
//  Serial.println("alt buff   "+String( altbuffer[0]) );
//  Serial.println("alt buff   "+String( altbuffer[1]) );
//  Serial.println("alt buff   "+String( altbuffer[2]) );
//  Serial.println("alt buff   "+String( altbuffer[3]) );
//  Serial.println("alt buff   "+String( altbuffer[4]) ); 
//  Serial.println("sim altitude        "+String( simulated_altitude(simulated_pressure,101325) ) );
//  Serial.println("counter   "+String(auxsimcounter));
//  

  if (simulated_altitude(simulated_pressure,101325)<0) {
                                                                if (descend==true){
                                                                  altbuffer[(auxsimcounter)%5]=altprom-10;
                                                                  Serial.println("XXX");
                                                                                  } else {
                                                                                     altbuffer[(auxsimcounter)%5]=altprom+10;
                                                                                     Serial.println("YYY");
                                                                                          }
    
                                                        }else{
                                                          
  altbuffer[(auxsimcounter)%5]=simulated_altitude(simulated_pressure,101325);
                                                        
                                                        
                                                        }
  
  Serial.println("alt buff");
//  Serial.println(String(altbuffer[0]));
//  Serial.println(String(altbuffer[1]));
//  Serial.println(String(altbuffer[2]));
//  Serial.println(String(altbuffer[3]));
//  Serial.println(String(altbuffer[4]));
  altprom=altbuffer[0]+altbuffer[1]+altbuffer[2]+altbuffer[3]+altbuffer[4];
  altprom=altprom/5.00;
  
  
//  altitude=String(altprom);
  Serial.println("altitude");
  Serial.println(String(altprom));
  //}
   
  
  
  
  
  
  
  
                      } else{
                          if (Timep4+100<millis()){
                          altprom=(int(bmp.readAltitude(calibratealtitude))+altprom)/2; 
                           altitude=String(altprom);
                                                 }
        
        
                                    } 
  
  
                                 



  
if (altprom > maxaltitude){
                          maxaltitude=altprom;
                          }
  

Time = millis();
RtcDateTime now = Rtc.GetDateTime();
TEMP=String(bmp.readTemperature());
TEMP.remove(TEMP.length() - 1,TEMP.length());
//voltage=analogRead(A1)*(1.0/68.0);
voltage=analogRead(A1)/52.413;
realtime=printDateTime(now);


//         TEAM_ID,       MISSION_TIME, PACKET_COUNT, PACKET_TYPE, MODE,TP_RELEASED, ALTITUDE, TEMP, VOLTAGE,  GPS_TIME, GPS_LATITUDE,    GPS_LONGITUDE, GPS_ALTITUDE, GPS_SATS, SOFTWARE_STATE, CMD_ECHO
//          1064  ,05/31/2021 16:54:36,           14,           C,    F,       FALSE,       0,  28.8,   2.90,  19:53:22,    2446.9387,6524.3452,1190.90,5,LAUNCH_WAIT,0,0,CXON

//            1000,23:52:02, 83,C,F,R,N,415,25.7,8.8,23:52:02,  37.2315,80.4265  ,418.2 ,7,RELEASE_1  ,74,0,CX-ON
//            1231,04:10:42,145,C,S,R,R,36 ,22.8,5.0, 7:10:24,2446.9368,6524.3423,1186.3,6,SP2_RELEASE,84,0,SIM_ACTIVATE

gpsaltitude=String(GPS.altitude);
//gpsaltitude="1190.90";
String volt;
volt=String(voltage);
volt=volt.substring(0,volt.length()-1);

  
sentencia="1064,"+realtime.substring(realtime.length()-8,realtime.length())+","+String(PACKET_COUNT)+",C,"+MODE+","+String(SP1_RELEASED)+","+altitude.substring(0,altitude.length()-3)+","+TEMP+","+volt+","+   String(GPS.hour)+":"+String(GPS.minute)+":"+String(GPS.seconds)+","+String(GPS.latitude, 4)+","+String(GPS.longitude, 4)+","+gpsaltitude.substring(0,gpsaltitude.length()-1)+","+String((int)GPS.satellites)+","+ SOFTWARE_STATE       +","+CMD_ECHO;
PACKET_COUNT ++;
Serial.println(sentencia);
if (simulation==true ){
                        auxsimcounter++;
                        if (auxsimcounter>138){
                                                auxsimcounter=auxsimcounter;
                                              }
                      }


///////////////////////XBEE  TX
tx(sentencia,groundaddress);
///////////////////////////
if (SOFTWARE_STATE=="LANDED"){
  telemetry=false;

tx("CMD,1064,SP1X,OFF",payload1address);
  

}

}

groundrx="";


////////////////////XBE RX

if (Serial2.available() > 0) {
  
  byte rx;
  rx=Serial2.read();
  //Serial.println(rx,HEX);
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
                          Serial.println("P1RX");
                            payload1rx=rxdata;
                            Serial.println(rxdata);
                            SP1_PACKET_COUNT++;
                              String toground,SP_ALTITUDE,SP_TEMP,SP_VOLTAGE,SP_GYRO_R,SP_GYRO_P,SP_GYRO_Y,SP_ACCEL_R,SP_ACCEL_P,SP_ACCEL_Y,SP_MAG_R,SP_MAG_P,SP_MAG_Y,SP_POINTINGERROR,TP_SOFTWARE_STATE;
                              int coma,coma2,coma3,coma4,coma5,coma6,coma7,coma8,coma9,coma10,coma11,coma12,coma13;
                              
                              //sentencia=String(counter)+","+String(altprom)+","+TEMP+","+String(voltage)+","+String(gx)+","+String(gy)+","+String(gz)+","+String(ax)+","+String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              
                              coma= rxdata.indexOf(',');
                              //Serial.println("valor"+rxdata.substring(0,coma));
                              rxdata=rxdata.substring(coma+1,rxdata.length());


                              //sentencia=String(altprom)+","+TEMP+","+String(voltage)+","+String(gx)+","+String(gy)+","+String(gz)+","+String(ax)+","+String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_ALTITUDE=rxdata.substring(0,coma);
                              altpayload=SP_ALTITUDE.toDouble();
                              rxdata=rxdata.substring(coma+1,rxdata.length());

                              //sentencia=TEMP+","+String(voltage)+","+String(gx)+","+String(gy)+","+String(gz)+","+String(ax)+","+String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_TEMP=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              //sentencia=String(voltage)+","+String(gx)+","+String(gy)+","+String(gz)+","+String(ax)+","+String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_VOLTAGE=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              //sentencia=String(gx)+","+String(gy)+","+String(gz)+","+String(ax)+","+String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_GYRO_R=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              //sentencia=String(gy)+","+String(gz)+","+String(ax)+","+String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_GYRO_P=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              //sentencia=String(gz)+","+String(ax)+","+String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_GYRO_Y=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              //sentencia=String(ax)+","+String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_ACCEL_R=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              
                              //sentencia=String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_ACCEL_P=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              
                              
                              //sentencia=String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_ACCEL_Y=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              
                              //sentencia=String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_MAG_R=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              
                              //sentencia=String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_MAG_P=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              
                              //sentencia=String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_MAG_Y=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              //sentencia=String(heading_rad-180.00);
                              coma= rxdata.indexOf(',');
                              SP_POINTINGERROR=rxdata.substring(0,coma);
                              rxdata=rxdata.substring(coma+1,rxdata.length());
                              
                              RtcDateTime now = Rtc.GetDateTime();
                              realtime=printDateTime(now);
                              String PACKET_TYPE;
//                              if (SP1_RELEASED=="R"){
//                                PACKET_TYPE="SP1";
//                                                        } TEAM_ID, MISSION_TIME, PACKET_COUNT, PACKET_TYPE, TP_ALTITUDE,TP_TEMP,                                                                                     TP_VOLTAGE, GYRO_R, GYRO_P, GYRO_Y, ACCEL_P, ACCEL_P,ACCEL_Y, MAG_R, MAG_P, MAG_Y, POINTING_ERROR, TP_SOFTWARE_STATE
                              //             1231,      23:54:35,              5 ,              ,               -1,23.7,0.54
                              //sentencia=String(counter)+","+String(altprom)+","+TEMP+","+String(voltage)+","+String(gx)+","+String(gy)+","+String(gz)+","+String(ax)+","+String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
                                      //175,0.00,19.9,4.17,0.00,0.00,-0.00,0.01,0.59,-0.81,0.32,0.91,-0.28,-181.07

                            //          1064,                                                          09:03:28,                              9     ,T      ,                       0.00,    .00               ,        00            ,                   0    ,                       ,                   19.9 ,                  9.9,.9,9,,3.67,.67,67,
                            
                              toground="1064," + realtime.substring(realtime.length()-8,realtime.length()) + "," + String(SP1_PACKET_COUNT) + "," + "T" + "," + String(SP_ALTITUDE) +  "," + String(SP_TEMP) +","+String(SP_VOLTAGE)+ "," + String(SP_GYRO_R)+"," + String(SP_GYRO_P)+"," + String(SP_GYRO_Y)+ ","+String(SP_ACCEL_R)+"," + String(SP_ACCEL_P)+"," + String(SP_ACCEL_Y)+ ","+String(SP_MAG_R)+"," + String(SP_MAG_P)+"," + String(SP_MAG_Y)+","+String(SP_POINTINGERROR)+","+ "RELEASED";
                              
                              tx(toground,groundaddress);
                              if (p1released && (Timep4+250)>millis() ) {
  
              Timep4=millis();
              Serial.println(String(payloadspeed));
              altprom=(int(bmp.readAltitude(calibratealtitude))+altprom)/2;
              if ( (altprom-altpayload)> (((millis()-Timep1)/1000)*0,5) ){//10m in 20sec 0,5 m/s}  muy rapido
                if (payloadspeed-1<0) {
                  payloadspeed--;
                }
                Servo2.write(payloadspeed);
              }
              if ( (altprom-altpayload)< (((millis()-Timep1)/1000)*0,5) ){  //10m in 20sec 0,5 m/s} muy lento
                if ( payloadspeed+1<=180 ) {
                  payloadspeed++;
                }
                Servo2.write(payloadspeed);
              }
              
}
  
                              
                              
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
  return(44330.00*(1-pow(P/P0,1/5.255)));

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
     
  

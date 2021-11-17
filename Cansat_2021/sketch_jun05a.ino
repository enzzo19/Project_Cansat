#include <MPU9250.h>
#include <Adafruit_BMP280.h>
MPU9250 IMU(Wire,0x68);
Adafruit_BMP280 bmp;

int status;
float calibratealtitude;
///////////XBEE
byte containeraddress[2]={0x01,0x11};
//byte payload1address[2]={0x00, 0x11};
//byte payload2address[2]={0x00, 0x12};
//byte groundaddress[2]={0x00, 0x13};
byte rxbuff[100];
byte rxaux=0;
byte rxchcksum;
String containerrx ;
bool ban;
unsigned long Time;
//////////


void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
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

status = IMU.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
//CMD,<TEAM_ID>,SP1X,<ON_OFF>
//containerrx="CMD,1231,SP1X,ON";


if (containerrx=="CMD,1231,SP1X,ON"){
                                    ban=true;
                                    }
if (containerrx=="CMD,1231,SP1X,OFF"){
                                    ban=false;
                                    }

if ((ban==true)&& ((Time+1000)<=millis())){
                                    //<TEAM_ID>,<SP_ALTITUDE>,<SP_TEMP>,<SP_ROTATION_RATE>
                                    String TEMP;
                                    Time = millis();
                                    //altitude=String(int(bmp.readAltitude(calibratealtitude))); 
                                    TEMP=String(bmp.readTemperature());
                                    TEMP.remove(TEMP.length() - 1,TEMP.length());
                                    tx("1231,"+String(int(bmp.readAltitude(calibratealtitude)))+","+TEMP+","+"xxx",containeraddress);
                                    //Serial.println(sentencia);
                                    }





if (Serial.available() > 0) {
  
  byte rx;
  rx=Serial.read();
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
                       
                                                                             
                                                                              if (  ((rxbuff[rxaux]-1)!=rxchcksum) && (rxaux>=5) &&(rxbuff[5]==0x11)){
                                                                              containerrx="";
                                                                              for (byte i=8; i<(rxaux-1); i++){
                                                                              containerrx=containerrx+(char)rxbuff[i];
                                                                                                                } 
                                                                                                                                    } 
                                                              
                        
                          


                           
                           }
                           

   
//                            

   
}
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
    Serial.write(bufferout[i] & 0xFF);
  }
  Serial.write(chk);
}
     

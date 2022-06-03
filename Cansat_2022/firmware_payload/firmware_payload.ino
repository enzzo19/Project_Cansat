#include <Adafruit_GPS.h>
#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <EEPROM.h>


#define EEPROM_Servo1low 0
#define EEPROM_Servo1high 1
MPU9250 IMU(Wire,0x68);
Adafruit_BMP280 bmp;
Servo Servo1;
Servo Servo2;
Servo Servo3;

//int value = 0;
float voltage;
float R1 = 47000.0;
float R2 = 33000.0;
byte auxsimcounter,payloadspeed;
unsigned long Time,Timep1,Timep2,Timep3,Timep4,Timep5;
int status;
String sentencia,altitude,Pressure,TEMP,SOFTWARE_STATE,CMD_ECHO,groundrx,payload1rx,containerrx,groundtx,p1rx,p1tx,p2rx,p2tx,realtime,gpsaltitude;
float calibratealtitude;
//double voltage;
int PACKET_COUNT,SP1_PACKET_COUNT;
char MODE,aux;
String SP1_RELEASED;
double altprom,altpayload;
int alt,counter,servopos,servopos2,maxaltitude;
double altbuffer[4];
bool telemetry,gimballon,cameraon,notpass;
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
uint8_t eeprom_buffer[24];
float value;
//////////


//////////imu

/* accelerometer and magnetometer data */
float a, ax, ay, az, h, hx, hy, hz,gx,gy,gz;
/* magnetometer calibration data */
float hxb, hxs, hyb, hys, hzb, hzs;
/* euler angles */
float pitch_rad, roll_rad, yaw_rad, heading_rad;
/* filtered heading */
float filtered_heading_rad;
float window_size = 20;
/* conversion radians to degrees */
const float R2D = 180.0f / PI;
/* MPU 9250 object */
/* MPU 9250 data ready pin */
const uint8_t kMpu9250DrdyPin = 2;
/* Flag set to indicate MPU 9250 data is ready */
volatile bool imu_data_ready = false;

/* ISR to set data ready flag */
void data_ready()
{
  imu_data_ready = true;
}


//////

void setup() {
  pinMode(3, OUTPUT); 
  gimballon=false;
  Serial.begin(9600);
  rxpacket=false;
  Serial2.begin(19200);
  Serial3.begin(19200);
  Servo1.attach(9);
  Servo2.attach(8);
  Servo1low=EEPROM.read(EEPROM_Servo1low);
  Servo1high=EEPROM.read(EEPROM_Servo1high);
  Servo1.write(Servo1high);
  payloadspeed=90;
  Servo2.write(payloadspeed);
  counter=0;
  notpass=true;
SOFTWARE_STATE="BOOT";



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
 IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
  IMU.setSrd(19);
  IMU.enableDataReadyInterrupt();
  uint8_t eeprom_buffer[24];
  for (unsigned int i = 0; i < sizeof(eeprom_buffer); i++ ) {
    eeprom_buffer[i] = EEPROM.read(i);
  }
  memcpy(&hxb, eeprom_buffer, sizeof(hxb));
  memcpy(&hyb, eeprom_buffer + 4, sizeof(hyb));
  memcpy(&hzb, eeprom_buffer + 8, sizeof(hzb));
  memcpy(&hxs, eeprom_buffer + 12, sizeof(hxs));
  memcpy(&hys, eeprom_buffer + 16, sizeof(hys));
  memcpy(&hzs, eeprom_buffer + 20, sizeof(hzs));
  IMU.setMagCalX(hxb, hxs);
  IMU.setMagCalY(hyb, hys);
  IMU.setMagCalZ(hzb, hzs);
  /* Attach the data ready interrupt to the data ready ISR */
  pinMode(kMpu9250DrdyPin, INPUT);
  attachInterrupt(kMpu9250DrdyPin, data_ready, RISING);
//


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

Timep2=millis();
}



void loop() {



if (Timep2+50<millis()){
altprom=(int(bmp.readAltitude(calibratealtitude))+altprom)/2; 
  
}
  
////SERIAL PORT
if (Serial.available() > 0) {
  
groundrx=Serial.readString();
 groundrx.trim();
 while(Serial.available() > 0){
  aux=Serial.read();
        }
 }
////

/////////XBEE RX

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
                       
                                                                             
                                                                              if ( (rxbuff[4]==0x00) &&(rxbuff[5]==0x10)  ){
                                                                                Serial.println("XX");
                                                                              containerrx="";
                                                                              for (byte i=8; i<(rxaux-1); i++){
                                                                              containerrx=containerrx+(char)rxbuff[i];
                                                                                                                } 
                                                                                                                Serial.println(containerrx);
                                                                                                                                    } 
                                                              
                        
                          


                           
                           }
                           

   
//                            

   
}




///////IMU



if (imu_data_ready) {
    imu_data_ready = false;
    /* Read the MPU 9250 data */
    IMU.readSensor();
    ax = IMU.getAccelX_mss();
    ay = IMU.getAccelY_mss();
    az = IMU.getAccelZ_mss();
    hx = IMU.getMagX_uT();
    hy = IMU.getMagY_uT();
    hz = IMU.getMagZ_uT();
    gx = IMU.getGyroX_rads();
    gy = IMU.getGyroY_rads();
    gz = IMU.getGyroZ_rads();
    /* Normalize accelerometer and magnetometer data */
    a = sqrtf(ax * ax + ay * ay + az * az);
    ax /= a;
    ay /= a;
    az /= a;
    h = sqrtf(hx * hx + hy * hy + hz * hz);
    hx /= h;
    hy /= h;
    hz /= h;
    /* Compute euler angles */
    pitch_rad = asinf(ax);
    roll_rad = asinf(-ay / cosf(pitch_rad));
    yaw_rad = atan2f(hz * sinf(roll_rad) - hy * cosf(roll_rad), hx * cosf(pitch_rad) + hy * sinf(pitch_rad) * sinf(roll_rad) + hz * sinf(pitch_rad) * cosf(roll_rad));
//heading_rad = constrainAngle360(yaw_rad);
heading_rad = constrainAngle180(yaw_rad);
    /* Filtering heading */
    filtered_heading_rad = (filtered_heading_rad * (window_size - 1.0f) + heading_rad) / window_size;
    /* Display the results */
    //Serial.print(pitch_rad * R2D);
    //Serial.print("\t");
    //Serial.print(roll_rad * R2D);
    //Serial.print("\t");
    //Serial.print(yaw_rad * R2D);
    //Serial.print("\t");
    //Serial.print(heading_rad * R2D);
    //Serial.print("\t");
    //Serial.println(filtered_heading_rad * R2D);
  }

/////////////////////
//CMD,<TEAM_ID>,SP1X,<ON_OFF>
if ((containerrx=="CMD,1064,SP1X,ON")){
 telemetry=true;
 Timep1=millis();
 containerrx="";
 Serial.println("Tel true");
}
if ((containerrx=="CMD,1064,SP1X,OFF")){
 telemetry=false;
}


//CMD,<TEAM_ID>,GIMBALL,<ON_OFF>
if ((containerrx=="CMD,1064,GIMBALL,ON")){
 gimballon=true;
}
if ((containerrx=="CMD,1064,GIMBALL,OFF")){
 gimballon=false;
} 

//CMD,<TEAM_ID>,GIMBALL,<ON_OFF>
if ((containerrx=="CMD,1064,GIMBALL,ON")){
 gimballon=true;
}
if ((containerrx=="CMD,1064,GIMBALL,OFF")){
 gimballon=false;
} 

if (gimballon){
  //(filtered_heading_rad * R2D)
}




//CMD,<TEAM_ID>,CAMERA,<ON_OFF>
if ((containerrx=="CMD,1064,CAMERA,ON")){
 cameraon=true;
 
}
if ((containerrx=="CMD,1064,CAMERA,OFF")){
 cameraon=false;
} 


if (cameraon && notpass){
   digitalWrite(3, HIGH);
   notpass=false;
   Timep3=millis();
}
if (!notpass && Timep3+120<=millis()){
   digitalWrite(3, LOW);
   notpass=true;
}


if ((containerrx.substring(0,12)=="CMD,1064,GC,")){
 String gimballcommand;
 gimballcommand=containerrx.substring(12,containerrx.length());
}


if ( telemetry && (Timep1+250<=millis())  ) {
//if ( telemetry ) {
  Serial.println("YY");
  Timep1=millis();
//TEAM_ID, MISSION_TIME, PACKET_COUNT, PACKET_TYPE, TP_ALTITUDE,TP_TEMP,TP_VOLTAGE, GYRO_R, GYRO_P, GYRO_Y, ACCEL_P, ACCEL_P,ACCEL_Y, MAG_R, MAG_P, MAG_Y, POINTING_ERROR, TP_SOFTWARE_STATE
  counter++;
  
  TEMP=String(bmp.readTemperature());
  TEMP.remove(TEMP.length() - 1,TEMP.length());
  value = analogRead(A0);
  voltage = value * (5.0/1024)*((R1 + R2)/R2);
   //gx,gy,gz,ax,ay,az,hx,hy,hz
  sentencia=String(counter)+","+String(altprom)+","+TEMP+","+String(voltage)+","+String(gx)+","+String(gy)+","+String(gz)+","+String(ax)+","+String(ay)+","+String(az)+","+String(hx)+","+String(hy)+","+String(hz)+","+String(heading_rad-180.00);
  tx(sentencia,containeraddress);
}







}





#define countof(a) (sizeof(a) / sizeof(a[0]))


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



float constrainAngle360(float dta) {
  dta = fmod(dta, 2.0 * PI);
  if (dta < 0.0)
    dta += 2.0 * PI;
  return dta;
}
float constrainAngle180(float dta) {
  if(dta >  PI) dta -= (PI*2.0f);
  if(dta < -PI) dta += (PI*2.0f);
  return dta;
}

#include <U8glib.h>

// =======RANGKAIAN UTAMA==========
// DRIVER MOTOR(6)
//    pwm1 =9  (karena bisa PWM)
//    dir1 =8
//    pwm2 =11  (karena bisa PWM)
//    dir2 =10
// Sensor JARAK(4)
//    trigOut =4 
//    echoIn  =5   (karena sinyal digital)
// Sensor JOYSTIK(5)
//    vert =A1    (bisa analog input)
//    hor  =A0    (bisa analog input)
// BUZZER
//    Buzzer =7
// MOTOR BRAKE
//    Brake Mosfet=6
// =======Rangkaian Fitur=========
// OLED
//    I2C1 =A4   (bisa komunikasi TWI)
//    I2C2 =A5   (bisa komunikasi TWI)
// VOLT METER
//    volt =A2    (bisa analog input)
// ENCODER
//    Encoder1 =2   (bisa External Interrupt) 
//    Encoder2 =3   (bisa External Interrupt) 
// Klakson
//    (NONE)        (Kehubung langsung sama button, gamasuk ke ardu)

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========UTAMA===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| 
//Motor 1
#define lalahuta 6
#define jancuk 9
//kok bisa, yaiyadong
//lalalalalaaaaa ulala

int pwm1 = 9; //PWM Motor 1
int dir1 = 8; //dir Motor 1

// Motor 2
 
int pwm2 = 11; //PWM Motor 2
int dir2 = 10; //dir Motor 2

// Joystick Input
 
int joyVert = A1; // Vertical Y
int joyHorz = A0; // Horizontal X
// int button = 12; //button joystik (belom kepake)

// Buzzer Mundur

int buzzer = 7;

// Brake

int brake = 6;

// Sensor jarak

int trigOut = 4;
int echoIn = 5;
 
// kecepatan motor
int MotorSpeed1 = 0;
int MotorSpeed2 = 0;

int MAX_FOR_SPEED=200;
int MAX_BACK_SPEED=150;

// Joystick Values 
 
int joyposVert = 512;
int joyposHorz = 512;  
 
// buzzer mundur

//
//
//

int buzzerState = HIGH;
unsigned long buzzerStarted = 0;
const long interval = 1000000;

//
//
//

//==================================HC-SR04========================================================

unsigned long currentMicros = 0;
int distance=40; // the distance in cms         
unsigned long previousMicros = 0; // will sotre last time TRIGGER was updated    
long OnTime = 10; //microseconds of on-time     
long OffTime = 2; //microseconds of off-time //will store last time viewTime was updated   
int triggerState = LOW; // triggerState used to set it up   
long duration;

int a,b=40;
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========UTAMA===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========FITUR===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||



U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0); // SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9, Res = 12

                                                // For OLED display with SH1106 driver. If you use another display,

                                                // then please check the u8glib documentation and website at

                                                // https://github.com/olikraus/u8glib


int volt, battery=0;

int xmax=128;                                   // max length x-axis

int ymax=64;                                    // max length y-axis

int xcenter=xmax/2;                             // center of x-axis

int ycenter=ymax/2+27;                          // center of y-axis

int arc=ymax/2;                             

int angle=0;                      
    // some custom gauge labels

                  // predefined x-position of a gauge label

#define potmeterPin A1                                 // simulate analogue value with potentiometer

int m=10;
float REAL_SPEED1=10;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++

u8g_uint_t xx=0;



// ------------------------------------------------- void gauge() ------------------------------------------

void gauge(uint8_t angle) {



  // draw border of the gauge

  u8g.drawCircle(xcenter,ycenter,arc+6, U8G_DRAW_UPPER_RIGHT);

  u8g.drawCircle(xcenter,ycenter,arc+4, U8G_DRAW_UPPER_RIGHT);

  u8g.drawCircle(xcenter,ycenter,arc+6, U8G_DRAW_UPPER_LEFT);

  u8g.drawCircle(xcenter,ycenter,arc+4, U8G_DRAW_UPPER_LEFT);



  // draw the needle

  float x1=sin(2*angle*2*3.14/360);              // needle position

  float y1=cos(2*angle*2*3.14/360); 

  u8g.drawLine(xcenter, ycenter, xcenter+arc*x1, ycenter-arc*y1);

  u8g.drawDisc(xcenter, ycenter, 5, U8G_DRAW_UPPER_LEFT);

  u8g.drawDisc(xcenter, ycenter, 5, U8G_DRAW_UPPER_RIGHT);
  

  u8g.setFont(u8g_font_chikita);
 
  // show scale labels

  u8g.drawStr( 20, 59, "0");                   

  u8g.drawStr( 25, 34, "1.5");

  u8g.drawStr( 60, 31, "3");

  u8g.drawStr( 94, 34, "4.5");

  u8g.drawStr( 105, 59, "6");
  
  u8g.drawStr(57,16,"km/h");

  // show gauge label

  u8g.setPrintPos(51,49);           

  u8g.print("SPooD");

 

  // show digital value and align its position

  u8g.setFont(u8g_font_profont22);             

  u8g.setPrintPos(19,16);//+12

  u8g.print((float)REAL_SPEED1/10,1);
  //u8g.print(distance);
//------------------------------------------------baterai-----------------------------------------------------------

  u8g.setFont(u8g_font_unifont);
  
  u8g.drawFrame(100,3,28,12);
  u8g.drawBox(98,7,2,4);
  
  if (battery >10){
    u8g.drawBox(122,5,4,8);
    if (battery >25){
      u8g.drawBox(117,5,4,8);
      if (battery >40){
        u8g.drawBox(112,5,4,8);
        if (battery >60){
          u8g.drawBox(107,5,4,8);
          if (battery >80){
            u8g.drawBox(102,5,4,8);
          }
        }
      }
    }
  }
  else{}
//------------------------------------------------------------------------------------------------------------------
}
//——-------------------------------------------------code for RPM—————-——---------------------------------------
const byte PulsesPerRevolution = 1; 
const unsigned long ZeroTimeout = 3000000;
const byte numReadings = 4;


volatile unsigned long LastTimeWeMeasured1;  // Stores the last time we measured a pulse so we can calculate the period.
volatile unsigned long PeriodBetweenPulses1 = ZeroTimeout+1000;  // Stores the period between pulses in microseconds.
                       // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
volatile unsigned long PeriodAverage1 = ZeroTimeout+1000;  // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.
                       // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
unsigned long FrequencyRaw1;  // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.
unsigned long FrequencyReal1;  // Frequency without decimals.
unsigned long RPM1;  // Raw RPM without any processing.
unsigned int PulseCounter1 = 1;  // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.

unsigned long PeriodSum1; // Stores the summation of all the periods to do the average.

unsigned long LastTimeCycleMeasure1 = LastTimeWeMeasured1;  // Stores the last time we measure a pulse in that cycle.
                                    // We need a variable with a value that is not going to be affected by the interrupt
                                    // because we are going to do math and functions that are going to mess up if the values
                                    // changes in the middle of the cycle.
unsigned int AmountOfReadings1 = 1;

unsigned int ZeroDebouncingExtra1; 

unsigned long readings1[numReadings];  // The input.
unsigned long readIndex1;  // The index of the current reading.
unsigned long total1;  // The running total.
unsigned long average1;  // The RPM value after applying the smoothing.

  //========================================2222222222222222222====================================================



volatile unsigned long LastTimeWeMeasured2;  // Stores the last time we measured a pulse so we can calculate the period.
volatile unsigned long PeriodBetweenPulses2 = ZeroTimeout+1000;  // Stores the period between pulses in microseconds.
                       // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
volatile unsigned long PeriodAverage2 = ZeroTimeout+1000;  // Stores the period between pulses in microseconds in total, if we are taking multiple pulses.
                       // It has a big number so it doesn't start with 0 which would be interpreted as a high frequency.
unsigned long FrequencyRaw2;  // Calculated frequency, based on the period. This has a lot of extra decimals without the decimal point.
unsigned long FrequencyReal2;  // Frequency without decimals.
unsigned long RPM2;  // Raw RPM without any processing.
unsigned int PulseCounter2 = 1;  // Counts the amount of pulse readings we took so we can average multiple pulses before calculating the period.

unsigned long PeriodSum2; // Stores the summation of all the periods to do the average.

unsigned long LastTimeCycleMeasure2 = LastTimeWeMeasured2;  // Stores the last time we measure a pulse in that cycle.
                                    // We need a variable with a value that is not going to be affected by the interrupt
                                    // because we are going to do math and functions that are going to mess up if the values
                                    // changes in the middle of the cycle. 
unsigned int AmountOfReadings2 = 1;

unsigned int ZeroDebouncingExtra2; 

unsigned long readings2[numReadings];  // The input.
unsigned long readIndex2;  // The index of the current reading.
unsigned long total2;  // The running total.
unsigned long average2;  // The RPM value after applying the smoothing.

//——--------------------------------------------------------------—————-——---------------------------------------



//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========FITUR===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||


void setup()
 
{
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========UTAMA===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
 
  // pin yang kehubung ke Driver motor jadi output
 
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

  // buat kondisi pertama motor maju
  
  // Motor A
  
  digitalWrite(pwm1, LOW);
  digitalWrite(dir1, HIGH);
  
  // Motor B
  
  digitalWrite(pwm2, LOW);
  digitalWrite(dir2, HIGH);

  // pin Trig sensor jarak & buzzer jadi output

  pinMode(buzzer, OUTPUT);
  pinMode(trigOut, OUTPUT);

  // Sensor jarak

  pinMode(trigOut, OUTPUT); // 
  pinMode(echoIn, INPUT); //

  pinMode(brake, OUTPUT);
  
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========UTAMA===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========FITUR===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||


  u8g.setFont(u8g_font_chikita);

  u8g.setColorIndex(1);                         // Instructs the display to draw with a pixel on.



  // assign default color value

  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) {

    u8g.setColorIndex(255);                     // white

  }

  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT ) {

    u8g.setColorIndex(3);                       // max intensity

  }

  else if ( u8g.getMode() == U8G_MODE_BW ) {

    u8g.setColorIndex(1);                       // pixel on

  }

  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {

    u8g.setHiColorByRGB(255,255,255);

  }

//———-------------------------------------------–code for RPM—————-------------------------------------
  attachInterrupt(digitalPinToInterrupt(2), Pulse_Event1, RISING);  // Enable interruption pin 2 when going from LOW to HIGH.
  //========================================2222222222222222222====================================================
  attachInterrupt(digitalPinToInterrupt(3), Pulse_Event2, RISING);
  delay(1000);
//———-------------------------------------------–-----------—————-------------------------------------


//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========FITUR===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

}
 
void loop() {
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========UTAMA===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
  // ngebaca joystik
 
  joyposVert = analogRead(joyVert); 
  joyposHorz = analogRead(joyHorz);
  joyposVert = map(joyposVert, 0, 1023, 1023, 0);
  currentMicros = micros();

   
  // maks kanan 1023, maks atas 1023
  // maks kiri 0, maks bawah 0
  // dari sini kita bisa nentuin arah yang mau dituju ama pengguna
 
  if (joyposVert < 459)
  {
    // mundur
  
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  //==============================NGUKUR JARAK============================================

  if((triggerState == LOW) && (currentMicros - previousMicros >= OffTime))     
   {      
     triggerState = HIGH; // turn it on      
     previousMicros = currentMicros; // remember the time     
     digitalWrite(trigOut, triggerState); // update the actual trigger     
   }    
   else if((triggerState == HIGH) && (currentMicros - previousMicros >= OnTime))    
   {           
     triggerState = LOW; // turn it off          
     previousMicros = currentMicros; // remember the time           
     digitalWrite(trigOut, triggerState); // update the actual trigger      
   }    

    duration = pulseIn(echoIn,HIGH);        
    distance = ((duration*0.034)/2);      

    a=distance;
    distance=max(a,b);
    b=a;
    
  //==============================================================================================

    if (distance < 30 or distance >60) // kalau jarak kurang dari 30cm atau lebih dari 60, kursi roda mati
    {
      // berentiin kursi roda & Buzzer bunyi nonstop
  //    MotorSpeed1 = 0;
    //  MotorSpeed2 = 0; 

      digitalWrite(buzzer,HIGH);
    }

  //==============================================================================================

    
    else
    {
      // Motor mundur & buzzer beep beep
   
   
      // nentuin kecepatan motor
       // buat angkanya jadi positif
   
      //joyposVert = joyposVert - 459; // biar ga kebalik kecepatannya
      //joyposVert = joyposVert * -1;  // buat jadi positif
   
      MotorSpeed1 = map(joyposVert, 459, 0, 0, MAX_BACK_SPEED); // 200 biar mundurnya ga kekencengan
      MotorSpeed2 = map(joyposVert, 459, 0, 0, MAX_BACK_SPEED);


      //
      //
      //
      if (currentMicros - buzzerStarted-500000>= interval) {
        buzzerStarted = currentMicros;
        if (buzzerState == HIGH) {
          buzzerState = LOW;
        } else {
          buzzerState = HIGH;
        }
      digitalWrite(buzzer, buzzerState);

        //
        //
        //
       }
    }
 
  }
  else if (joyposVert > 564)
  {
    // maju
  
    // motor 1 maju
 
    digitalWrite(dir1, HIGH); 
    digitalWrite(dir2, HIGH);
 
    // nentuin kecepatan
 
    MotorSpeed1 = map(joyposVert, 564, 1023, 0, MAX_FOR_SPEED);
    MotorSpeed2 = map(joyposVert, 564, 1023, 0, MAX_FOR_SPEED); 

    digitalWrite(buzzer,LOW); // matiin buzzer
 
  }
  else
  {
    // This is Stopped
 
    MotorSpeed1 = 0;
    MotorSpeed2 = 0; 

    digitalWrite(buzzer,LOW);
  }
  
  // Now do the steering
  // The Horizontal position will "weigh" the motor speed
  // Values for each motor
 
  if (joyposHorz < 459)
  {
    // kiri
 
    //joyposHorz = joyposHorz - 459; // sama kaya mundur
    //joyposHorz = joyposHorz * -1;  

 
    joyposHorz = map(joyposHorz, 459, 0, 0, 255);
        
 
    MotorSpeed2 = MotorSpeed2 - joyposHorz/3;
    MotorSpeed1 = MotorSpeed1 + joyposHorz;
 
    // biar motor speed 2 galebih dari 255 (walaupun gabisa tapi biar bagus aja :D)
 
    if (MotorSpeed2 < 0)MotorSpeed2 = 0;
    if (MotorSpeed1 > MAX_FOR_SPEED)MotorSpeed1 = MAX_FOR_SPEED;
    if (MotorSpeed1 > MAX_BACK_SPEED and joyposVert < 511)MotorSpeed1 = MAX_BACK_SPEED;
 
  }
  else if (joyposHorz > 564)
  {
    // kanan
 
    joyposHorz = map(joyposHorz, 564, 1023, 0, 255);
        
 
    MotorSpeed2 = MotorSpeed2 + joyposHorz;
    MotorSpeed1 = MotorSpeed1 - joyposHorz/3;
 
    // sama kaya kiri
 
    if (MotorSpeed2 > MAX_FOR_SPEED)MotorSpeed2 = MAX_FOR_SPEED;
    if (MotorSpeed2 > MAX_BACK_SPEED and joyposVert < 511)MotorSpeed2 = MAX_BACK_SPEED;
    if (MotorSpeed1 < 0)MotorSpeed1 = 0;      
 
  }
 //      =========================YANG MUNDUR BELOM DIPATOK=========================
 
  // kalo motor speed <= 8, motornya gagerak cuma ngeden
 
  if (MotorSpeed1 < 8)MotorSpeed1 = 0;
  if (MotorSpeed2 < 8)MotorSpeed2 = 0;

  if (MotorSpeed1 < 8 and MotorSpeed2 < 8)digitalWrite(brake,HIGH);
  else digitalWrite(brake,LOW);
  // keluarin sinyal ke driver
 
  analogWrite(pwm1, MotorSpeed1);
  analogWrite(pwm2, MotorSpeed2);

 //selesaiiiii
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========UTAMA===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========FITUR===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||


//-------------------------------------------------RPM--------------------------------------------------------
  LastTimeCycleMeasure1 = LastTimeWeMeasured1;  // Store the LastTimeWeMeasured in a variable.

  if(currentMicros < LastTimeCycleMeasure1)
  {
    LastTimeCycleMeasure1 = currentMicros;
  }
  // Calculate the frequency:
  FrequencyRaw1 = 10000000000 / PeriodAverage1;  // Calculate the frequency using the period between pulses.
  
  if(PeriodBetweenPulses1 > ZeroTimeout - ZeroDebouncingExtra1 || currentMicros - LastTimeCycleMeasure1 > ZeroTimeout - ZeroDebouncingExtra1)
  {  // If the pulses are too far apart that we reached the timeout for zero:
    FrequencyRaw1 = 0;  // Set frequency as 0.
    ZeroDebouncingExtra1 = 2000;  // Change the threshold a little so it doesn't bounce.
  }
  else
  {
    ZeroDebouncingExtra1 = 0;  // Reset the threshold to the normal value so it doesn't bounce.
  }
  
  FrequencyReal1 = FrequencyRaw1 / 10000; 
  RPM1 = FrequencyRaw1 / PulsesPerRevolution * 60;  // Frequency divided by amount of pulses per revolution multiply by
                                                  // 60 seconds to get minutes.
  RPM1 = RPM1 / 10000;  // Remove the decimals.

  // Smoothing RPM:
  total1 = total1 - readings1[readIndex1];  // Advance to the next position in the array.
  readings1[readIndex1] = RPM1;  // Takes the value that we are going to smooth.
  total1 = total1 + readings1[readIndex1];  // Add the reading to the total.
  readIndex1 = readIndex1 + 1;  // Advance to the next position in the array.

  if (readIndex1 >= numReadings)  // If we're at the end of the array:
  {
    readIndex1 = 0;  // Reset array index.
  }
  
  average1 = total1 / numReadings; 

//========================================2222222222222222222====================================================
  LastTimeCycleMeasure2 = LastTimeWeMeasured2;  // Store the LastTimeWeMeasured in a variable.

  if(currentMicros < LastTimeCycleMeasure2)
  {
    LastTimeCycleMeasure2 = currentMicros;
  }
  // Calculate the frequency:
  FrequencyRaw2 = 10000000000 / PeriodAverage2;  // Calculate the frequency using the period between pulses.
  
  if(PeriodBetweenPulses2 > ZeroTimeout - ZeroDebouncingExtra2 || currentMicros - LastTimeCycleMeasure2 > ZeroTimeout - ZeroDebouncingExtra2)
  {  // If the pulses are too far apart that we reached the timeout for zero:
    FrequencyRaw2 = 0;  // Set frequency as 0.
    ZeroDebouncingExtra2 = 2000;  // Change the threshold a little so it doesn't bounce.
  }
  else
  {
    ZeroDebouncingExtra2 = 0;  // Reset the threshold to the normal value so it doesn't bounce.
  }
  
  FrequencyReal2 = FrequencyRaw2 / 10000; 
  RPM2 = FrequencyRaw2 / PulsesPerRevolution * 60;  // Frequency divided by amount of pulses per revolution multiply by
                                                  // 60 seconds to get minutes.
  RPM2 = RPM2 / 10000;  // Remove the decimals.

  // Smoothing RPM:
  total2 = total2 - readings2[readIndex2];  // Advance to the next position in the array.
  readings2[readIndex2] = RPM2;  // Takes the value that we are going to smooth.
  total2 = total2 + readings2[readIndex2];  // Add the reading to the total.
  readIndex2 = readIndex2 + 1;  // Advance to the next position in the array.

  if (readIndex2 >= numReadings)  // If we're at the end of the array:
  {
    readIndex2 = 0;  // Reset array index.
  }
  
  average2 = total2 / numReadings; 
//REAL_SPEED2=average2*1.13097335; //Frekuensi*(2*pi*r/1000*3600/60/20)*10;

//REAL_SPEED1=(average1+average2)/2*1.13097335;
REAL_SPEED1=(average1+average2)/2*0.05654866776;
//-----------------------------------------------------------------------------------------------------------------------
 


   
volt = analogRead(A2);
battery = map(volt,927,960,0,100);


m = map(REAL_SPEED1,0,60,0,90);

  // show needle and dial

  xx = m;                                      // 135 = zero position, 180 = just before middle, 0 = middle, 45 = max

  if (xx<=45){                                   // positie correctie

    xx=xx+135;

  }

  else {

    xx=xx-45;

  }

 

  // picture loop

  {

    u8g.firstPage(); 

    do {             

      gauge(xx);
    }

    while( u8g.nextPage() );

  }


//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||=========FITUR===========||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

}
void Pulse_Event1()  // The interrupt runs this to calculate the period between pulses:
{

  PeriodBetweenPulses1 = micros() - LastTimeWeMeasured1;  // Current "micros" minus the old "micros" when the last pulse happens.
                                                        // This will result with the period (microseconds) between both pulses.
                                                        // The way is made, the overflow of the "micros" is not going to cause any issue.

  LastTimeWeMeasured1 = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.





  if(PulseCounter1 >= AmountOfReadings1)  // If counter for amount of readings reach the set limit:
  {
    PeriodAverage1 = PeriodSum1 / AmountOfReadings1;  // Calculate the final period dividing the sum of all readings by the
                                                   // amount of readings to get the average.
    PulseCounter1 = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSum1 = PeriodBetweenPulses1;  // Reset PeriodSum to start a new averaging operation.


    // Change the amount of readings depending on the period between pulses.
    // To be very responsive, ideally we should read every pulse. The problem is that at higher speeds the period gets
    // too low decreasing the accuracy. To get more accurate readings at higher speeds we should get multiple pulses and
    // average the period, but if we do that at lower speeds then we would have readings too far apart (laggy or sluggish).
    // To have both advantages at different speeds, we will change the amount of readings depending on the period between pulses.
    // Remap period to the amount of readings:
    int RemapedAmountOfReadings1 = map(PeriodBetweenPulses1, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
    // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
    // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
    // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
    // 4th and 5th values are the amount of readings range.
    RemapedAmountOfReadings1 = constrain(RemapedAmountOfReadings1, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
    AmountOfReadings1 = RemapedAmountOfReadings1;  // Set amount of readings as the remaped value.
  }
  else
  {
    PulseCounter1++;  // Increase the counter for amount of readings by 1.
    PeriodSum1 = PeriodSum1 + PeriodBetweenPulses1;  // Add the periods so later we can average.
  }

}  // End of Pulse_Event.

// ====================================================222222222222222222222=====================================================

void Pulse_Event2()  // The interrupt runs this to calculate the period between pulses:
{

  PeriodBetweenPulses2 = micros() - LastTimeWeMeasured2;  // Current "micros" minus the old "micros" when the last pulse happens.
                                                        // This will result with the period (microseconds) between both pulses.
                                                        // The way is made, the overflow of the "micros" is not going to cause any issue.

  LastTimeWeMeasured2 = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.





  if(PulseCounter2 >= AmountOfReadings2)  // If counter for amount of readings reach the set limit:
  {
    PeriodAverage2 = PeriodSum2 / AmountOfReadings2;  // Calculate the final period dividing the sum of all readings by the
                                                   // amount of readings to get the average.
    PulseCounter2 = 1;  // Reset the counter to start over. The reset value is 1 because its the minimum setting allowed (1 reading).
    PeriodSum2 = PeriodBetweenPulses2;  // Reset PeriodSum to start a new averaging operation.


    // Change the amount of readings depending on the period between pulses.
    // To be very responsive, ideally we should read every pulse. The problem is that at higher speeds the period gets
    // too low decreasing the accuracy. To get more accurate readings at higher speeds we should get multiple pulses and
    // average the period, but if we do that at lower speeds then we would have readings too far apart (laggy or sluggish).
    // To have both advantages at different speeds, we will change the amount of readings depending on the period between pulses.
    // Remap period to the amount of readings:
    int RemapedAmountOfReadings2 = map(PeriodBetweenPulses2, 40000, 5000, 1, 10);  // Remap the period range to the reading range.
    // 1st value is what are we going to remap. In this case is the PeriodBetweenPulses.
    // 2nd value is the period value when we are going to have only 1 reading. The higher it is, the lower RPM has to be to reach 1 reading.
    // 3rd value is the period value when we are going to have 10 readings. The higher it is, the lower RPM has to be to reach 10 readings.
    // 4th and 5th values are the amount of readings range.
    RemapedAmountOfReadings2 = constrain(RemapedAmountOfReadings2, 1, 10);  // Constrain the value so it doesn't go below or above the limits.
    AmountOfReadings2 = RemapedAmountOfReadings2;  // Set amount of readings as the remaped value.
  }
  else
  {
    PulseCounter2++;  // Increase the counter for amount of readings by 1.
    PeriodSum2 = PeriodSum2 + PeriodBetweenPulses2;  // Add the periods so later we can average.
  }

}  // End of Pulse_Event2.

#include <TM1637.h>

/* Example code with timer intyerrutp that will create an interruption each 
 *  500ms using timer1 and prescalar of 256.
Calculations (for 500ms): 
  System clock 16 Mhz and Prescalar 256;
  Timer 1 speed = 16Mhz/256 = 62.5 Khz    
  Pulse time = 1/62.5 Khz =  16us  
  Count up to = 1000ms / 16us = 62500 (so this is the value the OCR register should have)*/ 

  /*Time params*/
uint32_t secs = 0;
uint32_t mins = 0;
uint32_t hours = 0;
uint32_t timeMillis = 0;

  /*Time adding function*/
void addSecs();
void addMins();
void addHours();

  /*Validations*/
bool timeValidation();

  /*Debug functions*/
void printTime();
void printTempTime(int tempHours, int tempMins);

  /*Setup function*/
void setTime();

  /*Display function*/
void displayTime(int hours, int minutes);

  /* Pin Planning */
int button1 = 4;//D4;
int button2 = 3;//D3;
int button3 = 2;//D2;

  /*Interfacing params*/
int CLKPin = 5;
int DIOPin = 6;
TM1637 interface(CLKPin,DIOPin);

void setup() {
  Serial.begin(9600);

  secs = 30;
  mins = 59;
  hours = 23;
  
  cli();                      //stop interrupts for till we make the settings
  /*1. First we reset the control register to amke sure we start with everything disabled.*/
  TCCR1A = 0;                 // Reset entire TCCR1A to 0 
  TCCR1B = 0;                 // Reset entire TCCR1B to 0
 
  /*2. We set the prescalar to the desired value by changing the CS10 CS12 and CS12 bits. */  
  TCCR1B |= B00000100;        //Set CS12 to 1 so we get prescalar 256  
  
  /*3. We enable compare match mode on register A*/
  TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 
  
  /*4. Set the value of register A to 31250*/
//  OCR1A = 62500;             //Finally we set compare register A to this value  
  //temp test
  OCR1A = 62500/4;
  sei();                     //Enable back the interrupts

  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  attachInterrupt(digitalPinToInterrupt(button3), setTime, RISING);

  interface.init();
  interface.set(7);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!timeValidation()) {
//    cli();
//    Serial.flush();
    Serial.println("Time reset needed!");
    delay(100);
    setTime();
//    sei();
  }
  delay(50);
}

//With the settings above, this IRS will trigger each 500ms.
ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  addSecs();
}



void addSecs()
{
  if (timeValidation() == false) {
    return;
  }
  if (secs < 59 && secs >= 0) {
    secs++;
  }
  else if (secs == 59) {
    secs = 0;
    addMins();
  }
//  printTime();
  displayTime(hours, mins);
}

void addMins()
{
  if (mins < 59 && mins >= 0) {
    mins++;
  }
  else if (mins == 59) {
    mins = 0;
    addHours();
  }
}

void addHours()
{
  if (hours < 23 && hours >= 0) {
    hours++;
  }
  else if (hours == 23) {
    hours = 0;
  }
}

bool timeValidation() {
  if (secs > 59 || mins > 59 || hours > 23 ||
      secs < 0 || mins < 0 || hours < 0) {
        return false;
      }
  else {
    return true;
  }
}

void setTime() {
//  cli();
//sei();
  int tempMins = mins;
  int tempHours = hours;
  Serial.println("Set minutes");
  
  while(1) {
    if (digitalRead(button1)) {
      TCNT1  = 0;
      while (TCNT1 < 62500/4){};
      if(tempMins == 59) {
        tempMins = 0;
      }
      else {
        tempMins++;
      }
      }
      if(digitalRead(button2)) {break;}
      TCNT1  = 0;
      while (TCNT1 < 62500){};
      if(digitalRead(button2)) {break;}
      interface.clearDisplay();
      TCNT1  = 0;
      while (TCNT1 < 62500){};
      if(digitalRead(button2)) {break;}
      displayTime(tempHours, tempMins);
    
  }
  TCNT1  = 0;
  while (TCNT1 < 62500/1){};
  Serial.println("Set hours");
  while(1) {
    if (digitalRead(button1)) {
      if(tempHours == 23) {
        tempHours = 0;
      }
      else {
        tempHours++;
      }
      }
      if(digitalRead(button2)) {break;}
      TCNT1  = 0;
      while (TCNT1 < 62500){};
      if(digitalRead(button2)) {break;}
      interface.clearDisplay();
      TCNT1  = 0;
      while (TCNT1 < 62500){};
      if(digitalRead(button2)) {break;}
//      printTempTime(tempHours, tempMins);
        displayTime(tempHours, tempMins);
    
  }
  mins = tempMins;
  hours = tempHours;
  secs = 0;

  TCNT1 = 0;
//  sei();
  return;
}

void printTime() {
  Serial.print(hours);
  Serial.print(":");
  Serial.print(mins);
  Serial.print(":");
  Serial.println(secs);
}

void printTempTime(int tempHours, int tempMins) {
  Serial.print(tempHours);
  Serial.print(":");
  Serial.println(tempMins);
}

void displayTime(int hours, int minutes) {
  interface.display(0, (hours/10)%10);
  interface.display(1, hours%10);
  interface.point(1);
  interface.display(2, (minutes/10)%10);
  interface.display(3, minutes%10);
}

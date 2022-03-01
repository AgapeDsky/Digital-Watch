/**
 * Digital Clock Program
 * by Agape D'sky (13219010), I Gusti Lanang Ari Tri S (13219046)
 * 
 * @brief Digital Clock with set time, stopwatch, and alarm feature
 * @brief Schematic is included in the repository
 * 
 * This program uses Arduino Nano (ATmega328p)
 * Data sheet : https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf
 */

 /*Includes*/
#include <TM1637.h>
#include <Arduino_FreeRTOS.h>
//#define configUSE_TIMERS 2

/*Time params*/
uint32_t secs = 0;
uint32_t mins = 0;
uint32_t hours = 0;
unsigned long timeMillis = 0;
uint32_t alarmHours = 0;
uint32_t alarmMins = 0;
uint32_t alarmStopHours = 0;
uint32_t alarmStopMins = 1;

/*Temporary time params*/
uint32_t tempMainProcessHours1 = 0;
uint32_t tempMainProcessMins1 = 0;
uint32_t tempMainProcessSecs1 = 0;

uint32_t tempMainProcessHours2 = 0;
uint32_t tempMainProcessMins2 = 0;
uint32_t tempMainProcessSecs2 = 0;

uint32_t stopWatchSecs = 0;
uint32_t stopWatchMins = 0;

/*Time adding function*/
void addSecs();
void addMins();
void addHours();
void addTempSecs();
void addTempMins();
void addTempHours();
void mainProcessCallback(void* pvParameters);
void displayCallback(void* pvParameters);
void stopWatchCallback(void* pvParameters);

/*Validations*/
bool timeValidation();

/*Display function*/
void displayTime(int hours, int minutes);
void displayHours(int hours);
void displayMins(int minutes);

/* Input Pin Planning */
int buttonInc = 3; // EXTI/Input 1
int buttonNext = 4; // Input1
int buttonMode = 2; // EXTI/Input 2

/* Interfacing Pin Planning */
int CLKPin = 5;
int DIOPin = 6;
TM1637 interface(CLKPin,DIOPin);
int stopWatchFinish = A5;

/* Display Mode */
uint8_t displayMode = 0;
bool setupMode = 0;
int setTimeMode = 10;
int setAlarmMode = 11;
int stopWatchMode = 12;

/* Alarm */
/*define note*/
#define timeNote 1.5
#define C4 262
#define D4 294
#define E4 330
#define F4 349
#define G4 392
#define A4 440
#define B4 494
#define C5 523
#define D5 587
#define E5 659
#define F5 698
#define G5 784
#define A5 880
#define B5 988
#define C6 1047
#define D6 1175
#define E6 1319
#define F6 1397
#define G6 1568
#define A6 1760
#define B6 1976

/*notes in the melody: */
int melody[] = {
  G5,E5,C5,A5,G5,0 ,E5,G5,F5,G5,F5,D5,E5,0, E6,C6,G5,C6,G5,E5,G5,0,A5,G5,E5,C6,G5,A5,G5,B5,A5,F5,D5,C5,0 ,C6
};
int noteDurations[] {
  4,8,4,4,2,8 ,6,6,4,8,4,4,8,8 ,8,8,8,6,8,8,8,8 ,4,8,4,4,2, 4,8,4,8,4,4,4,2,4
};

int base=9; //pin base
int alarm=8;
bool alarmOn = 0;

void setup() {
  Serial.begin(9600);

  // Initialize time
  secs = 0;
  mins = 10;
  hours = 10;
  
  cli();                      // Disable Interrupt
  
  TCCR1A = 0;                 // Reset Control Registers 
  TCCR1B = 0;                
  TCCR1B |= B00000100;        // Set Presc to 256 -> 1 second uses 62500 ticks  
  TIMSK1 |= B00000010;        // Enable compare match A
  OCR1A = 62500;              // Set compare value 
   
  sei();                      // Enable Interrupt

  /*Initialize Pin Modes*/
  pinMode(buttonInc, INPUT);
  pinMode(buttonNext, INPUT);
  pinMode(buttonMode, INPUT);
  pinMode(alarm, OUTPUT);
  pinMode(stopWatchFinish, OUTPUT);
  pinMode(setTimeMode, OUTPUT);
  pinMode(setAlarmMode, OUTPUT);
  pinMode(stopWatchMode, OUTPUT);
  pinMode(alarm, OUTPUT);
  pinMode(base,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(buttonMode), changeMode, RISING);

  /*Initialize Display*/
  interface.init();
  interface.set(7);

  /*Initialize thread*/
  xTaskCreate(displayCallback, "Task A", 128, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(mainProcessCallback, "Task B", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(stopWatchCallback, "Task C", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(alarmCallback, "Task D", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
}

void loop() {
  
}

void displayCallback(void* pvParameters) {
  (void) pvParameters;
  while(1) {
    switch (displayMode) {
      case 0 : displayTime(hours, mins); break;
      case 1 : displayTime(tempMainProcessHours1, tempMainProcessMins1); break;
      case 2 : displayTime(tempMainProcessHours2, tempMainProcessMins2); break;
      case 3 : displayTime(stopWatchMins, stopWatchSecs); break;
    }
    switch (setupMode) {
      case false : interface.set(7); break;
      case true : interface.set(1); break;
    }
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

void mainProcessCallback(void* pvParameters) {
  (void) pvParameters;
  while(1) {
    if (displayMode == 1) {           // Set time
      tempMainProcessMins1 = mins;
      tempMainProcessHours1 = hours;
      if (digitalRead(buttonNext) && setupMode == false) {
        digitalWrite(setTimeMode, HIGH);
        setupMode = true;
        timeMillis = millis();
        while((millis()-timeMillis)<500){};
        
        attachInterrupt(digitalPinToInterrupt(buttonInc), addTempMins1, RISING);
        while(!digitalRead(buttonNext)) {}
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        timeMillis = millis();
        while((millis()-timeMillis)<500){}

        attachInterrupt(digitalPinToInterrupt(buttonInc), addTempHours1, RISING);
        while(!digitalRead(buttonNext)) {}
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        vTaskDelay(500/portTICK_PERIOD_MS);
        hours = tempMainProcessHours1;
        mins = tempMainProcessMins1;
        setupMode = false;
        digitalWrite(setTimeMode, LOW);
      }
    }

    else if (displayMode == 2) {
      tempMainProcessMins2 = alarmMins;
      tempMainProcessHours2 = alarmHours;
      if (digitalRead(buttonNext) && setupMode == false) {
        digitalWrite(setAlarmMode, HIGH);
        setupMode = true;
        timeMillis = millis();
        while((millis()-timeMillis)<500){};
        
        attachInterrupt(digitalPinToInterrupt(buttonInc), addTempMins2, RISING);
        while(!digitalRead(buttonNext)) {}
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        timeMillis = millis();
        while((millis()-timeMillis)<500){};

        attachInterrupt(digitalPinToInterrupt(buttonInc), addTempHours2, RISING);
        while(!digitalRead(buttonNext)) {}
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        vTaskDelay(500/portTICK_PERIOD_MS);
        
        alarmHours = tempMainProcessHours2;
        alarmMins = tempMainProcessMins2;

        if(alarmHours == 23) {
          if(alarmMins == 59) {
            alarmStopMins = 0;
            alarmStopHours = 0;
          }
          else {
            alarmStopMins = alarmMins + 1;
            alarmStopHours = 23;
          }
        }
        else {
          if(alarmMins == 59) {
            alarmStopMins = 0;
            alarmStopHours = alarmHours + 1;
          }
          else {
            alarmStopMins = alarmMins + 1;
            alarmStopHours = alarmHours + 1;
          }
        }
        setupMode = false;
        digitalWrite(setAlarmMode, LOW);
      }
    }

    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

void stopWatchCallback(void* pvParameters) {
  (void) pvParameters;
  while(1) {
    if (digitalRead(buttonNext) && displayMode == 3 && setupMode == false) {
      digitalWrite(stopWatchMode, HIGH);
      setupMode = true;
      timeMillis = millis();
      while((millis()-timeMillis)<500){};
      
      attachInterrupt(digitalPinToInterrupt(buttonInc), addStopWatchSecs, RISING);
      while(!digitalRead(buttonNext)) {}
      detachInterrupt(digitalPinToInterrupt(buttonInc));
  
      timeMillis = millis();
      while((millis()-timeMillis)<500){};
  
      attachInterrupt(digitalPinToInterrupt(buttonInc), addStopWatchMins, RISING);
      while(!digitalRead(buttonNext)) {}
      detachInterrupt(digitalPinToInterrupt(buttonInc));
  
      timeMillis = millis();
      while((millis()-timeMillis)<500){};

      digitalWrite(stopWatchMode, LOW);
      setupMode = false;

      while(stopWatchMins > 0 || stopWatchSecs > 0) {
        timeMillis = millis();
        while(millis()-timeMillis<1000) {}
        decrementStopWatch();
      }

      digitalWrite(stopWatchFinish, 1);
      timeMillis = millis();
      while(millis()-timeMillis<1000) {}
      digitalWrite(stopWatchFinish, 0);
    }

    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

void alarmCallback(void* pvParameters) {
  (void) pvParameters;
  while(1) {
    if(alarmOn) {
      digitalWrite(base, 1);
      melody_alarm();
      digitalWrite(base, 0);
    }
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

ISR(TIMER1_COMPA_vect){
  /**
   * @brief ISR for timer interrupt
   */
   
  //Reset counter
  TCNT1  = 0;
  //Add seconds
  addSecs();

  //Alarm handler
  if((mins == alarmMins) && (hours == alarmHours)) {
    digitalWrite(alarm, HIGH);
    alarmOn = true;
  }
  else if ((mins == alarmStopMins) && (hours == alarmStopHours)) {
    digitalWrite(alarm, LOW);
    alarmOn = false;
  }
}

void changeMode() {
  if (displayMode == 0) {
    displayMode = 1;
  }
  else if (displayMode == 1) {
    displayMode = 2;
  }
  else if (displayMode == 2) {
    displayMode = 3;
  }
  else if (displayMode == 3) {
    displayMode = 0;
  }
}

void addStopWatchSecs() {
  if (stopWatchSecs == 59) {
    stopWatchSecs = 0;
  }
  else {
    stopWatchSecs++;
  }
}

void addTempMins1() {
  if (tempMainProcessMins1 == 59) {
    tempMainProcessMins1 = 0;
  }
  else {
    tempMainProcessMins1++;
  }
}

void addTempMins2() {
  if (tempMainProcessMins2 == 59) {
    tempMainProcessMins2 = 0;
  }
  else {
    tempMainProcessMins2++;
  }
}

void addStopWatchMins() {
  if (stopWatchMins == 99) {
    stopWatchMins = 0;
  }
  else {
    stopWatchMins++;
  }
}

void addTempHours1() {
  if (tempMainProcessHours1 == 23) {
    tempMainProcessHours1 = 0;
  }
  else {
    tempMainProcessHours1++;
  }
}

void addTempHours2() {
  if (tempMainProcessHours2 == 23) {
    tempMainProcessHours2 = 0;
  }
  else {
    tempMainProcessHours2++;
  }
}

void decrementStopWatch() {
  if (stopWatchSecs == 0) {
    if (stopWatchMins > 0) {
      stopWatchMins--;
      stopWatchSecs = 59;
    }
    else {
      return;
    }
  }
  else {
    stopWatchSecs--;
  }
}

void displayTime(int hours, int minutes) {
  /**
   * @brief function to display whole time
   */
  interface.display(0, (hours/10)%10);
  interface.display(1, hours%10);
  interface.point(1);
  interface.display(2, (minutes/10)%10);
  interface.display(3, minutes%10);
}

void addSecs()
{
  /**
   * @brief function to add seconds
   */
  if (secs < 59 && secs >= 0) {
    secs++;
  }
  else if (secs == 59) {
    secs = 0;
    addMins();
  }
}

void addMins()
{
  /**
   * @brief function to add minutes
   */
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
  /**
   * @brief function to add hours
   */
  if (hours < 23 && hours >= 0) {
    hours++;
  }
  else if (hours == 23) {
    hours = 0;
  }
}

void melody_alarm(){
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < sizeof(melody)/2; thisNote++) {
    tone(alarm, melody[thisNote], 1000/noteDurations[thisNote]);
    vTaskDelay(((1000/portTICK_PERIOD_MS)/noteDurations[thisNote])*timeNote);
  }
  noTone(alarm);
}

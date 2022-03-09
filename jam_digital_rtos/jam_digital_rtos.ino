/**
 * Digital Clock Program
 * by Agape D'sky (13219010), I Gusti Lanang Ari Tri S (13219046)
 * 
 * @brief Digital Clock with set time, timer, and alarm feature
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
uint32_t alarmSecs = 0;
uint32_t alarmStopHours = 0;
uint32_t alarmStopMins = 0;

/*Temporary time params*/
uint32_t tempMainProcessHours1 = 0;
uint32_t tempMainProcessMins1 = 0;
uint32_t tempMainProcessSecs1 = 0;

uint32_t tempMainProcessHours2 = 0;
uint32_t tempMainProcessMins2 = 0;
uint32_t tempMainProcessSecs2 = 0;

/* Timer time params */
uint32_t timerSecs = 0;
uint32_t timerMins = 0;

/*Time adding function*/
void addSecs();
void addMins();
void addHours();

/*Temporary Time Params Adding Functions */
/**
 * @brief templates for temporary time params
 */
void addTempSecs(); // can be written addTempSecs1 for temp var 1, etc
void addTempMins();
void addTempHours();

/* RTOS Thread */
void mainProcessCallback(void* pvParameters);
void displayCallback(void* pvParameters);
void timerCallback(void* pvParameters);

/*Display function*/
void displayTime(int hours, int minutes);
void displayHours(int hours);
void displayMins(int minutes);

/* Misc. functions */
void changeMode();      // display mode change interrupt
void decrementTimer();  // timer sec decrement function
void shutAlarm();       // shut alarm forcefully using interrupt routine
void shutTimer();       // shut timer forcefully using interrupt routine

/* Input Pin Planning */
int buttonInc = 3; // EXTI/Input 1
int buttonNext = 4; // Input1
int buttonMode = 2; // EXTI/Input 2

/* Interfacing Pin Planning */
int CLKPin = 5;
int DIOPin = 6;
TM1637 interface(CLKPin,DIOPin);
int timerFinish = A5;

/* Setup Mode LEDs */
int setTimeMode = 10;
int setAlarmMode = 11;
int timerMode = 12;

/* State variables */
bool forceShutTimer = 0;
uint8_t displayMode = 0;
bool setupMode = 0;
bool alarmOn = 0;

/* Alarm */
/* Alarm pin Mapping */
int base = 9; //transistor base control
int alarm = 8;

/*notes in the melody: */
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

int melody[] = {
  G5,E5,C5,A5,G5,0 ,E5,G5,F5,G5,F5,D5,E5,0, E6,C6,G5,C6,G5,E5,G5,0,A5,G5,E5,C6,G5,A5,G5,B5,A5,F5,D5,C5,0 ,C6
};
int noteDurations[] {
  4,8,4,4,2,8 ,6,6,4,8,4,4,8,8 ,8,8,8,6,8,8,8,8 ,4,8,4,4,2, 4,8,4,8,4,4,4,2,4
};


void setup() {
  Serial.begin(9600);

  // Initialize time
  secs = 50;
  mins = 10;
  hours = 10;
  // Initialize alarm time params
  alarmHours = 0;
  alarmMins = 0;
  alarmSecs = 0;
  alarmStopHours = 0;
  alarmStopMins = 1;
  
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
  pinMode(timerFinish, OUTPUT);
  pinMode(setTimeMode, OUTPUT);
  pinMode(setAlarmMode, OUTPUT);
  pinMode(timerMode, OUTPUT);
  pinMode(alarm, OUTPUT);
  pinMode(base,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(buttonMode), changeMode, RISING);

  /*Initialize Display*/
  interface.init();
  interface.set(7);

  /*Initialize thread*/
  xTaskCreate(displayCallback, "Task A", 128, NULL, tskIDLE_PRIORITY + 4, NULL);
  xTaskCreate(timerCallback, "Task C", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(mainProcessCallback, "Task B", 128, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(alarmCallback, "Task D", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
}

void loop() {
  
}

void displayCallback(void* pvParameters) {
  (void) pvParameters;
  while(1) {
    switch (displayMode) {
      /**
       * @brief switch display based on displayMode (attached to EXTI buttonMode)
       */
      case 0 : displayTime(hours, mins); break;
      case 1 : displayTime(tempMainProcessHours1, tempMainProcessMins1); break;
      case 2 : displayTime(tempMainProcessHours2, tempMainProcessMins2); break;
      case 3 : displayTime(timerMins, timerSecs); break;
    }
    switch (setupMode) {
      /**
       * @brief switch display dim based on setupMode (activated everytime setup mode occurs)
       */
      case false : interface.set(7); break;
      case true : interface.set(1); break;
    }
    // RTOS-safe delay
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

void mainProcessCallback(void* pvParameters) {
  (void) pvParameters;
  while(1) {
    /**
     * @brief entering displayMode 1 (set time)
     * @brief setupMode will be invoked when buttonNext is pressed in this current mode
     */
    if (displayMode == 1) {
      tempMainProcessMins1 = mins;
      tempMainProcessHours1 = hours;
      if (digitalRead(buttonNext) && setupMode == false) {
        // Set setup LED to HIGH
        digitalWrite(setTimeMode, HIGH);
        // Set setupMode to TRUE
        setupMode = true;

        // Thread safe delay, for UX purposes
        timeMillis = millis();
        while((millis()-timeMillis)<500){};

        // Increment minutes value
        attachInterrupt(digitalPinToInterrupt(buttonInc), addTempMins1, RISING);
        while(!digitalRead(buttonNext)) {}
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        // Thread safe delay, for UX purposes
        timeMillis = millis();
        while((millis()-timeMillis)<500){}

        // Increment hours value
        attachInterrupt(digitalPinToInterrupt(buttonInc), addTempHours1, RISING);
        while(!digitalRead(buttonNext)) {}
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        // Thread safe delay, for UX purposes
        vTaskDelay(500/portTICK_PERIOD_MS);

        // Assign changes to current time
        hours = tempMainProcessHours1;
        mins = tempMainProcessMins1;

        // Set setupMode to FALSE
        setupMode = false;
        digitalWrite(setTimeMode, LOW);
      }
    }

    else if (displayMode == 2) {
      /**
       * @brief entering displayMode 2 (set alarm)
       * @brief setupMode will be invoked when buttonNext is pressed in this current mode
       */
      tempMainProcessMins2 = alarmMins;
      tempMainProcessHours2 = alarmHours;
      if (digitalRead(buttonNext) && setupMode == false) {
        // Set Setup LED to HIGH
        digitalWrite(setAlarmMode, HIGH);
        // Set setupMode to TRUE
        setupMode = true;

        // Thread safe delay, for UX purposes
        timeMillis = millis();
        while((millis()-timeMillis)<500){};

        // Increment alarm minutes
        attachInterrupt(digitalPinToInterrupt(buttonInc), addTempMins2, RISING);
        while(!digitalRead(buttonNext)) {}
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        // Thread safe delay, for UX purposes
        timeMillis = millis();
        while((millis()-timeMillis)<500){};

        // Increment alarm hours
        attachInterrupt(digitalPinToInterrupt(buttonInc), addTempHours2, RISING);
        while(!digitalRead(buttonNext)) {}
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        // Thread safe delay, for UX purposes
        vTaskDelay(500/portTICK_PERIOD_MS);

        // Assign changes to current alarm
        alarmHours = tempMainProcessHours2;
        alarmMins = tempMainProcessMins2;

        // Set alarm stop time (1 minute after alarm on event)
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

        // Set setupMode to FALSE
        setupMode = false;
        // Set setup LED to LOW
        digitalWrite(setAlarmMode, LOW);
      }
    }
    // RTOS-safe delay
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

void timerCallback(void* pvParameters) {
  (void) pvParameters;
  while(1) {
    if(displayMode == 3) {
      /**
       * @brief entering displayMode 3 (set timer)
       * @brief setupMode will be invoked when buttonNext is pressed in this current mode
       */
      if (digitalRead(buttonNext) && setupMode == false) {
        // Set setupMode to TRUE
        setupMode = true;              
        // set setup LED to HIGH
        digitalWrite(timerMode, HIGH);                                                                                                                                                                                               printf("%d", setupMode);

        // Thread safe delay, for UX purposes
        timeMillis = millis();
        while((millis()-timeMillis)<500){};

        // Increment timer seconds
        attachInterrupt(digitalPinToInterrupt(buttonInc), addTimerSecs, RISING);
        while(!digitalRead(buttonNext)) {}
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        // Thread safe delay, for UX purposes
        timeMillis = millis();
        while((millis()-timeMillis)<500){};

        // Increment timer minutes (up to 99 mins)
        attachInterrupt(digitalPinToInterrupt(buttonInc), addTimerMins, RISING);
        while(!digitalRead(buttonNext)) {}
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        // Thread safe delay, for UX purposes
        timeMillis = millis();
        while((millis()-timeMillis)<500){};

        // set setup LED to LOW
        digitalWrite(timerMode, LOW);
        // set setupMode to FALSE
        setupMode = false;

        // State variable for interrupt attachments, so that timer killing interrupt will only be attached when the display shows timer mode
        bool attachedOnce = 0;

        // Start timer count
        while(timerMins > 0 || timerSecs > 0) {
          decrementTimer();
          if (displayMode == 3) {
            if (attachedOnce == 0) {
              attachInterrupt(digitalPinToInterrupt(buttonInc), shutTimer, RISING);
              attachedOnce = 1;
            }
            else {
              // do nothing
            }
          }
          else {
            if (attachedOnce == 1) {
              detachInterrupt(digitalPinToInterrupt(buttonInc));
            }
            else {
              // do nothing
            }
          }
          timeMillis = millis();
          while(millis()-timeMillis<1000) {}
        }
        detachInterrupt(digitalPinToInterrupt(buttonInc));

        // Evaluating timer shut (normally or forced)
        if (forceShutTimer == 0) {
          // Timer shutted normally
          digitalWrite(timerFinish, 1);
          digitalWrite(base, 1);
          digitalWrite(alarm, 1);
          timeMillis = millis();
          while(millis()-timeMillis<1000) {}
          digitalWrite(timerFinish, 0);
          digitalWrite(base, 0);
          digitalWrite(alarm, 0);
        }
        else {
          // Timer shutted forcefully, do nothing
        }
        forceShutTimer = 0;
      }
    }
    // RTOS-safe delay
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

void alarmCallback(void* pvParameters) {
  /**
   * @brief alarm callback, additional thread for sounding the alarm
   * @brief this is needed since the program uses PWM
   */
  (void) pvParameters;
  while(1) {
    if(alarmOn) {
      melody_alarm();
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
  if((mins == alarmMins) && (hours == alarmHours) && (secs == alarmSecs)) {
    digitalWrite(alarm, HIGH);
    alarmOn = true;
    // Attach shutAlarm to buttonInc interrupt pin, for manually shutting the alarm
    attachInterrupt(digitalPinToInterrupt(buttonInc), shutAlarm, RISING);
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

void addTimerSecs() {
  if (displayMode == 3){
    if (timerSecs == 59) {
      timerSecs = 0;
    }
    else {
      timerSecs++;
    }
  }
}

void addTempMins1() {
  if (displayMode == 1) {
    if (tempMainProcessMins1 == 59) {
      tempMainProcessMins1 = 0;
    }
    else {
      tempMainProcessMins1++;
    }
  }
}

void addTempMins2() {
  if (displayMode == 2) {
    if (tempMainProcessMins2 == 59) {
      tempMainProcessMins2 = 0;
    }
    else {
      tempMainProcessMins2++;
    }
  }
}

void addTimerMins() {
  if (displayMode == 3) {
    if (timerMins == 99) {
      timerMins = 0;
    }
    else {
      timerMins++;
    }
  }
}

void addTempHours1() {
  if (displayMode == 1) {
    if (tempMainProcessHours1 == 23) {
      tempMainProcessHours1 = 0;
    }
    else {
      tempMainProcessHours1++;
    }
  }
}

void addTempHours2() {
  if (displayMode == 2) {
    if (tempMainProcessHours2 == 23) {
      tempMainProcessHours2 = 0;
    }
    else {
      tempMainProcessHours2++;
    }
  }
}

void decrementTimer() {
  if (timerSecs == 0) {
    if (timerMins > 0) {
      timerMins--;
      timerSecs = 59;
    }
    else {
      return;
    }
  }
  else {
    timerSecs--;
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

void shutAlarm() {
  alarmOn = 0;
  detachInterrupt(buttonInc);
}

void shutTimer() {
  timerMins = 0;
  timerSecs = 0;
  forceShutTimer = 1;
}

void melody_alarm(){
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < sizeof(melody)/2; thisNote++) {
    if (melody[thisNote] == 0) {
      digitalWrite(base, 0);
      vTaskDelay(((1000/portTICK_PERIOD_MS)/noteDurations[thisNote])*timeNote);
    }
    else {
      if (alarmOn) {
        digitalWrite(base, 1);
        tone(alarm, melody[thisNote], 1000/noteDurations[thisNote]);
        vTaskDelay(((1000/portTICK_PERIOD_MS)/noteDurations[thisNote])*timeNote);
      }
    }
    digitalWrite(base, 0);
    
//    delay(((1000)/noteDurations[thisNote])*timeNote);
  }
  noTone(alarm);
}

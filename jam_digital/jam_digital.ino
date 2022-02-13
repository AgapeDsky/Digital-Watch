/**
 * Digital Clock Program
 * by Agape D'sky (13219010), I Gusti Lanang Ari Tri S (13219046)
 * 
 * @brief Digital Clock with set time and alarm feature
 * @brief Schematic is included in the repository
 * 
 * This program uses Arduino Nano (ATmega328p)
 * Data sheet : https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf
 */

 /*Includes*/
#include <TM1637.h>

/*Time params*/
uint32_t secs = 0;
uint32_t mins = 0;
uint32_t hours = 0;
unsigned long timeMillis = 0;
int alarmHours = 0;
int alarmMins = 0;
int alarmStopHours = 0;
int alarmStopMins = 1;

/*Time adding function*/
void addSecs();
void addMins();
void addHours();
void addAlarmMins();
void addAlarmHours();

/*Validations*/
bool timeValidation();

/*Debug functions*/
void printTime();
void printTempTime(int tempHours, int tempMins);

/*Setup function*/
void setTime();

/*Display function*/
void displayTime(int hours, int minutes);
void displayHours(int hours);
void displayMins(int minutes);

/* Input Pin Planning */
int button1 = 3;
int button2 = 4;
int button3 = 2;
int setAlarmButton = 7;

/* Interfacing Pin Planning */
int CLKPin = 5;
int DIOPin = 6;
TM1637 interface(CLKPin,DIOPin);
int setAlarm = 8;
int alarm = 9;

void setup() {
  Serial.begin(9600);

  // Initialize time
  secs = 30;
  mins = 59;
  hours = 23;
  
  cli();                      // Disable Interrupt
  
  TCCR1A = 0;                 // Reset Control Registers 
  TCCR1B = 0;                
  TCCR1B |= B00000100;        // Set Presc to 256 -> 1 second uses 62500 ticks  
  TIMSK1 |= B00000010;        // Enable compare match A
  OCR1A = 62500;              // Set compare value 
   
  sei();                      // Enable Interrupt

  /*Initialize Pin Modes*/
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  pinMode(setAlarmButton, INPUT);
  pinMode(setAlarm, OUTPUT);
  pinMode(alarm, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(button3), setTime, RISING);

  /*Initialize Display*/
  interface.init();
  interface.set(7);
}

void loop() {
  if (!timeValidation()) {
    // Time validity checker, extra protection for faulty occasions
    Serial.println("Time reset needed!");
    setTime();
  }
  // Display time
  displayTime(hours, mins);

  //Set Alarm Condition
  //Invoked within general routine to prevent ISR blocking
  if (digitalRead(setAlarmButton)) {
    
    //Initialize set alarm parameter
    digitalWrite(setAlarm, HIGH);
    interface.set(1);

    //Attach button 1 to increment alarm's minute value
    attachInterrupt(digitalPinToInterrupt(button1), addAlarmMins, RISING);
    while(!digitalRead(button2)) {
      displayTime(alarmHours, alarmMins);
    }
    detachInterrupt(digitalPinToInterrupt(button1));

    //Wait
    timeMillis = millis();
    while((millis()-timeMillis)<500){};

    //Attach button 1 to increment alarm's hour value
    attachInterrupt(digitalPinToInterrupt(button1), addAlarmHours, RISING);
    while(!digitalRead(button2)) {
      displayTime(alarmHours, alarmMins);
    }
    detachInterrupt(digitalPinToInterrupt(button1));

    //Set alarm's stop time
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

    //Exit alarm setup mode
    interface.set(7);
    digitalWrite(setAlarm, LOW);
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
  }
  else if ((mins == alarmStopMins) && (hours == alarmStopHours)) {
    digitalWrite(alarm, LOW);
  }
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

bool timeValidation() {
  /**
   * @brief time validation function
   * @brief checks whether the time showed is valid or not
   */
  if (secs > 59 || mins > 59 || hours > 23 ||
      secs < 0 || mins < 0 || hours < 0) {
        return false;
      }
  else {
    return true;
  }
}

void setTime() {
  /**
   * @brief function to set time. Invoked if the time is invalid OR the user commands the watch to do so
   */

  //Disable interrupt
  cli();

  //Set temporary time variables
  int tempMins = mins;
  int tempHours = hours;

  //Additional parameters for conditional statements
  bool exitLoop = 0;
  bool interfaceState = 0;
  int add = 0;
  interface.set(1);
  
  while(!exitLoop) {
    //Reset timer counter. This operation is safe since the ISR is not needed at the moment
    TCNT1 = 0;
    //Add minutes loop
    while (TCNT1 < 62500/2) {
      if(digitalRead(button2)) {              //Exit loop detection
        exitLoop = 1;
      }
      if (digitalRead(button1)) {
        add = 1;                              //Minute adding detection
      }
    }

    //Add minute based on add variable
    if (add == 1) {                           
      if (tempMins == 59) {
        tempMins = 0;
      }
      else {
        tempMins++;
      }
    }

    //Blink LED for UX purposes
    if (interfaceState) {
      displayTime(tempHours, tempMins);
    }
    else {
      displayHours(tempHours);
    }
    interfaceState = !interfaceState;

    //Reset add state
    add = 0;
  }
  displayTime(tempHours, tempMins);

  interfaceState = 0;
  exitLoop = 0;
  
  //Add hours loop
  //The operation is identical to add minutes loop
  while(!exitLoop) {
    TCNT1 = 0;
    while (TCNT1 < 62500/2) {
      if(digitalRead(button2)) {
        exitLoop = 1;
      }
      if (digitalRead(button1)) {
        add = 1;
      }
    }
    
    if (add == 1) {
      if (tempHours == 23) {
        tempHours = 0;
      }
      else {
        tempHours++;
      }
    }
    if (interfaceState) {
      displayTime(tempHours, tempMins);
    }
    else {
      displayMins(tempMins);
    }
    interfaceState = !interfaceState;
    add = 0;
  }

  //Set temporary variables to the real clock
  mins = tempMins;
  hours = tempHours;
  secs = 0;

  //Reset counter to recount from 0
  TCNT1 = 0;

  interface.set(7);

  //Enable interrupt
  sei();
  return;
}

void printTime() {
  /**
   * @brief basic function for debugging purposes
   */
  Serial.print(hours);
  Serial.print(":");
  Serial.print(mins);
  Serial.print(":");
  Serial.println(secs);
}

void printTempTime(int tempHours, int tempMins) {
  /**
   * @brief basic function for debugging purposes
   */
  Serial.print(tempHours);
  Serial.print(":");
  Serial.println(tempMins);
}

void addAlarmMins() {
  /**
   * @brief function to add alarm's minute parameter
   */
  if(alarmMins == 59) {
    alarmMins = 0;
  }
  else {
    alarmMins++;
  }
}
void addAlarmHours() {
  /**
   * @brief function to add alarm's hour parameter
   */
  if(alarmHours == 23) {
    alarmHours = 0;
  }
  else {
    alarmHours++;
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

void displayHours(int hours) {
  /**
   * @brief function to display only hour
   */
  interface.clearDisplay();
  interface.display(0, (hours/10)%10);
  interface.display(1, hours%10);
  interface.point(1);
}

void displayMins(int minutes) {
  /**
   * @brief function to display only minute
   */
  interface.clearDisplay();
  interface.point(1);
  interface.display(2, (minutes/10)%10);
  interface.display(3, minutes%10);
}

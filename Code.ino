#define LED_PIN             4
#define MOTOR_PIN          10 // PWM  
#define MEASURE_PIN        A0
#define MEASURE_POWER_PIN   8
#define BUTTON_PIN          2 // the number of the pushbutton pin

bool humidyflag = false; // for autostart humidify change to true 
int  meas_del=60; // moisture meausurment delay (sec)
int  hum_del=5;  // humidify delay (sec)
int  moist_lim_high = 700;  
int  moist_lim_low  = 200; 
//int  moist_lim_low_2 = analogRead(POT_PIN_LOW);
int  moist;
int hum_cycle = 0; 
int hum_cycle_limit = 30;
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can't be stored in an int.
unsigned long lastCalibTime = 0;  // the last time the output pin was toggled
unsigned long calibDelay = 5*1000;    // button pressed time delay

void setup()
{
  Serial.begin(9600);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(MEASURE_PIN, INPUT);
  pinMode(MEASURE_POWER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  moist_measure();
}

int moist_measure ()
{
  digitalWrite(MEASURE_POWER_PIN, HIGH);
  delay(500);
  int moist = 1024 - analogRead(MEASURE_PIN);
  digitalWrite(MEASURE_POWER_PIN, LOW);
  return moist;
}

int calibration_high()
{
  digitalWrite(MEASURE_POWER_PIN, HIGH);
  delay(1000);
  moist = 1024 - analogRead(MEASURE_PIN);
  moist_lim_high = moist;
  digitalWrite(MEASURE_POWER_PIN, LOW);
  return moist_lim_high;
}

int moisture ()
{
  analogWrite (MOTOR_PIN, 100); 
  digitalWrite (LED_PIN, HIGH);
  delay (1000*hum_del); //???????? ??????
  analogWrite (MOTOR_PIN, LOW); 
  digitalWrite (LED_PIN, LOW);;
  hum_cycle++;
  return;
}

void loop() {
  
  unsigned long times = millis();

  // read the state of the switch into a local variable:
  int reading = digitalRead(BUTTON_PIN);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastCalibTime = millis();
     }

  
  if ((millis() - lastCalibTime) > calibDelay) calibration_high();
    // whatever the reading is at, it's been there for longer
    // than the calib delay, so take it as the actual current state:

    

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;

    
    if (times % (meas_del*1000) == 0) moist_measure(); //Returning moist value every (meas_del) sec

    if (moist < moist_lim_low)  humidyflag = true;
    if (moist > moist_lim_high) humidyflag = false;
    
   Serial.print("LOW LIMIT-"); Serial.print(moist_lim_low); Serial.print("\t");
   Serial.print("HIGH LIMIT-");Serial.print(moist_lim_high);   Serial.print("\t");
   Serial.print("Current moist val"); Serial.print(moist); Serial.print("\t");
   Serial.print("Humidify?-"); Serial.println(humidyflag);
         
   if (times % (meas_del*1000) == 1500 && (humidyflag)) moisture();

   if (hum_cycle > hum_cycle_limit) { 
      while(hum_cycle > hum_cycle_limit) {
        digitalWrite(LED_PIN,HIGH);
        delay (500);
        digitalWrite(LED_PIN,LOW);
        delay (500);
                                          }
                                     }
  }

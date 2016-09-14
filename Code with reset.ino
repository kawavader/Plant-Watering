#define LED_PIN             4
#define MOTOR_PIN          10 // PWM  
#define MEASURE_PIN        A0
#define MEASURE_POWER_PIN   8
#define BUTTON_PIN          2 // the number of the pushbutton pin

bool humidyflag = false; // for autostart humidify change to true 
int  meas_del=6;                     // moisture meausurment delay (sec)
int  hum_del=1;       // humidify delay (sec)
int  hum_cycle_limit = 2;
int  moist_lim_high = 450;           //can be changed during calibration  
int  moist_lim_low  = 200; 
unsigned long calibDelay = 3 * 1000; // button pressed time delay for calibration

int  moist;
int  hum_cycle = 0; 
int  buttonState;                    // the current reading from the input pin
int  lastButtonState = LOW;          // the previous reading from the input pin
unsigned long lastCalibTime = 0;     // the last time the output pin was toggled

void setup()
{
  Serial.begin(9600);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(MEASURE_PIN, INPUT);
  pinMode(MEASURE_POWER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  //pinMode(RESET_PIN, OUTPUT);
  //digitalWrite(RESET_PIN, HIGH);
  //moist_measure();
}

int moist_measure()
{
  digitalWrite(MEASURE_POWER_PIN, HIGH);
  delay(300);
  int moist = 1024 - analogRead(MEASURE_PIN);
  digitalWrite(MEASURE_POWER_PIN, LOW);
  if (moist <= moist_lim_low)  humidyflag = true;
  if (moist >= moist_lim_high) humidyflag = false;
  Serial.print("Current moist val-"); Serial.println(moist);
  Serial.println(humidyflag);Serial.println(moist_lim_high);
  return;
}

int calibration_high()
{
  digitalWrite(MEASURE_POWER_PIN, HIGH);
  delay(300);
  moist_lim_high = 1024 - analogRead(MEASURE_PIN);
  digitalWrite(MEASURE_POWER_PIN, LOW);
  Serial.println("calibration");
  Serial.print("New High-");Serial.println(moist_lim_high);
  return;
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
  int reading = digitalRead(BUTTON_PIN); // read the state of the switch into a local variable:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastCalibTime = millis();
    }

  if ((millis() - lastCalibTime) > calibDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) calibration_high();
      }
  }
  lastButtonState = reading;

  if (times % (meas_del*1000) == 0) moist_measure(); //Returning moist value every (meas_del) sec
       
  if ((times % (meas_del*1000) == 1100) && (humidyflag)) moisture();

  if (hum_cycle > hum_cycle_limit) { 
    while(hum_cycle > hum_cycle_limit) {
      if (digitalRead(BUTTON_PIN) == HIGH) {
       hum_cycle = 0;
       }
      digitalWrite(LED_PIN,HIGH);
      delay (300);
      digitalWrite(LED_PIN,LOW);
      delay (100);
      }
    }
  }
 

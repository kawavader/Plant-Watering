#define LED_PIN   4
#define MOTOR_PIN   10 // PWM  
#define MEASURE_PIN   A0
#define MEASURE_POWER_PIN   8
#define BUTTON_PIN    2

bool humidyflag = false; // for autostart humidify change to true 
int  meas_del=60; // moisture meausurment delay (sec)
int  hum_del=5;  // humidify delay (sec)
int  moist_lim_high = 700;  
int  moist_lim_low  = 200; 
int  moist_lim_low_2 = analogRead(POT_PIN_LOW);
int  moist, hum_cycle=0, hum_cycle_limit=30;

void setup()
{
  Serial.begin(9600);
  pinMode (MOTOR_PIN, OUTPUT);
  pinMode (MEASURE_PIN, INPUT);
  pinMode (MEASURE_POWER_PIN, OUTPUT);
  pinMode (POT_PIN_LOW, INPUT);
  pinMode (LED_PIN, OUTPUT);
// pinMode(POT_PIN_LOW, INPUT);
  moist_measure();
}

int moist_measure ()
{
  digitalWrite(MEASURE_POWER_PIN, HIGH);
  delay(1000);
  int moist = 1024 - analogRead(MEASURE_PIN);
  digitalWrite(MEASURE_POWER_PIN, LOW);
  return moist;
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

void loop()
{
    unsigned long times = millis();
    
    if (times % (meas_del*1000) == 0) moist_measure(); //Returning moist value every (meas_del) sec

    if (moist < moist_lim_low)  humidyflag = true;
    if (moist > moist_lim_high) humidyflag = false;
    
   Serial.print("LOW LIMIT-"); Serial.print(moist_lim_low); Serial.print("\t");
   Serial.print("LOW LIMIT POT-");Serial.print(moist_lim_low_2);   Serial.print("\t");
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

//#define MOTOR_PIN   13 //????????? ?? ?????
#define MOTOR_PIN   6 //PWM 
#define MEASURE_PIN   A0
#define MEASURE_POWER_PIN   4
#define POT_PIN_LOW     A1
//#define POT_PIN_HIGH    A1

bool humidyflag = false;
int  meas_del=10; //???????? ????? ????????? ?????????? (???????)
int  hum_del=5;  //????? ?????? (???????)

void setup()
{
  Serial.begin(9600);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(MEASURE_PIN, INPUT);
  pinMode(MEASURE_POWER_PIN, OUTPUT);
  pinMode(POT_PIN_LOW, INPUT);
 // pinMode(POT_PIN_LOW, INPUT);
}

int moist_measure ()
{
  digitalWrite(MEASURE_POWER_PIN, HIGH);
  delay(1000);
  int moist1 = 1024 - analogRead(MEASURE_PIN);
  digitalWrite(MEASURE_POWER_PIN, LOW);
  return moist1;
}

void loop()
{
int moist_lim_high = 500; 
//analogRead (POT_PIN_HIGH);  
//int moist_lim_low  = 200; 
int moist_lim_low = analogRead(POT_PIN_LOW);
int moist = moist_measure();

    if (moist < moist_lim_low)  humidyflag = true;
    if (moist > moist_lim_high) humidyflag = false;
    //bool humidy = (moist > moist_lim_low && moist < moist_lim_high && humidyflag) || (humidyflag);

    if (humidyflag) {
    analogWrite (MOTOR_PIN, 90); 
    delay (1000*hum_del); //???????? ??????
    analogWrite (MOTOR_PIN, LOW); 
    }
   delay (1000*meas_del); // ???????? ????? ????????? ??????????
   Serial.print(moist_lim_low);
	 Serial.print(moist);
   Serial.print("\t");
   Serial.println(humidyflag);
  }

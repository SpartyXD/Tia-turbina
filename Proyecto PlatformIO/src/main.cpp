#include <Arduino.h>
#include <misc.h>


//==================== <Objects> ===========================

//------- Motors ----------

void setMotor(int a_pin, int b_pin, int val){
  val = constrain(val, -255.0, 255.0);

  if (val >= 0){
    analogWrite(a_pin, val);
    analogWrite(b_pin, LOW);
  }
  else{
    //reverse
    analogWrite(a_pin, LOW);
    analogWrite(b_pin, val);
  }
}


void setMotors(int left, int right){
  setMotor(AIN1, AIN2, left);
  setMotor(BIN1, BIN2, right);
}


void setTurbine(int val){
  val = constrain(val, 0, 255.0);
  analogWrite(TURBINE_PIN, val);
}


//==================== <Sensors> ===========================

//------- GLOBALS ----------

int v_s_min[SENSOR_COUNT+2];
int v_s_max[SENSOR_COUNT+2];
int v_s[SENSOR_COUNT+2];

int l_threshold = 0;
int r_threshold = 0;

volatile int s_p[SENSOR_COUNT];
bool white_line = true;
bool online;
int OUT_OF_LINE_POS = 255;

int pos;
int l_pos;

//------- SENSOR FUNCTIONS ----------

void Sensors_init(){
  pinMode(LEFT_PIN, INPUT);
  pinMode(RIGHT_PIN, INPUT);
  rep(i, SENSOR_COUNT)
    pinMode(sensorCentralPins[i], INPUT);

  //Init arrays
  rep(i, SENSOR_COUNT+2){
    v_s_min[i] = 4096;
    v_s_max[i] = 0;
  }
}


void readAll(){
  rep(i, SENSOR_COUNT)
    v_s[i] = analogRead(sensorCentralPins[i]);
  
  v_s[LEFT_IDX] = analogRead(LEFT_PIN);
  v_s[RIGHT_IDX] = analogRead(RIGHT_PIN);
}


void sensorCalibration(){
  for (int j = 0; j < 100; j++) {
    delay(30);
    readAll();

    //Update arrays
    rep(i, SENSOR_COUNT+2){
      if(DBG_MODE){
        Serial.print(v_s[i]);
        Serial.print("\t");
      }

      v_s_min[i] = min(v_s_min[i], v_s[i]);
      v_s_max[i] = max(v_s_max[i], v_s[i]);
    }
  }

  l_threshold = (v_s_min[LEFT_IDX] + v_s_max[LEFT_IDX]) * LEFT_THRESHOLD;
  r_threshold = (v_s_min[RIGHT_IDX] + v_s_max[RIGHT_IDX]) * RIGHT_THRESHOLD;


  if(DBG_MODE){
    Serial.print("\n\nMin values\t");
    rep(i, SENSOR_COUNT){
      Serial.print(v_s_min[i]);
      Serial.print("\t");
    }
    Serial.println();

    Serial.print("\n\nMax values\t");
    rep(i, SENSOR_COUNT){
      Serial.print(v_s_max[i]);
      Serial.print("\t");
    }
    Serial.println();

    Serial.print("Thresholds:\t");
    Serial.print(r_threshold);
    Serial.print("\t\t");
    Serial.print(r_threshold);
    Serial.println("\n");
  }

}


void readSensors(){
  volatile int s[SENSOR_COUNT];

  rep(i, SENSOR_COUNT){
    s[i] = analogRead(sensorCentralPins[i]);
    s[i] = constrain(s[i], v_s_min[i], v_s_max[i]);

    if(white_line)
      s_p[i] = map(s[i], v_s_min[i], v_s_max[i], 100, 0);
    else
      s_p[i] = map(s[i], v_s_min[i], v_s_max[i], 0, 100);
  }

  volatile int sum = 0;
  rep(i, SENSOR_COUNT)
    sum += s_p[i];
  
  if(sum > 100)
    online = 1;
  else{
    online = 0;
    sum = 100;
  }

}


int GetPos(){
  readSensors();
  int prom = 0;

  rep(i, SENSOR_COUNT)
    prom += weights[i]*s_p[i];
  
  int sum = 0;
  rep(i, SENSOR_COUNT)
    sum += s_p[i];

  if (online)
    pos = int(100.0 * prom / sum);
  else
    l_pos = (l_pos < 0) ? -OUT_OF_LINE_POS : OUT_OF_LINE_POS;
  
  l_pos = pos;
  return pos;
}

//==================== <Main program> ===========================

//------- FUNCTIONS ----------

//Main
void restart_errors();
void update_error(int &e);
void print_data();
void start_Race();
void end_race();

//Hits
void Read_hits();
void detectGeo();


//------- PARAMETROS GLOBALES ----------

//Velocidades
#define N_SPEEDS 3
int CURRENT_SPEED = 0;

// base | kp | kd | ki | turbine
float PIDs[N_SPEEDS][5] = {
  {100, 0.65, 25, 0.00001, 0},
  {150, 0.8, 30, 0.00001, 0},
  {175, 0.875, 32, 0.00001, 0}
};

//PID
int base = 0;
int turbine_speed = 0;
float Kp = 0;
float Kd = 0;
float Ki = 0;

#define N_ERRORS 25
int error_sum = 0;
int errors[N_ERRORS];
int current_idx = 0;

int setpoint = 0;
int error = 0;
int last_error = 0;
int max_power = 250;

//Hits
int final_hit_count = 0;
bool ON_RACE = false;

int l_geo=0;
int ll_geo = 0;
int lll_geo = 0;
int geo = 0;
    

int HL, HR = 0;

//------- Function implementations ----------

void update_PID_data(int idx){
  idx = constrain(idx, 0, N_SPEEDS);

  base = PIDs[idx][0];
  Kp = PIDs[idx][1];
  Kd = PIDs[idx][2];
  Ki = PIDs[idx][3];
  turbine_speed = PIDs[idx][4];
}

void restart_errors(){
    error = 0;
    error_sum = 0;
    last_error = 0;
    current_idx = 0;

    rep(i, N_ERRORS)
        errors[i] = 0;
}


void update_error(int &e){
    error_sum -= errors[current_idx];
    errors[current_idx] = e;
    error_sum += e;
    current_idx = (current_idx+1)%N_ERRORS;
}


void print_data(){
    if(DBG_MODE){
      //Imprimir
      Serial.println("Data:\n");
      Serial.println("KP: " + String(Kp));
      Serial.println("KD: " + String(Kd));
      Serial.println("KI: " + String(Ki));
      Serial.println("Base: " + String(base));
      Serial.println("Turbine speed: " + String(turbine_speed));
      Serial.println("Max power: " + String(max_power));
      Serial.println("White line: " + String(white_line));
      Serial.println("=====================\n");
    }
}


void start_Race(){
    print_data();
    ON_RACE = true;
    final_hit_count = 0;

    update_PID_data(CURRENT_SPEED);
    restart_errors();
}


void end_race(){
    //Stop all
    setMotors(0, 0);
    setTurbine(0);
    ON_RACE = false;
    final_hit_count = 0;

    l_geo = ll_geo = lll_geo = 0;
    geo = 0;

    CURRENT_SPEED = (CURRENT_SPEED + 1) % N_SPEEDS;
}


//==================== <Hits> ===========================

void Read_hits(){
    HL = analogRead(LEFT_PIN);
    HR = analogRead(RIGHT_PIN);

    HL = (HL > l_threshold) ? 0 : 1;
    HR = (HR > r_threshold) ? 0: 1;

    if(!white_line){
        HL = !HL;
        HR = !HR;
    }
}


void detectGeo() {
    Read_hits();

    //Detect actual geo
    if(HL == HR)
        geo = (HL == 0) ? 0 : 3; // 0 0 | 1 1
    else
        geo = (HL == 1) ? 1 : 2; // 1 0 | 0 1

    //Check for changes
    if(l_geo == geo)
        return;

    if(geo == 0 && l_geo == 2 && ll_geo == 0){
      final_hit_count++;

      if(final_hit_count >= 2)
        end_race();
    }
    
    //Update geo vals
    lll_geo = ll_geo;
    ll_geo = l_geo;
    l_geo = geo;
}


//==================== <main> ===========================

void setup(){
  if (DBG_MODE){
    Serial.begin(115200);
    Serial.println("Initializing systems...");
  }

  setLed(POWER_LED, 255);
  setLed(STATUS_LED, 0);
  setLed(MODE_LED, 0);
  
  init_pins();
  Sensors_init();
  setMotors(0, 0);
  setTurbine(0);

  delay(500);

  setLed(POWER_LED, 255);
  setLed(STATUS_LED, 0);
  setLed(MODE_LED, 255);
  
  WaitBoton(A);
  setLed(POWER_LED, 0);
  sensorCalibration();
}


void loop(){
  if(digitalRead(BUTTON_PIN_A))
    end_race();

  int p = GetPos();
  detectGeo();

  if(!ON_RACE){
    setLed(POWER_LED, 0);
    setLed(STATUS_LED, 255);
    setLed(MODE_LED, 0);

    WaitBoton(A);

    setLed(POWER_LED, 255);
    setLed(STATUS_LED, 255);
    setLed(MODE_LED, 255);

    ON_RACE = true;
    return;
  }

  //Get error
  int error = p - setpoint;
  int d = error - last_error;
  update_error(error);
  last_error = error;

  //Apply PID
  int turn_power = int(Kp * error) + int(Kd * (d)) + int(Ki * (error_sum));
  turn_power = constrain(turn_power, -max_power, max_power);
  
  setMotors(base + turn_power, base - turn_power);
  last_error = error;
}



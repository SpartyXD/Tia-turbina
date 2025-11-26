#include <Arduino.h>

//Macros
#define rep(i, n) for(int i=0; i<n; i++)
#define DBG_MODE true

//==================== <Pines> ===========================

//------- Perifericos ----------
#define BUTTON_PIN_A 2
#define BUTTON_PIN_B 5
#define PINBUZZER 10

//------- Motores ----------
//Izq
# define AIN1 9
# define AIN2 8
# define PWMA 5

//Der
# define BIN1 7 
# define BIN2 4 
# define PWMB 6 

//------- Sensores IR ----------

#define SENSOR_COUNT 6
const int sensorCentralPins[SENSOR_COUNT] = {A6, A5, A4, A3, A2, A1};
const double weights[SENSOR_COUNT] = {-2.5, -1.5, -0.5, 0.5, 1.5, 2.5};

#define LEFT_PIN A7
#define RIGHT_PIN A0

//Pin manager
const int INPUT_PINS[] = {};
const int OUTPUT_PINS[] = {};

const int N_INPUT = sizeof(INPUT_PINS)/sizeof(int);
const int N_OUTPUT = sizeof(OUTPUT_PINS)/sizeof(int);


//------- Code ----------

void init_pins(){
  pinMode(BUTTON_PIN_A, INPUT); 
  pinMode(BUTTON_PIN_B, INPUT);
  pinMode(PINBUZZER, OUTPUT);
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  rep(i, N_INPUT)
      pinMode(INPUT_PINS[i], INPUT);
  rep(i, N_OUTPUT)
      pinMode(OUTPUT_PINS[i], OUTPUT);  
}



//==================== <Objects> ===========================

//------- Motores ----------

void MotorIz(int val) {
  val = constrain(val, -255.0, 255.0);

  if (val >= 0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else{
    //atras
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    val *= -1;
  }

  // Setea Velocidad
  analogWrite(PWMA, val);
}


void MotorDe(int val){
  val = constrain(val, -255.0, 255.0);

  if (val >= 0){
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } 
  else{
    //atras
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    val *= -1;
  }

  // Setea Velocidad
  analogWrite(PWMB, val);
}


void Motores(int left, int right){
  MotorIz(left);
  MotorDe(right);
}


//------- Buzzer ----------

void beep(unsigned int frec=2000, unsigned int dur=100){
  tone(PINBUZZER, frec, dur); 
  delay(200);
}


void startupBeep(){
    int C5 = 523;  
    int E5 = 659;  
    int G5 = 784;  
    int shortDuration = 200;
    int longDuration = 400; 

    beep(C5, shortDuration);
    beep(E5, shortDuration);
    beep(G5, longDuration);
}


void chillBeep(){
    beep(700, 100);
    delay(50);
    beep(1000, 100);
    delay(50);
    beep(1300, 100);
    delay(150);
    beep(1000, 150);
}


void calibrateBeep(){
    beep(700, 100);
    delay(50);
    beep(1000, 100);
    delay(50);
    beep(1300, 100);
    delay(150);
    beep(1000, 150);
}


//------- Boton ----------

void WaitBoton(){
  if(DBG_MODE){
    Serial.println("\nEsperando boton\n");
  }
  
  delay(1000);

  while(!digitalRead(BUTTON_PIN_A)){}
  
  beep();
}


//==================== <Sensors> ===========================

//------- GLOBALS ----------

int v_s_min[SENSOR_COUNT+2];
int v_s_max[SENSOR_COUNT+2];
int v_s[SENSOR_COUNT+2];


int umbral_l = 0;
int umbral_r = 0;

volatile int s_p[SENSOR_COUNT];
bool white_line = true;
bool online;
int OUT_OF_LINE_POS = 255;

int pos;
int l_pos;

#define LEFT_IDX SENSOR_COUNT
#define RIGHT_IDX SENSOR_COUNT+1

//------- FUNCIONES SENSORES ----------

void Sensors_init(){
  //Modo pines
  pinMode(LEFT_PIN, INPUT);
  pinMode(RIGHT_PIN, INPUT);
  rep(i, SENSOR_COUNT)
    pinMode(sensorCentralPins[i], INPUT);

  //Inicializar arrays
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


void calibracion(){
  for (int j = 0; j < 100; j++) {
    delay(30);
    readAll();

    //Actualizar arrays
    rep(i, SENSOR_COUNT+2){
      if(DBG_MODE){
        Serial.print(v_s[i]);
        Serial.print("\t");
      }

      v_s_min[i] = min(v_s_min[i], v_s[i]);
      v_s_max[i] = max(v_s_max[i], v_s[i]);
    }
  }

  umbral_l = (v_s_min[LEFT_IDX] + v_s_max[LEFT_IDX]) * 0.4;
  umbral_r = (v_s_min[RIGHT_IDX] + v_s_max[RIGHT_IDX]) * 0.6;

  beep();
  beep();

  if(DBG_MODE){
    Serial.print("\n\nMinimos\t");
    rep(i, SENSOR_COUNT){
      Serial.print(v_s_min[i]);
      Serial.print("\t");
    }
    Serial.println();

    Serial.print("\n\nMaximos\t");
    rep(i, SENSOR_COUNT){
      Serial.print(v_s_max[i]);
      Serial.print("\t");
    }
    Serial.println();

    Serial.print("Umbrales:\t");
    Serial.print(umbral_r);
    Serial.print("\t\t");
    Serial.print(umbral_r);
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

//==================== <LauchaBot> ===========================

//------- FUNCIONES ----------

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
#define CANT_VELOCIDADES 3
int CURRENT_SPEED = 0;

float PIDs[CANT_VELOCIDADES][4] = {
  {100, 0.65, 25, 0.00001},
  {150, 0.8, 30, 0.00001},
  {175, 0.875, 32, 0.00001}
};

//PID
int base = 0;
float Kp = 0;
float Kd = 0;
float Ki = 0;

#define CANT_ERRORES 25
int error_sum = 0;
int errors[CANT_ERRORES];
int current_idx = 0;

int setpoint = 0;
int error = 0;
int last_error = 0;
int pot_limite = 250;

//Hits
int fin = 0;
bool ON_RACE = false;

int l_geo=0;
int ll_geo = 0;
int lll_geo = 0;
int geo = 0;
    

int HL, HR = 0;

//------- FUNCIONES MAIN ----------

void update_PID_data(int idx){
  idx = constrain(idx, 0, CANT_VELOCIDADES);

  base = PIDs[idx][0];
  Kp = PIDs[idx][1];
  Kd = PIDs[idx][2];
  Ki = PIDs[idx][3];
}

void restart_errors(){
    error = 0;
    error_sum = 0;
    last_error = 0;
    current_idx = 0;

    rep(i, CANT_ERRORES)
        errors[i] = 0;
}


void update_error(int &e){
    error_sum -= errors[current_idx];
    errors[current_idx] = e;
    error_sum += e;
    current_idx = (current_idx+1)%CANT_ERRORES;
}


void print_data(){
    if(DBG_MODE){
      //Imprimir
      Serial.println("Datos:\n");
      Serial.println("KP: " + String(Kp));
      Serial.println("KD: " + String(Kd));
      Serial.println("KI: " + String(Ki));
      Serial.println("Base: " + String(base));
      Serial.println("Pot limite: " + String(pot_limite));
      Serial.println("White line: " + String(white_line));
      Serial.println("=====================\n");
    }
}


void start_Race(){
    ON_RACE = true;
    fin = 0;

    update_PID_data(CURRENT_SPEED);
    restart_errors();

    beep(2000, 100);
    beep(2300, 200);
}


void end_race(){
    //Parar todo
    Motores(0, 0);
    ON_RACE = false;
    fin = 0;

    l_geo = ll_geo = lll_geo = 0;
    geo = 0;

    //Musiquita de final
    chillBeep();
    CURRENT_SPEED = (CURRENT_SPEED + 1) % CANT_VELOCIDADES;
}


//==================== <Hits> ===========================

void Read_hits(){
    HL = analogRead(LEFT_PIN);
    HR = analogRead(RIGHT_PIN);

    HL = (HL > umbral_l) ? 0 : 1;
    HR = (HR > umbral_r) ? 0: 1;

    if(!white_line){
        HL = !HL;
        HR = !HR;
    }
}


void detectGeo() {
    Read_hits();

    //Detectar geo actual
    if(HL == HR)
        geo = (HL == 0) ? 0 : 3; // 0 0 | 1 1
    else
        geo = (HL == 1) ? 1 : 2; // 1 0 | 0 1

    //Ver si algo ha cambiado xd
    if(l_geo == geo)
        return;

    if(geo == 0 && l_geo == 2 && ll_geo == 0){
      fin++;

      if(fin >= 2)
        end_race();
    }
    
    //Actualizar valores de geo
    lll_geo = ll_geo;
    ll_geo = l_geo;
    l_geo = geo;
}


//==================== <main> ===========================

void setup(){
  if (DBG_MODE){
    Serial.begin(9600);
    Serial.println("Iniciando todo..");
  }
  
  init_pins();
  Sensors_init();
  Motores(0, 0);

  delay(500);

  startupBeep();
  delay(500);
  calibrateBeep();
  
  WaitBoton();
  calibracion();
}


void loop(){
  // if(digitalRead(BUTTON_PIN_A))
  //   end_race();

  int p = GetPos();
  detectGeo();

  if(!ON_RACE){
    WaitBoton();
    ON_RACE = true;
    return;
  }

  //Obtener error
  int error = p - setpoint;
  int d = error - last_error;
  update_error(error);
  last_error = error;

  //Obtener PID
  int pot_giro = int(Kp * error) + int(Kd * (d)) + int(Ki * (error_sum));
  pot_giro = constrain(pot_giro, -pot_limite, pot_limite);
  
  Motores(base + pot_giro, base - pot_giro);
  last_error = error;
}



#include <Arduino.h>

#define rep(i, n) for(int i=0; i<n; i++)
#define DBG_MODE true

//-------------- Pines ---------------

//Motor A
#define AIN1 9
#define AIN2 8
#define PWMA 5

//Motor B
#define BIN1 7
#define BIN2 4
#define PWMB 6

//boton
#define BOTON 2

// Buzzer
#define BUZZER 10

//------------------- <Perifericos> -----------------

// Función para emitir un sonido
void beep(unsigned int frec=2000, unsigned int dur=100){
  tone(BUZZER, frec, dur); // Emite un tono de 2000 Hz durante 100 ms
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

void WaitBoton() {
  // Entra en un bucle infinito de espera hasta que se presione el botón
  while (!digitalRead(BOTON)); // No hace nada dentro del bucle, solo espera
  beep(); // Se llama a la función beep() cuando se presiona el botón
}

//----------------- <IR Sensors> -------------

#define SENSOR_COUNT 6
const int sensorCentralPins[SENSOR_COUNT] = {A6, A5, A4, A3, A2, A1};
const double weights[SENSOR_COUNT] = {-2.5, -1.5, -0.5, 0.5, 1.5, 2.5};

#define LEFT_PIN A7
#define RIGHT_PIN A0


//------- ARRAYS GLOBALES ----------

int v_s_min[SENSOR_COUNT+2];
int v_s_max[SENSOR_COUNT+2];
int v_s[SENSOR_COUNT+2];


int umbral_l = 0;
int umbral_r = 0;

int Hiz, Hde;

volatile int s_p[SENSOR_COUNT];
bool white_line = true;
bool online;
int OUT_OF_LINE_ERROR = 2550;

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

    //Imprimir lecturas y actualizar arrays
    rep(i, SENSOR_COUNT+2){
      // Serial.print(v_s[i]);
      // Serial.print("\t");

      v_s_min[i] = min(v_s_min[i], v_s[i]);
      v_s_max[i] = max(v_s_max[i], v_s[i]);
    }
    // Serial.println();
  }

  beep();
  beep();

  // Serial.print("\n\nMinimos\t");
  // rep(i, SENSOR_COUNT){
  //   Serial.print(v_s_min[i]);
  //   Serial.print("\t");
  // }
  // Serial.println();


  // Serial.print("\n\nMaximos\t");
  // rep(i, SENSOR_COUNT){
  //   Serial.print(v_s_max[i]);
  //   Serial.print("\t");
  // }
  // Serial.println();

  umbral_l = (v_s_min[LEFT_IDX] + v_s_max[LEFT_IDX]) * 0.4;
  umbral_r = (v_s_min[RIGHT_IDX] + v_s_max[RIGHT_IDX]) * 0.6;

  // Serial.print("Umbrales:\t");
  // Serial.print(umbral_r);
  // Serial.print("\t\t");
  // Serial.print(umbral_r);
  // Serial.println("\n");
  
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
  
  if(sum >= SENSOR_COUNT*100)
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
    l_pos = (l_pos < 0) ? -OUT_OF_LINE_ERROR : OUT_OF_LINE_ERROR;
  
  l_pos = pos;
  return pos;
}

//----------- <Motores> ------------------


//accionamiento motor izquierdo
void Motorde(int value) {
  if ( value >= 0 ) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMB, value);
}
//accionamiento motor derecho
void Motoriz(int value) {
  if ( value >= 0 ) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMA, value);
}

//Accionamiento de motores
void Motor(int left, int righ) {
  Motoriz(righ);
  Motorde(left);
}

//------------- <Hits> ----------------------

//Variables SensorLat
int fin = 0;
int geo1 = 0, geo2 = 0, geo3 = 0, geo4 = 0, geo5 = 0;
int geo = 0;

void funcionCruce() {
  // tone(BUZZER, 2000, 100);
}

void funcionHitoDe() {
  tone(BUZZER, 2000, 100);
  switch (fin) {
    case 2:
      delay(50);
  
      Motor(0, 0);
      WaitBoton();
      delay(3000);
      
      break;
    case 4:
      delay(50);
      
      Motor(0, 0);
      WaitBoton();
      delay(3000);
      
      break;
    case 6:
      delay(50);
      
      Motor(0, 0);
      WaitBoton();
      delay(3000);
      
      break;
    case 8:
      delay(50);
      
      Motor(0, 0);
      WaitBoton();
      delay(3000);
      
      break;
    case 10:
      delay(50);
      
      Motor(0, 0);
      WaitBoton();
      delay(3000);
      
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}

 
void funcionHitoIz() {
  // tone(BUZZER, 2000, 100);
  //aqui va el codigo para los cambios de curvatura
}


void hitos() {
  Hiz = analogRead(LEFT_PIN);
  Hde = analogRead(RIGHT_IDX);

  if (Hiz < umbral_l) {
    Hiz = 1;
  } else {
    Hiz = 0;
  }

  if (Hde < umbral_r) {
    Hde = 1;
  } else {
    Hde = 0;
  }
  
  if (Hiz == 0 && Hde == 0) {
    geo = 0;
  }
  if (Hiz == 1 && Hde == 0) {
    geo = 1;
  }
  if (Hiz == 0 && Hde == 1) {
    geo = 2;
  }
  if (Hiz == 1 && Hde == 1) {
    geo = 3;
  }

  if (geo1 != geo) {
    if (geo == 0 && geo1 == 1 && geo2 == 0) {
      tone(BUZZER, 1000, 100);
       funcionHitoIz();
    }
    if (geo == 0 && geo1 == 2 && geo2 == 0) {
         tone(BUZZER, 1000, 100);
      fin++;
      funcionHitoDe();
    }
    if (geo == 0 && ((geo1 == 3) || (geo2 == 3) || (geo3 == 3))) {

      funcionCruce();
    }

    geo5 = geo4;
    geo4 = geo3;
    geo3 = geo2;
    geo2 = geo1;
    geo1 = geo;
  }
}

//------------------- <main> ---------------

//Constantes
float KP = 0.65; // Constante Proporcional
float KI = 0.00001;    // Constante Integral
float KD = 24;  // KD

//Velocidades
int Tp1 = 100;
int Tp2 = 110;
int Tp3 = 110;
int Tp4 = 110;
int Tp5 = 110;

//SensorLat
int HL, HR = 0;

//Variables
int error = 0;      //Proporcional
int integral = 0;   //Integral
int lastError = 0;  //Ultimo error
int derivada = 0;   //Derivada
int cont = 0;

//referencia y velocidad base
int ref = 0;
int Tp = 150;



void setup(){
  if (DBG_MODE)
    Serial.begin(9600);

  //Seteo pines motor
  pinMode(BIN2  , OUTPUT);
  pinMode(BIN1  , OUTPUT);
  pinMode(PWMB  , OUTPUT);
  pinMode(AIN1  , OUTPUT);
  pinMode(AIN2  , OUTPUT);
  pinMode(PWMA  , OUTPUT);

  //seteo pin boton
  pinMode(BOTON, INPUT);

  //seteo buzzer
  pinMode(BUZZER, OUTPUT);

  //Seteo sensores
  Sensors_init();

  startupBeep();
  WaitBoton();
  calibracion();

  WaitBoton();
}

void loop() {
//LEEMOS LA SEÑAL DE LOS SENSORES
int posicion = GetPos();
                                                                                        
error = ((posicion)-(ref)); /// ERROR

hitos();
 

/////FRENOS////

if(error <= -OUT_OF_LINE_ERROR){
  Motor(-200, 200);
} 
else if (error>=OUT_OF_LINE_ERROR){
  Motor(200, -200);
}

/////////////////////////
else{
    derivada = (error - lastError); /// ERROR MENOS EL ERROR ANTERIOR , DERIVATIVO
    integral = (error + lastError); //INTEGRAL
      
    int giro = ( error * KP ) + ( derivada * KD ) + ( integral * KI );

    ///VELOCIDAMAX =VELOCIDAD PUNTA , V

    //Seteo velocidad ruedas
    int velizq = Tp + giro;
    int velder = Tp - giro;

    //Movimiento
    Motor(velizq, velder);  

    lastError = error;
  }

  
}

 
#include <PID_v1.h>

//-----------Variables Globales--------------
int SensorValue = 0;
int pv = 0;
int OutputValue = 0;
char tx_buff[10]; //buffer de salida
byte posicion = 0;

const int TriggerPin = 2; // pin del TRIGGER
const int EchoPin = 15; // pin del ECHO

long Duracion = 0;
long fDistancia(long tiempo);

//Variables de entrada salida y referencia
double Referencia, Entrada, Salida;

//Ganancias del PID
double Kp=3, Ki=2, Kd=0;
PID myPID(&Entrada, &Salida, &Referencia, Kp, Ki, Kd, DIRECT);

float error = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(EchoPin,INPUT);
  pinMode(TriggerPin,OUTPUT);
  
  Referencia = 50;
  
  myPID.SetMode(AUTOMATIC);

  analogReadResolution(8);
}

void loop()
{
  digitalWrite(TriggerPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(TriggerPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(TriggerPin, LOW);

  Duracion = pulseIn(EchoPin,HIGH);
  
  long Distancia_mm = fDistancia(Duracion);
  
  Entrada = map(Distancia_mm,25,226,0,255);
  Referencia = analogRead(4);
  
  myPID.Compute();
  //Salida 
  dacWrite(25, Salida); // Accion de control 
  //int errorrr = 1.3*Referencia-0.8*Entrada;
  //if(errorrr) errorrr = 0;
  
  Serial.print(Referencia);
  Serial.print(" ");
  Serial.println(Entrada);

  error = Referencia-Entrada;
  if(error <= 0)error = 0;
  dacWrite(26,error);
  delay(100);
}
long fDistancia(long tiempo)
{
  long DistanceCalc; 
  DistanceCalc = (tiempo /2.9) / 2; // distancia en milimetros
  // DistanceCalc = (tiempo /29) / 2; // Cálculos en centímetros
  // DistanceCalc = (tiempo / 74) / 2; // Cálculos en pulgadas  
  return DistanceCalc;
}
void pid(){
  double myInput=0;
  double input = myInput;
      double error = *mySetpoint - input;


      ITerm += (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;


      double dInput = (input - lastInput);
 
      /*Compute PID Output*/
      double output = kp * error + ITerm - kd * dInput;
  }

int disparo = 9;
int eco = 8;
unsigned long duracion; // se utilizan datos long debido a la respuesta del ultrasonido
float pulgadas, cm;
int IN3 = 5;    // Input3 conectada al pin 5
int IN4 = 4;    // Input4 conectada al pin 4 
int ENB = 3;    // ENB conectada al pin 3 de Arduino
int estado=1;
int PWM_valor=0;
const float Kp=5,Ki=1,Kd=1;// son constantes pero flotantes y esas hay que estarlas moviendo para sintonizar el PID
float error_ant,error=1,error_p,error_i,error_d;

void setup()
{
  Serial.begin(9600);
  pinMode(disparo, OUTPUT);
  pinMode(eco, INPUT);
  pinMode (ENB, OUTPUT); 
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
}

void loop()
{
  if (Serial.available() > 0) {
      Serial.println("ALGO 1");
      estado=Serial.read();
      estado=estado-48;
      Serial.println(estado);
    }
  while(estado!=0){
    if (Serial.available() > 0) {
      Serial.println("ALGO 2");
      estado=Serial.read();
      estado=estado-48;
      if(estado>0 && estado<5){
        Serial.print("Piso ");
        Serial.println(estado);
        float posicion = Obtener_Posicion();
        Serial.print("Posicion actual ");
        Serial.println(posicion);
        boolean si=ajustar_posicion(posicion,estado);
      }else if(estado==0){
        Serial.println("Saliendo del ciclo infinito");
      }else{
        Serial.println("Opcion no valida");
      }
      
    }
  }
  

  delay(1000);
  //motor
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  // Aplicamos PWM al pin ENB, haciendo girar el motor, cada 2 seg aumenta la velocidad
  analogWrite(ENB,255);
  delay(1000);
  // Apagamos el motor y esperamos 5 seg
  analogWrite(ENB,0);
  
}

float Obtener_Posicion(){
  digitalWrite(disparo, LOW);
  delay(5);
  digitalWrite(disparo, HIGH);
  delay(5);
  digitalWrite(disparo, LOW);
  duracion = pulseIn(eco, HIGH);
  pulgadas = msPulgadas(duracion);
  cm = msCentimetros(duracion);
  return cm;
}

float msPulgadas(long microsegundos){
  return ( (float)microsegundos / (74.0 * 2.0) );
}

float msCentimetros(long microsegundos){
  return ( (float)microsegundos / (29.0 * 2.0) );
}

boolean ajustar_posicion(float posicion_inicial,int piso){
  Serial.println("Entrando a ajustar_posicion ");
  Serial.print("POSICION ");
  Serial.println(posicion_inicial);
  Serial.print("PISO ");
  Serial.println(piso);
  Serial.print("ERROR ");
  Serial.println(error);
  switch(piso){
    case 1:
      while(error!=0){
        error_ant=error;
        error = 2.88 - Obtener_Posicion();
        error_p = Kp*error;
        error_i = Ki*error + error_i;
        error_d = Kd*(error-error_ant);
        PWM_valor = (int)(error_p + error_i + error_d);
        Serial.print("ERROR anterior ");
        Serial.println(error_ant);
        Serial.print("ERROR ");
        Serial.println(error);
        Serial.print("ERROR p ");
        Serial.println(error_p);
        Serial.print("ERROR i ");
        Serial.println(error_i);
        Serial.print("ERROR d ");
        Serial.println(error_d);
        Serial.print("PWM VALOR ");
        Serial.println(PWM_valor);
        delay(5000);
        if(error>0){
          Serial.println("AJUSTANDO BAJANDO");
          //bajar
        }
        if(error<0){
          Serial.println("AJUSTANDO SUBIENDO");
          //subir
        }
        if(error==0){
          Serial.println("AJUSTADO");
        }
      }
    break;
    case 2:
    break;
    case 3:
    break;
    case 4:
    break;
  }
  return true;
}

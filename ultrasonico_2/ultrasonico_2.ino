int disparo = 8;
int eco = 9;
unsigned long duracion; // se utilizan datos long debido a la respuesta del ultrasonido
float pulgadas, cm;
int IN3 = 5;    // Input3 conectada al pin 5
int IN4 = 4;    // Input4 conectada al pin 4 
int ENB = 3;    // ENB conectada al pin 3 de Arduino
int estado=1;
int PWM_valor=0;
//Pues ki menor que 1 Kd menor a 2 y kp menor a 15Kp   .2 Kd   .1 Ki    .00001
const float Kp=.31,Ki=.00011,Kd=.31;// son constantes pero flotantes y esas hay que estarlas moviendo para sintonizar el PID
float error_ant,error=1,error_p,error_i,error_d;
int motor_estable=185;

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
        Serial.println("");
        ajustar_posicion(posicion,estado);
      }else if(estado==0){
        Serial.println("Saliendo del ciclo infinito");
        break;
      }else{
        Serial.println("Opcion no valida");
      }
      
    }
  }
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

void ajustar_posicion(float posicion_inicial,int piso){
  Serial.println("***************Entrando a ajustar_posicion***************************");
  Serial.print("=>POSICION ");
  Serial.println(posicion_inicial);
  Serial.print("=>PISO ");
  Serial.println(piso);
  error=0;
  Serial.print("=>ERROR ");
  Serial.println(error);
  switch(piso){
    case 1:
      while(true){
        Serial.println("-------------------------Ciclo hasta que llegue al piso-----------------------");
        Serial.print("Error = ");
        Serial.println(error);
        error_ant=error;
        error = 45.60 - Obtener_Posicion();
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
        if(error>0){
          Serial.println("B A J A N D O");
          PWM_valor=85-PWM_valor;
        }else{
          Serial.println("S U B I E N D O");
          PWM_valor=210+PWM_valor;
        }
        Ajuste_motor(PWM_valor);
        delay(90);
        digitalWrite (IN3, HIGH);
        digitalWrite (IN4, LOW);
        analogWrite(ENB,motor_estable);
        delay(200);
        if((int)error==0){
          Serial.println("AJUSTADO");
          digitalWrite (IN3, HIGH);
          digitalWrite (IN4, LOW);
          analogWrite(ENB,motor_estable-20);
          break;
        }
        Serial.print("/////////////////////Errror a comparara/////////////////////////////// ");
        Serial.println(error,4);
      }
    break;
    case 2:
      while(true){
        Serial.println("-------------------------Ciclo error diferente de 0-----------------------");
        Serial.print("Error = ");
        Serial.println(error);
        error_ant=error;
        error = 38.01 - Obtener_Posicion();
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
       if(error>0){
          Serial.println("B A J A N D O");
          PWM_valor=85-PWM_valor;
        }else{
          Serial.println("S U B I E N D O");
          PWM_valor=210+PWM_valor;
        }
        Ajuste_motor(PWM_valor);
        delay(90);
        digitalWrite (IN3, HIGH);
        digitalWrite (IN4, LOW);
        analogWrite(ENB,motor_estable);
        delay(200);
        if((int)error==0){
          Serial.println("AJUSTADO");
          digitalWrite (IN3, HIGH);
          digitalWrite (IN4, LOW);
          analogWrite(ENB,motor_estable);
          break;
        }
        Serial.print("/////////////////////Errror a comparara/////////////////////////////// ");
        Serial.println(error,4);
      }
    break;
    case 3:
      while(true){
        Serial.println("-------------------------Ciclo error diferente de 0-----------------------");
        Serial.print("Error = ");
        Serial.println(error);
        error_ant=error;
        error = 22.88 - Obtener_Posicion();
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
        if(error>0){
          Serial.println("B A J A N D O");
          PWM_valor=85-PWM_valor;
        }else{
          Serial.println("S U B I E N D O");
          PWM_valor=210+PWM_valor;
        }
        Ajuste_motor(PWM_valor);
        delay(90);
        digitalWrite (IN3, HIGH);
        digitalWrite (IN4, LOW);
        analogWrite(ENB,motor_estable);
        delay(200);
        if((int)error==0){
          Serial.println("AJUSTADO");
          digitalWrite (IN3, HIGH);
          digitalWrite (IN4, LOW);
          analogWrite(ENB,motor_estable);
          break;
        }
        Serial.print("/////////////////////Errror a comparara/////////////////////////////// ");
        Serial.println(error,4);
      }
    break;
    case 4:
      while(true){
        Serial.println("-------------------------Ciclo error diferente de 0-----------------------");
        Serial.print("Error = ");
        Serial.println(error);
        error_ant=error;
        error = 11.74 - Obtener_Posicion();
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
        if(error>0){
          Serial.println("B A J A N D O");
          PWM_valor=85-PWM_valor;
        }else{
          Serial.println("S U B I E N D O");
          PWM_valor=220+PWM_valor;
        }
        Ajuste_motor(PWM_valor);
        delay(90);
        digitalWrite (IN3, HIGH);
        digitalWrite (IN4, LOW);
        analogWrite(ENB,motor_estable);
        delay(200);
        if((int)error==0){
          Serial.println("AJUSTADO");
          digitalWrite (IN3, HIGH);
          digitalWrite (IN4, LOW);
          analogWrite(ENB,motor_estable);
          break;
        }
        Serial.print("/////////////////////Errror a comparara/////////////////////////////// ");
        Serial.println(error,4);
      }
    break;
  }
  Serial.println("******* SALIENDO *******");
}
void Ajuste_motor(int pwd){
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  
  Serial.print("$$$$$$$$$$ SE MANDA AL MOTOR = ");
  Serial.println(pwd);
  analogWrite(ENB,pwd);
  delay(100);
}

/*
Conexiones:
          Encoder
Cable amarillo (A)  -> Pin 2
Cable verde    (B)  -> Pin 3

Cable negro          ->GND
Cable azul           ->5V

          Puente H
In1                  ->Pin 6
In2                  ->Pin 5
GND                  ->GND
5V                   ->5V

Recuerda conectar la tierra del arduino
*/


int dt_us = 2000;             // muestreo en micro-segundos
float dt = dt_us*0.000001;    // muestro en segudos
unsigned long t1 = 0, t2 = 0; // Tiempos para intervalos
unsigned long k = 0;          // Contador de muestras

int Np = 0;                   // Número de pulsos
const float R = 0.5;   // Resolución de salida           ////////////////Valor a modificar
float th = 0, thp =0;         // Posición angular y valor pasado
float dth_d = 0, dth_f =0;    // derivada discreta y s. filtrada
float alpha = 0.05;            // Coeficiente de filtro

//float kp = 0.48222, kd = 0.20961, ki = 0.23562;
//float kp = 1.5, kd = 0.10961, ki = 0.5562; // P
//float kp = 0.23, kd = 0.0470, ki = 0; // PD
 float kp = 0.8, kd = 0.023, ki = 0.1;   // PID
float e = 0, de = 0, inte = 0; 
float u = 0, usat = 0;        // SEÑAL DE CONTROL
float PWM = 0;                  // SEÑAL PWM
float th_des = 0;             // CONSIGAN DEL CONTROL
// REVISAR CAUTELOSAMENTE EL SENTIDO DE GIRO
const int sen1 = 5; //--------------------------PIN5
const int sen2 = 6; //--------------------------PIN6

String consigna;
void setup() {
  // Comunicación Serial
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), CH_A, RISING);//--------------------------PIN2
  attachInterrupt(digitalPinToInterrupt(3), CH_B, RISING);//--------------------------PIN3

  // Salidas de control PWM
  pinMode(sen1,OUTPUT);
  pinMode(sen2,OUTPUT);
}

void loop() {
  // El código se debe escribir entre la toma de los tiempo
  t1 = micros();  //Muestra de tiempo 1
    // ****************************************
    // LEER CONSIGNA DE CONTROL
      if(Serial.available()> 0){
      consigna = Serial.readStringUntil('\n');
      th_des = consigna.toFloat();
      }
      
      th = R*Np;            // Toma de muestra
 
      dth_d = (th-thp)/dt;  // Derivada discreta

      // Filtro de la velocidad 
      dth_f = alpha*dth_d + (1-alpha)*dth_f;
      

      /* Calcular elemetos del control */
      ////////////////////////////////////
      e = th_des - th;             // ERROR 
      de = - dth_f;                // DERIVADA DEL ERROR
      inte = inte + e*dt;          // INTEGRAL DEL ERROR
      // CONTROL PID
      u = kp*e + kd*de + ki*inte;  // VOLTAJE

      usat = constrain(u,-12,12); // SATURACION         ////////////////Valor a modificar
      // PWM 0 a 255
      PWM = usat*21.25; // 255/12V = 21.5       // 255/6V = 42.5               ////////////////Valor a modificar
      
             // MANDAR SEÑAL DE CONTROL COMO PWM
      if(PWM>0){
      analogWrite(sen1,PWM);
      analogWrite(sen2,0);
      }
      if(PWM<0){
      analogWrite(sen1,0);
      analogWrite(sen2,-1*PWM);
      }

      ////////////////////////////////////

      k=k+1;                // Número de muestra
      thp = th;             // Guardar valor de th
      
      // Imprimir los valores 
      Serial.print(th);
      Serial.print(',');
      Serial.println(PWM);
    // ****************************************
  t2 = micros();  //Muestra de tiempo 2
  while((t2-t1)<dt_us){
    t2 = micros();
  }
  //Serial.println(t2-t1);
}

void CH_A(){
  if (digitalRead(3)==HIGH)
  Np=Np+1;
  if (digitalRead(3)==LOW)
  Np=Np-1;
}

void CH_B(){
  if (digitalRead(2)==LOW)
  Np=Np+1;
  if (digitalRead(2)==HIGH)
  Np=Np-1;
}

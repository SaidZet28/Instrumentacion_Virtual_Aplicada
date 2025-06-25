#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pines de botones (conectados a GND, usan INPUT_PULLUP)
const int PIN_ADELANTE = 13;
const int PIN_ATRAS = 12;
const int PIN_REGRESAR = 14;
const int PIN_OK = 27;

// Pines del encoder y motor
const int encoderA = 18;
const int encoderB = 19;
const int in1 = 2;
const int in2 = 4;

const int pwmFreq = 5000;
const int pwmResolution = 8;

volatile int Np = 0; // Pulsos del encoder
int lastNp = 0;
float th = 0;
float w = 0;
const float R = 1.0 / 732; // Revoluciones/pulso

unsigned long lastTime = 0;

// Parámetros del controlador PI
float kp = 0.5, ki = 0.15;
float e = 0, inte = 0;
float PWM = 0;
float w_des = 0;  // En rev/s
float w_de = 0;  // En rev/m

String consigna;
// HMI
enum Estado {
  MENU_INICIO,
  MENU_PRINCIPAL,
  AJUSTAR_VEL,
  AJUSTAR_KP,
  AJUSTAR_KI,
  VER_VEL              // NUEVO ESTADO
};

Estado estado = MENU_INICIO;
int opcionActual = 0;

const int NUM_OPCIONES = 4;
String opciones[NUM_OPCIONES] = {
  "Ajustar Velocidad",
  "Ajustar Kp",
  "Ajustar Ki",
  "Ver velocidad"
};

///// BASE DE DATOS

#define WIFI_SSID "iPhone de Jimena"
#define WIFI_PASSWORD "HolaBola"
#define FIREBASE_AUTH "AIzaSyCEY1RBR5RWlt9i9w4F74V6nV-DjMblwFU"
#define FIREBASE_HOST "https://final-iva-default-rtdb.firebaseio.com"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

bool firebaseReady = false;

// === Firebase ===
void actualizarFirebase() {
  if (!firebaseReady) {
    Serial.println("Firebase no está listo para escribir.");
    return;
  }

  if (Firebase.setFloat(fbdo, "/jets/RPM", w_des*60)) {
    Serial.print("RPM guardado: "); Serial.println(w_des*60);
  } else {
    Serial.print("Error al guardar RPM: "); Serial.println(fbdo.errorReason());
  }

  if (Firebase.setFloat(fbdo, "/jets/KP", kp)) {
    Serial.print("KP guardado: "); Serial.println(kp);
  } else {
    Serial.print("Error al guardar KP: "); Serial.println(fbdo.errorReason());
  }

  if (Firebase.setFloat(fbdo, "/jets/KI", ki)) {
    Serial.print("KI guardado: "); Serial.println(ki);
  } else {
    Serial.print("Error al guardar KI: "); Serial.println(fbdo.errorReason());
  }
}

void leerFirebase() {
  if (!firebaseReady) {
    Serial.println("Firebase no está listo para leer.");
    return;
  }

  if (Firebase.getFloat(fbdo, "/jets/RPM")) {
    w_des = fbdo.floatData()/60;
    Serial.print("RPM leído: "); Serial.println(w_des*60);
  } else {
    Serial.print("Error al leer RPM: "); Serial.println(fbdo.errorReason());
  }

  if (Firebase.getFloat(fbdo, "/jets/KP")) {
    kp = fbdo.floatData();
    Serial.print("KP leído: "); Serial.println(kp);
  } else {
    Serial.print("Error al leer KP: "); Serial.println(fbdo.errorReason());
  }

  if (Firebase.getFloat(fbdo, "/jets/KI")) {
    ki = fbdo.floatData();
    Serial.print("KI leído: "); Serial.println(ki);
  } else {
    Serial.print("Error al leer KI: "); Serial.println(fbdo.errorReason());
  }
}



// ===================== FUNCIONES LCD ===========================
void mostrarInicio() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Vel: ");
  lcd.print(w_des * 60, 1);  // Mostrar en RPM
  lcd.print(" RPM");

  lcd.setCursor(0, 1);
  lcd.print("Presiona OK...");
}

void mostrarMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">");
  lcd.print(opciones[opcionActual]);

  int siguiente = (opcionActual + 1) % NUM_OPCIONES;
  lcd.setCursor(0, 1);
  lcd.print(" ");
  lcd.print(opciones[siguiente]);
}

void mostrarAjuste(String etiqueta, float valor, String unidad) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(etiqueta);
  lcd.setCursor(0, 1);
  lcd.print("Valor: ");
  lcd.print(valor, 2);
  lcd.print(" ");
  lcd.print(unidad);
}

// ===================== INTERRUPCIONES ===========================
void IRAM_ATTR CH_A() {
  if (digitalRead(encoderB) == HIGH)
    Np++;
  else
    Np--;
}

void IRAM_ATTR CH_B() {
  if (digitalRead(encoderA) == LOW)
    Np++;
  else
    Np--;
}

void mostrarVelocidad() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Vel: ");
  lcd.print(w * 60, 1);  

  lcd.setCursor(0, 1);
  lcd.print("Error: ");
  lcd.print(((w_des-w)*60.0f), 2);
  
}


// ===================== SETUP ===========================
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  // Encoder
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), CH_A, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), CH_B, RISING);

  // Motor
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // PWM
  ledcAttach(in1, pwmFreq, pwmResolution);
  ledcAttach(in2, pwmFreq, pwmResolution);
  // Botones
  pinMode(PIN_ADELANTE, INPUT_PULLUP);
  pinMode(PIN_ATRAS, INPUT_PULLUP);
  pinMode(PIN_REGRESAR, INPUT_PULLUP);
  pinMode(PIN_OK, INPUT_PULLUP);

  mostrarInicio();

  //Base de datos
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi conectado");

  // Configuración Firebase
  config.api_key = FIREBASE_AUTH;
  config.database_url = FIREBASE_HOST;

  Firebase.reconnectWiFi(true);

  Firebase.begin(&config, &auth);
  
  // Autenticación anónima
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Autenticación anónima exitosa");
    firebaseReady = true;
  } else {
    Serial.print("Error de autenticación: ");
    Serial.println(config.signer.signupError.message.c_str());
  }

  // Solo si todo salió bien
  if (firebaseReady) {
    leerFirebase();
  }
}

// ===================== LOOP ===========================
void loop() {
  // ========== Control PI ==========
   if (Serial.available() > 0) {
    consigna = Serial.readStringUntil('\n');
    consigna.trim();  // Eliminar espacios y saltos de línea

    int coma1 = consigna.indexOf(',');
    int coma2 = consigna.indexOf(',', coma1 + 1);
  
    if (coma1 == -1) {
      // Solo contiene el setpoint
      w_des = (consigna.toFloat())/60;
    } else {
      // Extraer setpoint, kp, kd, ki
      w_des = (consigna.substring(0, coma1).toFloat())/60;
      kp = consigna.substring(coma1 + 1, coma2).toFloat();
      ki = consigna.substring(coma2 + 1).toFloat();
    }
  }
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6;
  if (dt >= 0.01) { // cada 10ms
    lastTime = now;

    th = R * Np;
    w = (Np - lastNp) * R / dt;  // rev/s
    lastNp = Np;

    e = (w_des * 400) - w;
    inte += e * dt;
    inte = constrain(inte, -5, 5);
    PWM = kp * e + ki * inte;
    PWM = constrain(PWM, -255, 255);

    if (PWM > 0) {
      ledcWrite(in1, abs(PWM));
      ledcWrite(in2, 0);
    } else {
      ledcWrite(in1, 0);
      ledcWrite(in2, abs(PWM));
    }

  }

  // ========== Interfaz HMI ==========
  bool adelante = digitalRead(PIN_ADELANTE) == LOW;
  bool atras = digitalRead(PIN_ATRAS) == LOW;
  bool regresar = digitalRead(PIN_REGRESAR) == LOW;
  bool ok = digitalRead(PIN_OK) == LOW;

  delay(150);  // Debounce

  switch (estado) {
    case MENU_INICIO:
    mostrarInicio();  // Actualiza la pantalla 
     if (ok) {
     estado = MENU_PRINCIPAL;
     mostrarMenu();
  }
  break;


    case MENU_PRINCIPAL:
      if (adelante) {
        opcionActual = (opcionActual + 1) % NUM_OPCIONES;
        mostrarMenu();
      } else if (atras) {
        opcionActual = (opcionActual - 1 + NUM_OPCIONES) % NUM_OPCIONES;
        mostrarMenu();
      } else if (ok) {
        if (opcionActual == 0) estado = AJUSTAR_VEL;
        else if (opcionActual == 1) estado = AJUSTAR_KP;
        else if (opcionActual == 2) estado = AJUSTAR_KI;
        else if (opcionActual == 3) {
      estado = VER_VEL;
      mostrarVelocidad();
}
      }

      

      break;

    case AJUSTAR_VEL:
      if (adelante) w_des += 1.0 / 60.0;  // Aumenta 1 RPM
      else if (atras) w_des = max(0.0f, w_des - 1.0f / 60.0f);
      else if (regresar) {
        actualizarFirebase();  // después de cambiar kp, ki o w_des
        estado = MENU_PRINCIPAL;
        mostrarMenu();
        return;
      }
      mostrarAjuste("Ajustar Velocidad", w_des * 60, "RPM");
      break;

    case AJUSTAR_KP:
      if (adelante) kp += 0.1;
      else if (atras) kp = max(0.0f, kp - 0.1f);
      else if (regresar) {
        actualizarFirebase();  // después de cambiar kp, ki o w_des
        estado = MENU_PRINCIPAL;
        mostrarMenu();
        return;
      }
      mostrarAjuste("Ajustar Kp", kp, "");
      break;

    case AJUSTAR_KI:
      if (adelante) ki += 0.1;
      else if (atras) ki = max(0.0f, ki - 0.1f);
      else if (regresar) {
        actualizarFirebase();  // después de cambiar kp, ki o w_des
        estado = MENU_PRINCIPAL;
        mostrarMenu();
        return;
      }
      mostrarAjuste("Ajustar Ki", ki, "");
      break;

    case VER_VEL:
     if (regresar) {
      estado = MENU_PRINCIPAL;
    mostrarMenu();
    return;
     }
    mostrarVelocidad();
    break;

  }

  // Leer firebase solo cuando NO está en los modos de ajuste
static unsigned long lastFirebaseRead = 0;
if (millis() - lastFirebaseRead > 3000) {
  if (estado != AJUSTAR_VEL && estado != AJUSTAR_KP && estado != AJUSTAR_KI) {
    leerFirebase();
  }
  lastFirebaseRead = millis();
}


}

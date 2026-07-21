/*
 * =====================================================================================
 *  OTTO NINJA X7 - Firmware de controle (ESP32 + PCA9685)
 * =====================================================================================
 *  Robô com 7 servos motores controlados via placa PCA9685 (I2C):
 *    Canal 0 -> Servo de 180º da CABEÇA (pan)
 *    Canal 1 -> Servo do BRAÇO DIREITO
 *    Canal 2 -> Servo do BRAÇO ESQUERDO
 *    Canal 3 -> Servo da PERNA DIREITA   (inclinação / transformação pé <-> roda)
 *    Canal 4 -> Servo da PERNA ESQUERDA  (inclinação / transformação pé <-> roda)
 *    Canal 5 -> Servo do PÉ DIREITO      (passo ao andar / giro da roda)
 *    Canal 6 -> Servo do PÉ ESQUERDO     (passo ao andar / giro da roda)
 *
 *  Outros dispositivos:
 *    - ESP32 (controlador principal, Wi-Fi + servidor HTTP)
 *    - PCA9685 (driver PWM de 16 canais, comunicação I2C - SDA/SCL)
 *    - Buzzer (aviso sonoro de inicialização / eventos)
 *    - Sensor ultrassônico HC-SR04 (modo de desvio de obstáculo - "Sentinela")
 *    - LED interno (indica status de conexão Wi-Fi)
 *    - Regulador de tensão (alimentação estável dos servos - somente hardware,
 *      sem necessidade de código; garanta que ele suporte a corrente dos 7 servos)
 *
 *  Biblioteca necessária (Library Manager da Arduino IDE):
 *    "Adafruit PWM Servo Driver Library" (Adafruit_PWMServoDriver.h)
 * =====================================================================================
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ======================= REDE (Wi-Fi) =======================
//const char* ssid = "POCO X4 GT";         // escreva o nome da sua rede Wi-Fi
//const char* password = "jonny011"; // escreva a senha da sua rede Wi-Fi

const char* ssid = "POCO X4 GT";         // escreva o nome da sua rede Wi-Fi
const char* password = "jonny011"; // escreva a senha da sua rede Wi-Fi

// IP AUTOMÁTICO (DHCP): removemos a configuração de IP fixo (WiFi.config) que
// forçava 10.0.0.208 / gateway 192.168.1.1 - faixa incompatível com a rede real
// (10.221.50.0/24). Agora o roteador atribui o IP automaticamente e ele aparece
// no Serial Monitor (115200 baud) logo após "WiFi conectado".

WiFiServer server(80);

// ======================= HARDWARE =======================
// Driver PCA9685 no endereço I2C padrão 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Pinos do ESP32 usados pelo I2C do PCA9685 (padrão da maioria das placas ESP32)
#define I2C_SDA 21
#define I2C_SCL 22

// Sensor ultrassônico HC-SR04
#define TRIG_PIN 18
#define ECHO_PIN 19

// Buzzer
#define BUZZER_PIN 4

// LED interno (indicação de status)
#define LED_PIN 2

// Canais dos 7 servos no PCA9685
#define CH_HEAD   0   // cabeça (0-180º)
#define CH_ARM_R  1   // braço direito
#define CH_ARM_L  2   // braço esquerdo
#define CH_LEG_R  3   // perna direita (inclinação / transformação)
#define CH_LEG_L  4   // perna esquerda (inclinação / transformação)
#define CH_FOOT_R 5   // pé direito (passo / roda)
#define CH_FOOT_L 6   // pé esquerdo (passo / roda)

// Pulsos mínimo e máximo em "ticks" (0-4095) para 50Hz.
// Calibre esses valores conforme o modelo exato do seu servo, se necessário.
#define SERVO_MIN 102   // ~0.5 ms
#define SERVO_MAX 512   // ~2.5 ms

// Converte um ângulo (0-180º) em pulso do PCA9685 e envia ao canal
void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}

// "Desliga" o pulso de um canal (equivalente ao antigo servo.detach())
void releaseServo(uint8_t channel) {
  pwm.setPWM(channel, 0, 0);
}

// ======================= ESTADO GERAL =======================
bool walkMode = false;
bool rollMode = false;

int roll_right_forward_speed = 40;
int roll_left_forward_speed = 40;
int roll_right_backward_speed = 40;
int roll_left_backward_speed = 40;

String command = "";

// ------- Braços (controlados pelos botões A / A1 / B / B1) -------
int rightArmAngle = 90;
int leftArmAngle = 90;
const int ARM_STEP = 8;     // graus que o braço se move a cada comando recebido
const int ARM_MIN = 0;
const int ARM_MAX = 180;

// ------- Cabeça -------
int headAngle = 90;

// =====================================================================================
//  SENSOR ULTRASSÔNICO / MODO SENTINELA (desvio de obstáculo)
// =====================================================================================
long ultrasound_distance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout de 30ms evita travar o loop
  long distance = duration / 58;
  return distance;
}

void AvoidanceWalk() {
  if (ultrasound_distance() <= 15) {
    Home();
    delay(50);
    WalkRight();
  }
  WalkForward();
}

void AvoidanceRoll() {
  if (ultrasound_distance() <= 15) {
    NinjaRollStop();
    NinjaRollRight(roll_right_forward_speed, roll_left_forward_speed);
  }
  NinjaRollForward(roll_right_forward_speed, roll_left_forward_speed);
}

// =====================================================================================
//  POSIÇÕES BASE / TRANSFORMAÇÃO PÉ <-> RODA
// =====================================================================================
void Home() {
  //setServoAngle(CH_FOOT_L, 90);
  //setServoAngle(CH_FOOT_R, 90);
  setServoAngle(CH_LEG_L, 60);
  setServoAngle(CH_LEG_R, 120);
  setServoAngle(CH_HEAD, 90);
  headAngle = 90;
  setServoAngle(CH_ARM_R, rightArmAngle);
  setServoAngle(CH_ARM_L, leftArmAngle);
  delay(150);
}

void SetWalk() {
  walkMode = true;
  rollMode = false;
  setServoAngle(CH_LEG_L, 60);
  setServoAngle(CH_LEG_R, 120);
  delay(150);
  releaseServo(CH_LEG_L);
  releaseServo(CH_LEG_R);
  delay(100);
}

void SetRoll() {
  walkMode = false;
  rollMode = true;
  setServoAngle(CH_LEG_L, 170);
  setServoAngle(CH_LEG_R, 10);
  delay(150);
  releaseServo(CH_LEG_L);
  releaseServo(CH_LEG_R);
  delay(100);
}

void TiltToRight() {
  setServoAngle(CH_LEG_L, 0);
  setServoAngle(CH_LEG_R, 65);
  delay(300);
  releaseServo(CH_LEG_L);
  releaseServo(CH_LEG_R);
  delay(300);
}

void TiltToLeft() {
  setServoAngle(CH_LEG_L, 120);
  setServoAngle(CH_LEG_R, 180);
  delay(300);
  releaseServo(CH_LEG_L);
  releaseServo(CH_LEG_R);
  delay(300);
}

void MoveRightFoot(int s, int t) {
  setServoAngle(CH_FOOT_R, s);
  delay(t);
  setServoAngle(CH_FOOT_R, 90);
  delay(100);
  releaseServo(CH_FOOT_R);
  delay(300);
}

void MoveLeftFoot(int s, int t) {
  setServoAngle(CH_FOOT_L, s);
  delay(t);
  setServoAngle(CH_FOOT_L, 90);
  delay(100);
  releaseServo(CH_FOOT_L);
  delay(300);
}

void ReturnFromRight() {
  setServoAngle(CH_LEG_L, 60);
  for (int count = 65; count <= 120; count += 2) {
    setServoAngle(CH_LEG_R, count);
    delay(25);
  }
  delay(300);
  releaseServo(CH_LEG_L);
  releaseServo(CH_LEG_R);
}

void ReturnFromLeft() {
  setServoAngle(CH_LEG_R, 120);
  for (int count = 120; count >= 60; count -= 2) {
    setServoAngle(CH_LEG_L, count);
    delay(25);
  }
  delay(300);
  releaseServo(CH_LEG_L);
  releaseServo(CH_LEG_R);
}

// =====================================================================================
//  MODO BÍPEDE (andar)
// =====================================================================================
void WalkForward() {
  TiltToRight();
  delay(100);
  MoveRightFoot(70, 250);
  delay(100);
  ReturnFromRight();

  TiltToLeft();
  delay(100);
  MoveLeftFoot(140, 350);
  delay(100);
  ReturnFromLeft();
}

void WalkRight() {
  TiltToRight();
  delay(100);
  MoveRightFoot(70, 350);
  delay(100);
  ReturnFromRight();
}

void WalkBackward() {
  TiltToRight();
  delay(100);
  MoveRightFoot(120, 250);
  delay(100);
  ReturnFromRight();

  TiltToLeft();
  delay(100);
  MoveLeftFoot(60, 350);
  delay(100);
  ReturnFromLeft();
}

void WalkLeft() {
  TiltToLeft();
  delay(100);
  MoveLeftFoot(140, 350);
  delay(100);
  ReturnFromLeft();
}

// =====================================================================================
//  MODO RODA
// =====================================================================================
void NinjaRollForward(int speedR, int speedL) {
  setServoAngle(CH_FOOT_L, 90 + speedL);
  setServoAngle(CH_FOOT_R, 90 + speedR);
}

void NinjaRollRight(int speedR, int speedL) {
  setServoAngle(CH_FOOT_L, 90 + speedL);
  setServoAngle(CH_FOOT_R, 90 + speedR);
}

void NinjaRollLeft(int speedR, int speedL) {
  setServoAngle(CH_FOOT_L, 90 - speedL);
  setServoAngle(CH_FOOT_R, 90 - speedR);
}

void NinjaRollBackward(int speedR, int speedL) {
  setServoAngle(CH_FOOT_L, 90 + speedL);
  setServoAngle(CH_FOOT_R, 90 + speedR);
}

void NinjaRollStop() {
  setServoAngle(CH_FOOT_L, 90);
  setServoAngle(CH_FOOT_R, 90);
  releaseServo(CH_FOOT_L);
  releaseServo(CH_FOOT_R);
}

// =====================================================================================
//  BRAÇOS (botões A / A1 / B / B1)
//   A  -> sobe braço direito     A1 -> desce braço direito
//   B  -> sobe braço esquerdo    B1 -> desce braço esquerdo
// =====================================================================================
void ArmRightUp() {
  rightArmAngle = constrain(rightArmAngle + ARM_STEP, ARM_MIN, ARM_MAX);
  setServoAngle(CH_ARM_R, rightArmAngle);
}

void ArmRightDown() {
  rightArmAngle = constrain(rightArmAngle - ARM_STEP, ARM_MIN, ARM_MAX);
  setServoAngle(CH_ARM_R, rightArmAngle);
}

void ArmLeftUp() {
  leftArmAngle = constrain(leftArmAngle + ARM_STEP, ARM_MIN, ARM_MAX);
  setServoAngle(CH_ARM_L, leftArmAngle);
}

void ArmLeftDown() {
  leftArmAngle = constrain(leftArmAngle - ARM_STEP, ARM_MIN, ARM_MAX);
  setServoAngle(CH_ARM_L, leftArmAngle);
}

// =====================================================================================
//  JOYSTICK (usado tanto no modo bípede quanto no modo roda)
// =====================================================================================
void joystickWalk(int x, int y) {
  if ((x >= -5) && (x <= 5) && (y >= -5) && (y <= 5)) { command = "home"; }
  else if (y < -25 && x < -25 || y > 25 && x < -25) { command = "left"; }
  else if (y < -25 && x > 25 || y > 25 && x > 25) { command = "right"; }
  else if (y < -25) { command = "forward"; }
  else if (y > 25) { command = "backward"; }
}

void joystickRoll(int x, int y) {
  if ((x >= -5) && (x <= 5) && (y >= -5) && (y <= 5)) {
    NinjaRollStop();
  } else {
    int LWS = map(y, -50, 50, 135, 45);
    int RWS = map(y, -50, 50, 45, 135);
    int LWD = map(x, 50, -50, 45, 0);
    int RWD = map(x, 50, -50, 0, -45);
    setServoAngle(CH_FOOT_L, LWS + LWD);
    setServoAngle(CH_FOOT_R, RWS + RWD);
  }
}

// =====================================================================================
//  CONFIGURAÇÕES DE VELOCIDADE (modo roda)
// =====================================================================================
void Settings(String speeds) {
  decodeSpeeds(speeds);
}

void decodeSpeeds(String c) {
  int counter = 0;
  String RF = "";
  String LF = "";
  String RB = "";
  String LB = "";

  for (int i = 1; i < c.length(); i++) {
    if (isDigit(c[i])) {
      if (counter == 0) { RF += c[i]; }
      else if (counter == 1) { LF += c[i]; }
      else if (counter == 2) { RB += c[i]; }
      else if (counter == 3) { LB += c[i]; }
    } else if (c[i] == '&') {
      counter++;
    }
  }

  roll_right_forward_speed = RF.toInt();
  roll_left_forward_speed = LF.toInt();
  roll_right_backward_speed = RB.toInt();
  roll_left_backward_speed = LB.toInt();

  Serial.println("");
  Serial.println(c);
  Serial.println(RF.toInt());
  Serial.println(LF.toInt());
  Serial.println(RB.toInt());
  Serial.println(LB.toInt());
}

// =====================================================================================
//  SETUP / LOOP
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(10);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Inicializa o barramento I2C e o driver PCA9685
  Wire.begin(I2C_SDA, I2C_SCL);
  pwm.begin();
  pwm.setPWMFreq(50); // servos padrão trabalham em 50Hz

  Serial.println();
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // IP automático via DHCP (sem WiFi.config)

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  digitalWrite(LED_PIN, HIGH); // LED interno aceso = conectado

  server.begin();
  Serial.println("Servidor iniciado");
  Serial.print("IP do robo: ");
  Serial.println(WiFi.localIP()); // <-- use este IP no campo "IP Address" do web app

  tone(BUZZER_PIN, 440, 1000); // beep de inicialização
  Home();
}

void loop() {
  CheckClient(); // verifica se chegou algum comando novo

  if (command == "walkmode") {
    SetWalk();
    command = "";
  }
  else if (command == "rollmode") {
    SetRoll();
    command = "";
  }
  else if (command == "home") {
    Home();
    command = "";
  }
  else if (command == "forward") {
    if (walkMode) {
      WalkForward();
      command = "";
    } else {
      NinjaRollForward(roll_right_forward_speed, roll_left_forward_speed);
    }
  }
  else if (command == "backward") {
    if (walkMode) {
      WalkBackward();
      command = "";
    } else {
      NinjaRollBackward(roll_right_backward_speed, roll_left_backward_speed);
    }
  }
  else if (command == "right") {
    if (walkMode) {
      WalkRight();
      command = "";
    } else {
      NinjaRollRight(roll_right_forward_speed, roll_left_forward_speed);
      command = "";
    }
  }
  else if (command == "left") {
    if (walkMode) {
      WalkLeft();
      command = "";
    } else {
      NinjaRollLeft(roll_right_backward_speed, roll_left_backward_speed);
      command = "";
    }
  }
  else if (command == "stop") {
    NinjaRollStop();
  }
  else if (command == "avoidancewalk") {
    AvoidanceWalk();
  }
  else if (command == "avoidanceroll") {
    AvoidanceRoll();
  }
  else if (command == "armRightUp") {
    ArmRightUp();
    command = "";
  }
  else if (command == "armRightDown") {
    ArmRightDown();
    command = "";
  }
  else if (command == "armLeftUp") {
    ArmLeftUp();
    command = "";
  }
  else if (command == "armLeftDown") {
    ArmLeftDown();
    command = "";
  }
}

void CheckClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  Serial.println("novo cliente");
  while (!client.available()) {
    delay(1);
  }

  String req = client.readStringUntil('\r');
  Serial.println(req);
  client.flush();

  if (req.indexOf("/walkmode") != -1) {
    command = "walkmode";
  }
  else if (req.indexOf("/rollmode") != -1) {
    command = "rollmode";
  }
  else if (req.indexOf("/home") != -1) {
    command = "home";
  }
  else if (req.indexOf("/armRightUp") != -1) {
    command = "armRightUp";
  }
  else if (req.indexOf("/armRightDown") != -1) {
    command = "armRightDown";
  }
  else if (req.indexOf("/armLeftUp") != -1) {
    command = "armLeftUp";
  }
  else if (req.indexOf("/armLeftDown") != -1) {
    command = "armLeftDown";
  }
  else if (req.indexOf("/forward") != -1) {
    command = "forward";
  }
  else if (req.indexOf("/right") != -1) {
    command = "right";
  }
  else if (req.indexOf("/backward") != -1) {
    command = "backward";
  }
  else if (req.indexOf("/left") != -1) {
    command = "left";
  }
  else if (req.indexOf("/stop") != -1) {
    command = "stop";
  }
  else if (req.indexOf("/avoidancewalk") != -1) {
    command = "avoidancewalk";
  }
  else if (req.indexOf("/avoidanceroll") != -1) {
    command = "avoidanceroll";
  }
  else if (req.indexOf("/R=") > 0) {
    Settings(req);
    Home();
    command = "";
  }
  else if (req.indexOf("/J") > 0) {
    command = "joystick";
    GetCoords(req);
  }
  else {
    Serial.println("petición inválida");
    client.stop();
    return;
  }

  client.flush();

  String s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\nGPIO is now ";
  s += "</html>\n";

  client.print(s);
  delay(1);
  Serial.println("Cliente desconectado");
}

void GetCoords(String str) {
  String x = str.substring(str.lastIndexOf('J') + 1, str.lastIndexOf(','));
  String y = str.substring(str.lastIndexOf(',') + 1, str.lastIndexOf('H') - 1);
  walkMode == true ? joystickWalk(x.toInt(), y.toInt()) : joystickRoll(x.toInt(), y.toInt());
}

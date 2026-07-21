/*
 * =====================================================================================
 *  OTTO NINJA X7 - Firmware de controle (ESP32 + PCA9685 + OLED)
 * =====================================================================================
 *  Robô com 7 servos motores controlados via placa PCA9685 (I2C):
 *    Canal 0 -> Servo de 180º da CABEÇA (pan)
 *    Canal 1 -> Servo do BRAÇO DIREITO
 *    Canal 2 -> Servo do BRAÇO ESQUERDO
 *    Canal 3 -> Servo da PERNA DIREITA   (inclinação / transformação pé <-> roda)
 *    Canal 4 -> Servo da PERNA ESQUERDA  (inclinação / transformação pé <-> roda)
 *    Canal 5 -> Servo do PÉ DIREITO      (passo ao andar / giro da roda - 360º)
 *    Canal 6 -> Servo do PÉ ESQUERDO     (passo ao andar / giro da roda - 360º)
 *
 *  Outros dispositivos:
 *    - ESP32 (controlador principal, Wi-Fi + servidor HTTP)
 *    - PCA9685 (driver PWM de 16 canais, comunicação I2C - SDA/SCL, endereço 0x40)
 *      SDA -> GPIO21, SCL -> GPIO22, VCC lógica -> 3.3V ou 5V, GND -> GND (comum com ESP32)
 *      V+ (terminal de parafuso, alimentação dos servos) -> fonte externa dedicada de
 *      5-6V com pelo menos 2-3A (7 servos partindo juntos puxam bastante corrente de pico;
 *      um capacitor eletrolítico grande, ex: 1000uF, perto do V+ ajuda a segurar picos).
 *    - Display OLED SSD1306 128x64 (I2C, endereço 0x3C - mesmo barramento da PCA9685,
 *      sem conflito): SDA -> GPIO21, SCL -> GPIO22, VCC -> 3.3V, GND -> GND
 *    - Buzzer: pino positivo -> GPIO4, pino negativo -> GND
 *    - Sensor ultrassônico HC-SR04 (modo de desvio de obstáculo - "Sentinela")
 *      VCC -> 5V, GND -> GND, TRIG -> GPIO18, ECHO -> GPIO19
 *      ATENÇÃO: ECHO do HC-SR04 sai em 5V; use um divisor de tensão (ex: 1k + 2k)
 *      entre o ECHO e o GPIO19 pra não estourar a entrada de 3.3V do ESP32.
 *    - Interruptor tátil de desligar: GPIO27 -> uma perna, outra perna -> GND
 *    - LED interno (indica status de conexão Wi-Fi)
 *    - Regulador de tensão (alimentação estável dos servos - somente hardware,
 *      sem necessidade de código; garanta que ele suporte a corrente dos 7 servos)
 *
 *  Bibliotecas necessárias (Library Manager da Arduino IDE):
 *    "Adafruit PWM Servo Driver Library" (Adafruit_PWMServoDriver.h)
 *    "Adafruit GFX Library"              (Adafruit_GFX.h)
 *    "Adafruit SSD1306"                  (Adafruit_SSD1306.h)
 *
 *  Modos disponíveis (via web app, ver ninjaweb_x7.html):
 *    - Walk / Roll ......... andar (bípede) ou rodar (rodas 360°)
 *    - Sentinela ........... patrulha em um quadrado de ~1m desviando de
 *                            obstáculos (sensor ultrassônico); ao detectar um
 *                            obstáculo, mostra uma "cara de morto" no OLED
 *    - Sumo ................ sobe/desce os braços continuamente, com olhos de
 *                            raiva no OLED
 *    - Berserker ........... traça uma estrela de 6 pontas contínua no modo
 *                            roda (aproximação por tempo - calibrar)
 *    - Happy Birthday ...... toca "Parabéns pra Você" no buzzer com uma
 *                            pequena dancinha (performance única)
 *  Os tempos/contagens de passos dos modos Sentinela e Berserker são
 *  aproximações (SENTINELA_STRIDES_PER_SIDE, SENTINELA_TURN_STEPS,
 *  BERSERKER_FORWARD_MS, BERSERKER_TURN_MS) - calibre-os no robô real.
 * =====================================================================================
 */
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// ======================= REDE (Wi-Fi) =======================
const char* ssid = "POCO X4 GT";         // escreva o nome da sua rede Wi-Fi
const char* password = "jonny011"; // escreva a senha da sua rede Wi-Fi
// CORREÇÃO: removido o IP fixo (10.0.0.208 com gateway 192.168.1.1 - faixas
// incompatíveis, o que deixava o robô "conectado" no Wi-Fi mas inalcançável
// pela rede). Agora o roteador/hotspot atribui o IP automaticamente (DHCP).
// Depois de gravar, abra o Serial Monitor (115200 baud) pra ver qual IP o
// robô recebeu, e use esse IP (com barra no final) no site.
WiFiServer server(80);
// ======================= HARDWARE =======================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define I2C_SDA 21
#define I2C_SCL 22
#define TRIG_PIN 18
#define ECHO_PIN 19
#define BUZZER_PIN 4
#define LED_PIN 2
#define SHUTDOWN_PIN 27 // interruptor tátil: uma perna aqui, outra no GND (usa pull-up interno)

// ======================= OLED (SSD1306 128x64, I2C) =======================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C   // não usado diretamente mais - o setup() agora tenta 0x3C e 0x3D sozinho
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledOk = false; // fica false se a tela não for encontrada, pra não travar tentando escrever nela

#define CH_HEAD   0
#define CH_ARM_R  1
#define CH_ARM_L  2
#define CH_LEG_R  3
#define CH_LEG_L  4
#define CH_FOOT_R 5
#define CH_FOOT_L 6
#define SERVO_MIN_FOOT 90
#define SERVO_MIN 120
#define SERVO_MAX_FOOT 550
#define SERVO_MAX 520
void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}
void setServoAngleFoot(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVO_MIN_FOOT, SERVO_MAX_FOOT);
  pwm.setPWM(channel, 0, pulse);
}
void releaseServo(uint8_t channel) {
  pwm.setPWM(channel, 0, 0);
}
bool walkMode = false;
bool rollMode = false;
// NOVO: acompanha o estado da conexão Wi-Fi pra tocar sons e reagir a quedas de conexão
bool wasConnected = false;
unsigned long lastWifiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL_MS = 1000;

// NOVO: controla a taxa de atualização do OLED (não precisa redesenhar toda hora)
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 500;

int roll_right_forward_speed = 40;
int roll_left_forward_speed = 40;
int roll_right_backward_speed = 40;
int roll_left_backward_speed = 40;
String command = "";
unsigned long lastRollCommandTime = 0;
bool rollMoving = false;
const unsigned long ROLL_WATCHDOG_MS = 1000; // folga maior contra engasgos de rede (era 500ms, curto demais)
float speedFactor = 1.0;
int rightArmAngle = 90;
int leftArmAngle = 90;
const int ARM_STEP = 8;
const int ARM_MIN = 0;
const int ARM_MAX = 180;
int headAngle = 90;
const int HEAD_MIN = 0;
const int HEAD_MAX = 180;

// =====================================================================================
//  NOVOS MODOS: Sentinela (patrulha em quadrado), Sumo, Berserker
// =====================================================================================

// ---- Sentinela: anda em um quadrado de ~1m desviando de obstáculos ----
bool sentinelaActive = false;
int sentinelaSide = 0;           // lado atual do quadrado (0-3)
int sentinelaStride = 0;         // passadas já dadas no lado atual
bool sentinelaTurning = false;   // true enquanto gira os ~90° do canto
int sentinelaTurnCount = 0;
// CALIBRAR: nº de WalkForward() por lado do quadrado (depende do tamanho do passo do seu robô)
const int SENTINELA_STRIDES_PER_SIDE = 4;
// CALIBRAR: nº de WalkRight() que fecham ~90° de giro no canto
const int SENTINELA_TURN_STEPS = 3;
unsigned long sentinelaObstacleUntil = 0; // enquanto millis() < isso, mostra a "cara de morto" no OLED

// ---- Sumo: sobe/desce os braços com olhos de raiva no OLED ----
bool sumoActive = false;
bool sumoArmsUp = false;
unsigned long sumoLastToggle = 0;
const unsigned long SUMO_ARM_INTERVAL_MS = 450; // tempo entre subir/descer os braços

// ---- Berserker: estrela de 6 pontas contínua no modo roda ----
bool berserkerActive = false;
int berserkerPhase = 0;      // 0 = andando um lado da estrela, 1 = girando a "ponta"
int berserkerLap = 0;        // conta quantas "pontas" já foram feitas
bool berserkerTurnRight = true;
unsigned long berserkerPhaseStart = 0;
// CALIBRAR: duração de cada lado/giro da estrela (depende da velocidade real das rodas)
const unsigned long BERSERKER_FORWARD_MS = 500;
const unsigned long BERSERKER_TURN_MS = 380;

// =====================================================================================
//  BOTÃO TÁTIL (SHUTDOWN_PIN) - agora com duas funções:
//    1 clique  -> troca de função, alternando entre os 4 modos especiais
//                 (Sentinela -> Sumo -> Berserker -> Happy Birthday -> Sentinela...)
//    2 cliques rápidos -> desliga o robô (deep sleep, igual antes)
//  A detecção é não-bloqueante: espera um tempinho depois do 1º clique pra ver
//  se vem um 2º antes de decidir o que fazer.
// =====================================================================================
int lastButtonRawState = HIGH;    // HIGH = solto (pull-up interno), LOW = pressionado
int buttonStableState = HIGH;
unsigned long lastButtonRawChangeTime = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 30;

int buttonClickCount = 0;
unsigned long lastButtonClickTime = 0;
const unsigned long DOUBLE_CLICK_WINDOW_MS = 400; // tempo máx. entre os 2 cliques pra contar como duplo

int buttonFunctionIndex = -1; // -1 = nenhum modo escolhido ainda; próximo clique começa no Sentinela (0)

// =====================================================================================
//  OLED - tela de status
// =====================================================================================
// Desenha um "X" simples pra representar um olho na tela (usado na cara de morto)
void drawXEye(int cx, int cy) {
  display.drawLine(cx - 8, cy - 8, cx + 8, cy + 8, SSD1306_WHITE);
  display.drawLine(cx - 8, cy + 8, cx + 8, cy - 8, SSD1306_WHITE);
}

// Cara de "morto" (X X, boca reta) - mostrada no Sentinela ao detectar obstáculo
void ShowDeadFaceScreen() {
  if (!oledOk) return;
  display.clearDisplay();
  drawXEye(36, 26);
  drawXEye(92, 26);
  display.drawLine(40, 50, 88, 50, SSD1306_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("Obstaculo!");
  display.display();
}

// Um olho "de raiva": sobrancelha inclinada + pupila - usado no modo Sumo.
// blink=true desenha o olho fechado (risquinho), pra dar uma piscada de impacto.
void drawAngryEye(int cx, int cy, bool mirror, bool blink) {
  if (blink) {
    display.drawLine(cx - 9, cy, cx + 9, cy, SSD1306_WHITE);
    return;
  }
  display.fillCircle(cx, cy, 9, SSD1306_WHITE);
  display.fillCircle(cx, cy, 3, SSD1306_BLACK);
  // sobrancelha franzida, virada pra dentro (efeito de raiva)
  if (mirror) {
    display.drawLine(cx - 12, cy - 16, cx + 8, cy - 8, SSD1306_WHITE);
  } else {
    display.drawLine(cx + 12, cy - 16, cx - 8, cy - 8, SSD1306_WHITE);
  }
}

void ShowAngryEyesScreen(bool blink) {
  if (!oledOk) return;
  display.clearDisplay();
  drawAngryEye(36, 30, false, blink);
  drawAngryEye(92, 30, true, blink);
  display.setCursor(0, 52);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("MODO SUMO");
  display.display();
}

// Tela do Berserker: olhos afiados + texto, só pra dar personalidade ao modo
void ShowBerserkerScreen() {
  if (!oledOk) return;
  display.clearDisplay();
  display.fillTriangle(20, 20, 40, 20, 24, 34, SSD1306_WHITE);
  display.fillTriangle(108, 20, 88, 20, 104, 34, SSD1306_WHITE);
  display.setCursor(0, 50);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("BERSERKER!");
  display.display();
}

// Rosto feliz animado, usado durante o Happy Birthday. blink=true fecha os
// olhos (risquinho) por um instante, simulando uma piscada.
void drawHappyEye(int cx, int cy, bool blink) {
  if (blink) {
    display.drawLine(cx - 8, cy, cx + 8, cy, SSD1306_WHITE);
    return;
  }
  display.fillCircle(cx, cy, 8, SSD1306_WHITE);
  display.fillCircle(cx, cy, 3, SSD1306_BLACK);
}

// Sorriso simples (arco feito com 3 segmentos de linha)
void drawHappyMouth(int cx, int cy, int width) {
  display.drawLine(cx - width / 2, cy, cx - width / 4, cy + 8, SSD1306_WHITE);
  display.drawLine(cx - width / 4, cy + 8, cx + width / 4, cy + 8, SSD1306_WHITE);
  display.drawLine(cx + width / 4, cy + 8, cx + width / 2, cy, SSD1306_WHITE);
}

void ShowHappyFaceScreen(bool blink) {
  if (!oledOk) return;
  display.clearDisplay();
  drawHappyEye(36, 24, blink);
  drawHappyEye(92, 24, blink);
  drawHappyMouth(64, 40, 44);
  display.setCursor(0, 56);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("Feliz Aniversario!");
  display.display();
}

// Tela padrão de status - só é mostrada quando nenhum modo especial está ativo
void ShowStatusScreen() {
  if (!oledOk) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("Otto Ninja X7");
  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);

  display.setCursor(0, 16);
  display.print("Modo: ");
  display.println(walkMode ? "Andar" : "Carro");

  display.setCursor(0, 28);
  display.print("WiFi: ");
  display.println(wasConnected ? "Conectado" : "OFFLINE");

  display.setCursor(0, 40);
  if (wasConnected) {
    display.print(WiFi.localIP());
  } else {
    display.print("Sem IP");
  }

  display.setCursor(0, 52);
  display.print("Vel:");
  display.print((int)(speedFactor * 100));
  display.print("%  Cab:");
  display.print(headAngle);

  display.display();
}

// Escolhe qual tela mostrar de acordo com o modo ativo no momento
void UpdateDisplay() {
  if (!oledOk) return;

  if (sumoActive) {
    ShowAngryEyesScreen(false);
  } else if (berserkerActive) {
    ShowBerserkerScreen();
  } else if (millis() < sentinelaObstacleUntil) {
    ShowDeadFaceScreen();
  } else {
    ShowStatusScreen();
  }
}

void ShowBootScreen() {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println("Otto Ninja X7");
  display.setCursor(0, 35);
  display.println("Iniciando...");
  display.display();
}

void ShowConnectingScreen() {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println("Conectando ao");
  display.setCursor(0, 32);
  display.println(ssid);
  display.setCursor(0, 48);
  display.println("Aguarde...");
  display.display();
}

void ShowShutdownScreen() {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 28);
  display.println("Desligando...");
  display.display();
  delay(300);
  display.ssd1306_command(SSD1306_DISPLAYOFF); // economiza energia durante o deep sleep
}

// =====================================================================================
//  SONS (ligar / conectar / desconectar)
// =====================================================================================
void playBootTone() {
  tone(BUZZER_PIN, 440, 150);  // Lá
  delay(180);
  tone(BUZZER_PIN, 440, 150);  // Lá de novo - "bip-bip" característico de ligar
  delay(180);
}
void playConnectTone() {
  tone(BUZZER_PIN, 523, 120);  // Dó
  delay(140);
  tone(BUZZER_PIN, 659, 120);  // Mi
  delay(140);
  tone(BUZZER_PIN, 784, 200);  // Sol - som "subindo", de sucesso
  delay(220);
}
void playDisconnectTone() {
  tone(BUZZER_PIN, 784, 120);  // Sol
  delay(140);
  tone(BUZZER_PIN, 659, 120);  // Mi
  delay(140);
  tone(BUZZER_PIN, 392, 250);  // Sol grave - som "descendo", de queda de conexão
  delay(270);
}
void playShutdownTone() {
  tone(BUZZER_PIN, 659, 150);  // Mi
  delay(180);
  tone(BUZZER_PIN, 494, 150);  // Si
  delay(180);
  tone(BUZZER_PIN, 330, 150);  // Mi grave
  delay(180);
  tone(BUZZER_PIN, 220, 300);  // Lá grave - som "desligando", mais lento e grave que o de desconexão
  delay(320);
}
// Melodia de "Parabéns pra Você" (melodia tradicional de domínio público) tocada
// no buzzer, nota a nota. Cada linha é {frequência em Hz, duração em ms}.
// 0 Hz = pausa (silêncio).
const int happyBirthdayMelody[][2] = {
  {262, 300}, {262, 150}, {294, 450}, {262, 450}, {349, 450}, {330, 900},
  {0, 100},
  {262, 300}, {262, 150}, {294, 450}, {262, 450}, {392, 450}, {349, 900},
  {0, 100},
  {262, 300}, {262, 150}, {523, 450}, {440, 450}, {349, 450}, {330, 450}, {294, 900},
  {0, 100},
  {466, 300}, {466, 150}, {440, 450}, {349, 450}, {392, 450}, {349, 600}
};
const int happyBirthdayLen = sizeof(happyBirthdayMelody) / sizeof(happyBirthdayMelody[0]);

// Toca a melodia e faz uma pequena "dancinha" de comemoração (cabeça + braços)
// enquanto canta. É uma performance única (bloqueante) - não deve ser chamada em loop.
void PlayHappyBirthday() {
  Serial.println("Cantando Parabens pra Voce!");
  ShowHappyFaceScreen(false);

  bool armToggle = false;
  for (int i = 0; i < happyBirthdayLen; i++) {
    int freq = happyBirthdayMelody[i][0];
    int dur = happyBirthdayMelody[i][1];

    // pisca o olho nas pausas e a cada poucas notas - dá uma sensação de
    // animação simples sem precisar de muito processamento no OLED
    bool blinkNow = (freq == 0) || (i % 3 == 0);
    ShowHappyFaceScreen(blinkNow);

    if (freq > 0) {
      tone(BUZZER_PIN, freq, dur);
    }

    // a cada nota "acentuada" (duração maior), balança a cabeça e alterna os braços
    if (dur >= 400) {
      SetHeadAngle(armToggle ? 60 : 120);
      setServoAngle(CH_ARM_R, armToggle ? ARM_MAX : ARM_MIN);
      setServoAngle(CH_ARM_L, armToggle ? ARM_MIN : ARM_MAX);
      armToggle = !armToggle;
    }

    delay(dur + 30); // pequena folga entre notas
    ShowHappyFaceScreen(false); // reabre o olho antes da próxima nota
  }

  // volta pra posição neutra ao final
  SetHeadAngle(90);
  setServoAngle(CH_ARM_R, rightArmAngle);
  setServoAngle(CH_ARM_L, leftArmAngle);
  UpdateDisplay();
}

// Cancela qualquer modo especial em andamento (Sentinela / Sumo / Berserker) e
// para os motores - chamado sempre que chega um comando manual (andar, parar,
// trocar de modo, etc.) pra não deixar dois comportamentos brigando entre si.
void ExitSpecialModes() {
  sentinelaActive = false;
  sumoActive = false;
  berserkerActive = false;
  sentinelaObstacleUntil = 0;
  NinjaRollStop();
  rollMoving = false;
}

// Avança pro próximo modo do ciclo (Sentinela -> Sumo -> Berserker -> Happy
// Birthday -> Sentinela...) e o ativa. Chamada por um clique único do botão.
void CycleFunction() {
  ExitSpecialModes(); // desliga o que estiver rodando antes de trocar

  buttonFunctionIndex = (buttonFunctionIndex + 1) % 4;

  switch (buttonFunctionIndex) {
    case 0:
      Serial.println("Botao (1 clique): Sentinela");
      command = "sentinela";
      break;
    case 1:
      Serial.println("Botao (1 clique): Sumo");
      command = "sumo";
      break;
    case 2:
      Serial.println("Botao (1 clique): Berserker");
      command = "berserker";
      break;
    case 3:
      Serial.println("Botao (1 clique): Happy Birthday");
      command = "happybirthday";
      buttonFunctionIndex = -1; // é uma performance única - o próximo clique volta a começar pelo Sentinela
      break;
  }
}

// Detecta clique único (1x = troca de função) e clique duplo (2x rápido = desliga)
// no botão tátil, sem travar o resto do loop() esperando.
void CheckButtonClicks() {
  int raw = digitalRead(SHUTDOWN_PIN);

  // debounce: só aceita a mudança de estado depois que ela ficar estável por
  // BUTTON_DEBOUNCE_MS (filtra o "chattering" mecânico do botão)
  if (raw != lastButtonRawState) {
    lastButtonRawChangeTime = millis();
    lastButtonRawState = raw;
  }

  if ((millis() - lastButtonRawChangeTime) > BUTTON_DEBOUNCE_MS && raw != buttonStableState) {
    buttonStableState = raw;
    if (buttonStableState == LOW) { // borda de descida estável = um clique
      buttonClickCount++;
      lastButtonClickTime = millis();
    }
  }

  // Passou a janela de clique duplo sem um novo clique -> decide o que fazer
  if (buttonClickCount > 0 && (millis() - lastButtonClickTime) > DOUBLE_CLICK_WINDOW_MS) {
    if (buttonClickCount >= 2) {
      Shutdown(); // 2 cliques rápidos -> desliga (não retorna, entra em deep sleep)
    } else {
      CycleFunction(); // 1 clique -> troca de função
    }
    buttonClickCount = 0;
  }
}

// Relaxa todos os servos (solta o pulso, mantém a posição atual sem forçar) e
// coloca o ESP32 pra dormir. O mesmo botão (SHUTDOWN_PIN) acorda o robô de novo
// - ao acordar, ele reinicia do zero (roda o setup() inteiro de novo).
void Shutdown() {
  Serial.println("Botao de desligar pressionado - entrando em deep sleep.");
  playShutdownTone();
  ShowShutdownScreen();
  for (int ch = 0; ch < 7; ch++) {
    releaseServo(ch);
  }
  digitalWrite(LED_PIN, LOW);
  // Espera o botão ser solto antes de dormir, senão ele acorda na mesma hora
  while (digitalRead(SHUTDOWN_PIN) == LOW) {
    delay(10);
  }
  delay(200); // debounce
  rtc_gpio_pullup_en((gpio_num_t)SHUTDOWN_PIN);
  rtc_gpio_pulldown_dis((gpio_num_t)SHUTDOWN_PIN);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)SHUTDOWN_PIN, 0); // acorda quando o pino for LOW (botão apertado de novo)
  esp_deep_sleep_start();
}
long ultrasound_distance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
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
//  MODO SENTINELA - patrulha em um quadrado de ~1m desviando de obstáculos.
//  Chamada repetidamente pelo loop() enquanto command == "sentinela": a cada
//  chamada avança um pouco (uma passada ou um passo do giro de 90°), sem travar
//  o resto do sistema por muito tempo.
// =====================================================================================
void SentinelaStep() {
  if (ultrasound_distance() <= 15) {
    // Obstáculo à frente: mostra a "cara de morto" por 1.5s, recua um passo e
    // gira pra desviar. Não conta como passada do lado atual do quadrado.
    sentinelaObstacleUntil = millis() + 1500;
    UpdateDisplay();
    tone(BUZZER_PIN, 200, 200); // beep grave de alerta
    Home();
    delay(50);
    WalkBackward();
    WalkRight();
    return;
  }

  if (sentinelaTurning) {
    WalkRight();
    sentinelaTurnCount++;
    if (sentinelaTurnCount >= SENTINELA_TURN_STEPS) {
      sentinelaTurning = false;
      sentinelaTurnCount = 0;
      sentinelaSide = (sentinelaSide + 1) % 4;
      sentinelaStride = 0;
    }
    return;
  }

  WalkForward();
  sentinelaStride++;
  if (sentinelaStride >= SENTINELA_STRIDES_PER_SIDE) {
    sentinelaTurning = true;
  }
}

// =====================================================================================
//  MODO SUMO - sobe e desce os braços continuamente, com olhos de raiva no OLED.
//  Não bloqueante (baseado em millis()) pra não travar o resto do loop().
// =====================================================================================
void SumoStep() {
  if (millis() - sumoLastToggle >= SUMO_ARM_INTERVAL_MS) {
    sumoLastToggle = millis();
    sumoArmsUp = !sumoArmsUp;
    int angle = sumoArmsUp ? ARM_MAX : ARM_MIN;
    setServoAngle(CH_ARM_R, angle);
    setServoAngle(CH_ARM_L, angle);
    rightArmAngle = angle;
    leftArmAngle = angle;
    if (!sumoArmsUp) {
      tone(BUZZER_PIN, 100, 80); // "baque" grave quando os braços descem com força
      ShowAngryEyesScreen(true); // pisca no impacto, sincronizado com o baque
    } else {
      ShowAngryEyesScreen(false);
    }
  }
}

// =====================================================================================
//  MODO BERSERKER - traça continuamente uma estrela de 6 pontas no modo roda.
//  Aproximação: percorre dois "triângulos" sobrepostos (invertendo o sentido do
//  giro a cada 3 pontas), alternando entre andar em frente e girar ~120° no
//  lugar. Não bloqueante (baseado em millis()).
//  ATENÇÃO: os tempos BERSERKER_FORWARD_MS / BERSERKER_TURN_MS são aproximados
//  e devem ser calibrados no robô real pra fechar bem os ângulos da estrela.
// =====================================================================================
void BerserkerStep() {
  unsigned long elapsed = millis() - berserkerPhaseStart;

  if (berserkerPhase == 0) {
    NinjaRollForward(roll_right_forward_speed, roll_left_forward_speed);
    if (elapsed >= BERSERKER_FORWARD_MS) {
      berserkerPhase = 1;
      berserkerPhaseStart = millis();
    }
  } else {
    if (berserkerTurnRight) {
      NinjaRollRight(roll_right_forward_speed, roll_left_forward_speed);
    } else {
      NinjaRollLeft(roll_right_forward_speed, roll_left_forward_speed);
    }
    if (elapsed >= BERSERKER_TURN_MS) {
      berserkerLap++;
      // a cada 3 "pontas" inverte o sentido do giro, sobrepondo o segundo
      // triângulo invertido pra formar o hexagrama (estrela de 6 pontas)
      if (berserkerLap % 3 == 0) {
        berserkerTurnRight = !berserkerTurnRight;
      }
      berserkerPhase = 0;
      berserkerPhaseStart = millis();
    }
  }
}
void Home() {
  setServoAngleFoot(CH_FOOT_L, 90);
  setServoAngleFoot(CH_FOOT_R, 90);
  SetWalk();
  headAngle = 90;
  setServoAngle(CH_ARM_R, rightArmAngle);
  setServoAngle(CH_ARM_L, leftArmAngle);
  delay(150);
}
void SetWalk() {
  walkMode = true;
  rollMode = false;
  setServoAngle(CH_LEG_L, 35);
  setServoAngle(CH_LEG_R, 140);
  delay(150);
  releaseServo(CH_LEG_L);
  releaseServo(CH_LEG_R);
  delay(100);
  UpdateDisplay();
}
void SetRoll() {
  walkMode = false;
  rollMode = true;
  setServoAngle(CH_LEG_L, 110);
  setServoAngle(CH_LEG_R, 28);
  delay(150);
  releaseServo(CH_LEG_L);
  releaseServo(CH_LEG_R);
  delay(100);
  UpdateDisplay();
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
void NinjaRollForward(int speedR, int speedL) {
  setServoAngle(CH_FOOT_L, 90 + speedL);
  setServoAngle(CH_FOOT_R, 90 - speedR);
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
  setServoAngle(CH_FOOT_L, 90 - speedL);
  setServoAngle(CH_FOOT_R, 90 + speedR);
}
void NinjaRollStop() {
  setServoAngle(CH_FOOT_L, 90);
  setServoAngle(CH_FOOT_R, 90);
  releaseServo(CH_FOOT_L);
  releaseServo(CH_FOOT_R);
}
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
void SetHeadAngle(int angle) {
  headAngle = constrain(angle, HEAD_MIN, HEAD_MAX);
  setServoAngle(CH_HEAD, headAngle);
}
void joystickWalk(int x, int y) {
  if ((x >= -5) && (x <= 5) && (y >= -5) && (y <= 5)) { command = "home"; }
  else if ((y < -25 && x < -25) || (y > 25 && x < -25)) { command = "left"; }
  else if ((y < -25 && x > 25) || (y > 25 && x > 25)) { command = "right"; }
  else if (y < -25) { command = "forward"; }
  else if (y > 25) { command = "backward"; }
}
void joystickRoll(int x, int y) {
  if ((x >= -5) && (x <= 5) && (y >= -5) && (y <= 5)) {
    NinjaRollStop();
    rollMoving = false;
  } else {
    int LWS = map(y, -50, 50, 135, 45);
    int RWS = map(y, -50, 50, 45, 135);
    int LWD = map(x, 50, -50, 45, 0);
    int RWD = map(x, 50, -50, 0, -45);
    int leftAngle  = 90 + (int)((LWS + LWD - 90) * speedFactor);
    int rightAngle = 90 + (int)((RWS + RWD - 90) * speedFactor);
    setServoAngle(CH_FOOT_L, leftAngle);
    setServoAngle(CH_FOOT_R, rightAngle);
    rollMoving = true;
    lastRollCommandTime = millis();
  }
}
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
}
// =====================================================================================
//  DIAGNÓSTICO I2C / INICIALIZAÇÃO ROBUSTA DO OLED
//  Se o OLED "não der sinal de vida", o Serial Monitor (115200 baud) vai
//  mostrar exatamente quais endereços I2C responderam (ou nenhum) - isso ajuda
//  a distinguir problema de fiação/alimentação de problema de endereço errado.
// =====================================================================================
void scanI2CBus() {
  Serial.println("Procurando dispositivos no barramento I2C...");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  -> dispositivo encontrado no endereco 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  -> nenhum dispositivo respondeu! Confira SDA=GPIO21, SCL=GPIO22, VCC e GND.");
  } else {
    Serial.print(found);
    Serial.println(" dispositivo(s) I2C encontrado(s).");
  }
}

// Tenta iniciar o OLED em um endereço específico, com algumas tentativas
// (a tela às vezes demora um pouco a estabilizar a alimentação).
bool tryOledAddress(uint8_t addr) {
  for (int attempt = 0; attempt < 3; attempt++) {
    if (display.begin(SSD1306_SWITCHCAPVCC, addr)) {
      Serial.print("OLED respondeu no endereco 0x");
      Serial.println(addr, HEX);
      return true;
    }
    delay(150);
  }
  return false;
}

// Tenta os dois endereços mais comuns de SSD1306 (0x3C e 0x3D)
bool initOled() {
  if (tryOledAddress(0x3C)) return true;
  if (tryOledAddress(0x3D)) return true;
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(10);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SHUTDOWN_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);
  playBootTone(); // som ao ligar, antes de tentar conectar no Wi-Fi

  Wire.begin(I2C_SDA, I2C_SCL);
  delay(50); // pequena folga pra a tela e o PCA9685 estabilizarem a alimentação antes do 1º acesso I2C

  scanI2CBus(); // NOVO: lista no Serial tudo que responde no barramento I2C (ajuda a diagnosticar o OLED)

  // NOVO: tenta inicializar o OLED em 0x3C e depois 0x3D, com algumas
  // tentativas em cada um. Se não encontrar a tela, oledOk fica false e o
  // resto do código simplesmente ignora o display, sem travar nada.
  oledOk = initOled();
  if (oledOk) {
    ShowBootScreen();
  } else {
    Serial.println("OLED nao respondeu em 0x3C nem 0x3D - verifique a fiacao/alimentacao/endereco acima.");
  }

  pwm.begin();
  pwm.setPWMFreq(50);
  Serial.println();
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);
  ShowConnectingScreen();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  digitalWrite(LED_PIN, HIGH);
  wasConnected = true;
  playConnectTone(); // som ao conectar no Wi-Fi com sucesso
  delay(100);
  server.begin();
  server.setNoDelay(true); // reduz latência das respostas - ajuda o joystick a não engasgar
  Serial.println("Servidor iniciado");
  Serial.print("Use este IP no site (com barra no final), ex: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  Home();
  UpdateDisplay();
  delay(500);
}
void loop() {
  CheckButtonClicks(); // 1 clique = troca de função, 2 cliques rápidos = desliga
  CheckClient();

  // Se chegou um comando "normal" (não é um dos modos especiais) enquanto um
  // modo especial estava rodando, cancela o modo especial antes de continuar -
  // evita o Sentinela/Sumo/Berserker brigar com um comando manual do usuário.
  bool isSpecialCommand = (command == "sentinela" || command == "sumo" || command == "berserker" || command == "happybirthday");
  if (!isSpecialCommand && command != "" && (sentinelaActive || sumoActive || berserkerActive)) {
    ExitSpecialModes();
    buttonFunctionIndex = -1; // comando manual interrompeu o ciclo - próximo clique começa do zero
  }

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
      rollMoving = true;
      lastRollCommandTime = millis();
    }
  }
  else if (command == "backward") {
    if (walkMode) {
      WalkBackward();
      command = "";
    } else {
      NinjaRollBackward(roll_right_backward_speed, roll_left_backward_speed);
      rollMoving = true;
      lastRollCommandTime = millis();
    }
  }
  else if (command == "right") {
    if (walkMode) {
      WalkRight();
      command = "";
    } else {
      NinjaRollRight(roll_right_forward_speed, roll_left_forward_speed);
      rollMoving = true;
      lastRollCommandTime = millis();
      command = "";
    }
  }
  else if (command == "left") {
    if (walkMode) {
      WalkLeft();
      command = "";
    } else {
      NinjaRollLeft(roll_right_backward_speed, roll_left_backward_speed);
      rollMoving = true;
      lastRollCommandTime = millis();
      command = "";
    }
  }
  else if (command == "stop") {
    ExitSpecialModes();
    command = "";
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
  else if (command == "sentinela") {
    if (!sentinelaActive) {
      sentinelaActive = true;
      sentinelaSide = 0;
      sentinelaStride = 0;
      sentinelaTurning = false;
      sentinelaTurnCount = 0;
      if (!walkMode) SetWalk(); // o Sentinela anda, então garante o modo pé
    }
    SentinelaStep();
  }
  else if (command == "sumo") {
    if (!sumoActive) {
      sumoActive = true;
      sumoLastToggle = millis();
      UpdateDisplay(); // mostra os olhos de raiva imediatamente
    }
    SumoStep();
  }
  else if (command == "berserker") {
    if (!berserkerActive) {
      berserkerActive = true;
      berserkerPhase = 0;
      berserkerLap = 0;
      berserkerTurnRight = true;
      berserkerPhaseStart = millis();
      if (walkMode) SetRoll(); // o Berserker usa o modo roda
      UpdateDisplay();
    }
    BerserkerStep();
  }
  else if (command == "happybirthday") {
    command = ""; // performance única (bloqueante) - não repete a cada volta do loop
    PlayHappyBirthday();
  }
  if (rollMoving && (millis() - lastRollCommandTime > ROLL_WATCHDOG_MS)) {
    NinjaRollStop();
    rollMoving = false;
  }
  // Checa a conexão Wi-Fi periodicamente (sem sobrecarregar o loop).
  // Se estava conectado e caiu, toca o som de desconexão e para as rodas na hora,
  // sem depender apenas do watchdog de 500ms por falta de comando.
  if (millis() - lastWifiCheck > WIFI_CHECK_INTERVAL_MS) {
    lastWifiCheck = millis();
    bool isConnected = (WiFi.status() == WL_CONNECTED);
    if (wasConnected && !isConnected) {
      digitalWrite(LED_PIN, LOW);
      NinjaRollStop();
      rollMoving = false;
      playDisconnectTone();
    } else if (!wasConnected && isConnected) {
      digitalWrite(LED_PIN, HIGH);
      playConnectTone();
    }
    wasConnected = isConnected;
  }

  // NOVO: atualiza o OLED periodicamente (throttled - não precisa redesenhar a
  // cada volta do loop, isso só gastaria tempo de I2C à toa)
  if (millis() - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdate = millis();
    UpdateDisplay();
  }
}
void CheckClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  unsigned long start = millis();
  while (!client.available()) {
    if (millis() - start > 200) {
      client.stop();
      return;
    }
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
  else if (req.indexOf("/speed=") != -1) {
    int idx = req.indexOf("/speed=") + 7;
    String val = "";
    for (int i = idx; i < req.length() && isDigit(req.charAt(i)); i++) {
      val += req.charAt(i);
    }
    int pct = constrain(val.toInt(), 0, 100);
    speedFactor = pct / 100.0;
    command = "";
  }
  else if (req.indexOf("/head=") != -1) {
    int idx = req.indexOf("/head=") + 6;
    String val = "";
    for (int i = idx; i < req.length() && isDigit(req.charAt(i)); i++) {
      val += req.charAt(i);
    }
    SetHeadAngle(val.toInt());
    command = "";
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
  else if (req.indexOf("/sentinela") != -1) {
    command = "sentinela";
  }
  else if (req.indexOf("/sumo") != -1) {
    command = "sumo";
  }
  else if (req.indexOf("/berserker") != -1) {
    command = "berserker";
  }
  else if (req.indexOf("/happybirthday") != -1) {
    command = "happybirthday";
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

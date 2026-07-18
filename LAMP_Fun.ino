// LAMP_Fun V.2.6.4
// José Luís Marcos Bezos - Junio 2026.
// ESP32 + TFT ST7789 240x240 con Encoder EC11 con pulsador
// pulsador extra + WS2812B + INMP441 + MAX98357A

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <FastLED.h>
#include "time.h"
#include "pins.h"
#include <TFT_eSPI.h>
#include <math.h>

// ----------------- Config WiFi / NTP -----------------

const char* NTP_SERVER = "pool.ntp.org";

const char* WIFI_NAMESPACE = "wifi_cfg";

const int WIFI_SSID_BUF_LEN = 32;
const int WIFI_PWD_BUF_LEN  = 64;

char wifiSsid[WIFI_SSID_BUF_LEN + 1] = {0};
char wifiPwdEnc[WIFI_PWD_BUF_LEN + 1] = {0};

bool hasWifiCredentials = false;

// Reintento WiFi
unsigned long lastWifiCheckMillis = 0;
const unsigned long WIFI_RETRY_INTERVAL = 30000; // 30 s

// Resincronización NTP
unsigned long lastNtpSyncMillis = 0;
const unsigned long NTP_SYNC_INTERVAL = 6UL * 60UL * 60UL * 1000UL; // 6 horas

// ----------------- Tabla de zonas horarias -----------------

struct TimezoneEntry {
  const char* name;
  const char* tzStr;
};

const TimezoneEntry TIMEZONES[] = {
  { "UTC",            "UTC0" },
  { "Europa/Madrid",  "CET-1CEST,M3.5.0,M10.5.0/3" },
  { "Europa/Londres", "GMT0BST,M3.5.0/1,M10.5.0/2" },
  { "America/NY",     "EST5EDT,M3.2.0,M11.1.0" },
  { "America/BsAs",   "ART3" },
  { "Asia/Tokio",     "JST-9" },
  { "Australia/Syd",  "AEST-10AEDT,M10.1.0,M4.1.0/3" }
};
const int TIMEZONES_COUNT = sizeof(TIMEZONES) / sizeof(TIMEZONES[0]);

int8_t tzIndex       = 1;  // por defecto Europa/Madrid
int8_t tzOffsetSteps = 0;  // -4..+4 => -2.0..+2.0 h en pasos 0.5h

// ----------------- Configuración general -----------------

TFT_eSPI tft;
CRGB     leds[NUM_LEDS];

// ---------- Geometría del foco: "Foco de José Luís" ----------

const int NUM_RINGS = 9;
const int ringLength[NUM_RINGS] = {60, 48, 40, 32, 24, 16, 12, 8, 1};
const int ringStart[NUM_RINGS]  = {  0, 60,108,148,180,204,220,232,240 };

// Acceso cómodo: índice global de "aro" y "posición en aro" (ambos 0-based, horario)
inline int ringLedIndex(int ring, int posInRing) {
  int len = ringLength[ring];
  if (posInRing < 0) posInRing = (posInRing % len) + len;
  posInRing %= len;
  return ringStart[ring] + posInRing;
}

Preferences prefs;

bool    lampOn       = true;
bool    rainbowMode  = false;
uint8_t brightness   = 5;
uint8_t redValue     = 50;
uint8_t greenValue   = 50;
uint8_t blueValue    = 50;
uint8_t rainbowHue   = 0;

// ----------------- Efectos (infraestructura común) -----------------

// Flag global: hay algún efecto en ejecución
bool anyEffectActive = false;

// Flags específicos (para RESPIRACION ahora, futuros efectos después)
bool respEffectActive = false;

// Estado de RESPIRACION
uint8_t respPhase = 0;           // 0..255 fase de brillo
bool respEffectForward = true;   // subir/bajar
unsigned long respLastUpdate = 0;
uint16_t respIntervalMs = 20;    // velocidad base del efecto (ajustable después)

// Configuración de RESPIRACION
uint16_t respColorStart = 0x0000;  // negro por defecto
uint16_t respColorEnd   = 0xFFFF;  // blanco por defecto

// Índice de ciclo: 0..13 => 0.2, 0.4, 0.6, 0.8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 s
uint8_t  respCycleIndex = 4;       // por defecto 1 s (ver tabla más abajo)

// Tabla de duraciones de ciclo para RESPIRACION (en segundos, *10 para evitar float)
const uint8_t RESP_CYCLE_STEPS = 14;
const uint16_t respCycleTimesX10[RESP_CYCLE_STEPS] = {
  2, 4, 6, 8, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
}; 
// Representan: 0.2, 0.4, 0.6, 0.8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 s

// Ciclos COMETA: 1..10 s (en pasos de 1 s) en décimas
const uint16_t cometCycleTimesX10[10] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 };

// Estado de UI para pantalla RESPIRACION
enum RespFocus {
  RESP_FOCUS_START = 0,   // knob/color inicial
  RESP_FOCUS_END,         // knob/color final
  RESP_FOCUS_CYCLE,       // ciclo
  RESP_FOCUS_BUTTON       // botón Iniciar
};

RespFocus respFocus = RESP_FOCUS_START;

enum CometFocus {
  COMET_FOCUS_START,
  COMET_FOCUS_END,
  COMET_FOCUS_CYCLE,
  COMET_FOCUS_BUTTON
};

CometFocus cometFocus = COMET_FOCUS_START;

// Posiciones de knobs en el slider (0..sliderWidth-1)
int respKnobStartPos = 0;
int respKnobEndPos   = 0;

// COMETA: knobs propios
int cometKnobStartPos = 0;
int cometKnobEndPos   = 0;

bool use24hFormat = true;
bool useAutoTime  = true;

uint8_t  clockMode = 0; // 0 digital, 1 analógico

uint16_t digitalHMColor   = TFT_WHITE;
uint16_t digitalDateColor = TFT_WHITE;

uint16_t analogHourHandColor  = TFT_WHITE;
uint16_t analogMinHandColor   = TFT_WHITE;
uint16_t analogSecHandColor   = TFT_RED;
uint16_t analogDateColor      = TFT_WHITE;
uint16_t analogFaceFillColor  = TFT_BLACK;

bool firstManualBoot = false;

// ---------- Efecto COMETA ----------

bool cometEffectActive = false;      // este efecto está en marcha
// reutilizaremos anyEffectActive junto con cometEffectActive

// Configuración de usuario
uint16_t cometColorStart = 0xF800;   // rojo por defecto
uint16_t cometColorEnd   = 0x001F;   // azul por defecto

// Ciclo: tiempo total del efecto (de aros 1-4 hasta desaparecer en aro 9)
// Índices 0..9 => 1..10 segundos
uint8_t  cometCycleIndex = 2;        // por defecto 3 s aprox

// Dinámica interna del cometa
float    cometPhase = 0.0f;          // 0..1 a lo largo de TODO el recorrido
unsigned long cometLastUpdate = 0;

// Parámetros del recorrido
const int COMET_RING_WIDTH = 4;      // 4 aros de grosor de cometa
const int COMET_HEAD_LEN   = 3;      // LEDs de longitud angular de la cabeza
const int COMET_TAIL_LEN   = 18;     // LEDs efectivos de cola (ángulo)

// ---------- Efecto BARRIDO ----------

bool barridoEffectActive = false;    // este efecto está en marcha

// Configuración de usuario
uint16_t barridoColorStart = 0xF800;   // rojo por defecto
uint16_t barridoColorEnd   = 0x001F;   // azul por defecto

// Ciclo: tiempo total del barrido (subida + bajada)
// Índices 0..12 => 0.4, 0.6, 0.8, 1, 2..10 s
uint8_t  barridoCycleIndex = 4;        // por defecto 2 s aprox

// Dinámica interna del barrido
float    barridoPhase = 0.0f;          // 0..1 a lo largo de TODO el ciclo (sube + baja)
unsigned long barridoLastUpdate = 0;

// Knobs propios del slider de BARRIDO
int barridoKnobStartPos = 0;
int barridoKnobEndPos   = 0;

// Foco de la pantalla BARRIDO
enum BarridoFocus {
  BARRIDO_FOCUS_START,
  BARRIDO_FOCUS_END,
  BARRIDO_FOCUS_CYCLE,
  BARRIDO_FOCUS_BUTTON
};

BarridoFocus barridoFocus = BARRIDO_FOCUS_START;

// Tabla de tiempos de ciclo para BARRIDO (en décimas de segundo)
// 0.4, 0.6, 0.8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
const uint8_t BARRIDO_CYCLE_STEPS = 14;
const uint16_t barridoCycleTimesX10[BARRIDO_CYCLE_STEPS] = {
  2, 4, 6, 8, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
};

// ---------- Efecto PERSIANA ----------
bool persianaEffectActive = false;    // este efecto está en marcha

// Configuración de usuario
uint16_t persianaColorStart = 0xF800; // rojo por defecto
uint16_t persianaColorEnd   = 0x001F; // azul por defecto

// Ciclo: tiempo total del efecto (subida completa + bajada completa)
// Índices 0..13 => 0.2, 0.4, 0.6, 0.8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 s
uint8_t persianaCycleIndex = 4;       // por defecto 1 s

// Dinámica interna de la persiana
float persianaPhase = 0.0f;           // 0..1 a lo largo de TODO el ciclo
unsigned long persianaLastUpdate = 0; // para control de tiempo

// Knobs propios del slider de PERSIANA
int persianaKnobStartPos = 0;
int persianaKnobEndPos   = 0;

// Foco de la pantalla PERSIANA
enum PersianaFocus {
  PERSIANA_FOCUS_START,
  PERSIANA_FOCUS_END,
  PERSIANA_FOCUS_CYCLE,
  PERSIANA_FOCUS_BUTTON
};
PersianaFocus persianaFocus = PERSIANA_FOCUS_START;

// Tabla de tiempos de ciclo para PERSIANA (en décimas de segundo)
// 0.2, 0.4, 0.6, 0.8, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
const uint8_t PERSIANA_CYCLE_STEPS = 14;
const uint16_t persianaCycleTimesX10[PERSIANA_CYCLE_STEPS] = {
   2,  4,  6,  8, 10,
  20, 30, 40, 50,
  60, 70, 80, 90,
 100
};

// ----------------- Backlight TFT -----------------

const int TFT_BL_FREQUENCY = 5000;
const int TFT_BL_RES       = 8;

uint8_t tftBacklightLevel = 100;
bool    tftBacklightEnabled = true;

uint8_t backlightPercentToDuty(uint8_t level, bool enabled) {
  if (!enabled || level == 0) return 0;
  if (level >= 100) return 255;
  return (uint8_t)((level * 255UL) / 100UL);
}

void applyBacklightPWM() {
  uint8_t duty = backlightPercentToDuty(tftBacklightLevel, tftBacklightEnabled);
  ledcWrite(TFT_BL_PIN, duty);
}

void initBacklight() {
  pinMode(TFT_BL_PIN, OUTPUT);
  tftBacklightLevel   = 100;
  tftBacklightEnabled = true;
  ledcAttach(TFT_BL_PIN, TFT_BL_FREQUENCY, TFT_BL_RES);
  applyBacklightPWM();
}

// ----------------- Pantallas -----------------

enum Screen {
  SCREEN_SPLASH = 0,
  SCREEN_CLOCK,
  SCREEN_LIGHT,
  SCREEN_SETTINGS_CLOCK,
  SCREEN_SETTINGS_FORMAT,
  SCREEN_SETTINGS_DATEMODE,
  SCREEN_SETTINGS_DATETIME,
  SCREEN_SETTINGS_TIMEZONE,
  SCREEN_SETTINGS_TZOFFSET,
  SCREEN_SETTINGS_MAIN,
  SCREEN_SETTINGS_EFFECTS,
  SCREEN_SETTINGS_RESP,
  SCREEN_SETTINGS_COMET,
  SCREEN_SETTINGS_BARRIDO,
  SCREEN_SETTINGS_PERSIANA,
  SCREEN_SETTINGS_BACKLIGHT,
  SCREEN_SETTINGS_COLORS_DIGITAL,
  SCREEN_SETTINGS_COLORS_ANALOG,
  SCREEN_SETTINGS_WIFI,
  SCREEN_SETTINGS_WIFI_SCAN,
  SCREEN_SETTINGS_WIFI_PWD,
  SCREEN_SETTINGS_RESET_CONFIRM,
  SCREEN_SETTINGS_ABOUT
};

Screen currentScreen = SCREEN_SPLASH;

const unsigned long SPLASH_DURATION   = 2500;
unsigned long       splashStartMillis = 0;

const char* MONTH_NAMES_ES[12] = {
  "Enero", "Febrero", "Marzo", "Abril", "Mayo", "Junio",
  "Julio", "Agosto", "Septiembre", "Octubre", "Noviembre", "Diciembre"
};

// ----------------- Encoder -----------------

int  currentControl = 0;
bool editingBar     = false;
int  lastEncA       = 0;

int readEncoderStep() {
  int encA    = digitalRead(ENCODER_A);
  int encB    = digitalRead(ENCODER_B);
  int stepDir = 0;

  if (lastEncA == LOW && encA == HIGH) {
    if (encB == LOW) stepDir = +1;
    else             stepDir = -1;
  }
  lastEncA = encA;
  return stepDir;
}

// ----------------- NVS (config general) -----------------

void loadConfig() {
  if (!prefs.begin("lamp_cfg", false)) {
    redValue          = 50;
    greenValue        = 50;
    blueValue         = 50;
    brightness        = 5;
    use24hFormat      = true;
    useAutoTime       = true;
    tftBacklightLevel = 100;
    clockMode         = 0;
    tzIndex           = 1;
    tzOffsetSteps     = 0;
    return;
  }

  redValue          = (uint8_t)prefs.getUChar("R", 50);
  greenValue        = (uint8_t)prefs.getUChar("G", 50);
  blueValue         = (uint8_t)prefs.getUChar("B", 50);
  brightness        = (uint8_t)prefs.getUChar("Br", 5);
  use24hFormat      = prefs.getBool("24h", true);
  useAutoTime       = prefs.getBool("autoTime", true);
  tftBacklightLevel = prefs.getUChar("BLp", 100);
  clockMode         = prefs.getUChar("clkMode", 0);

  digitalHMColor   = prefs.getUShort("dhmColor", TFT_WHITE);
  digitalDateColor = prefs.getUShort("ddColor",  TFT_WHITE);

  analogHourHandColor = prefs.getUShort("ahColor", TFT_WHITE);
  analogMinHandColor  = prefs.getUShort("amColor", TFT_WHITE);
  analogSecHandColor  = prefs.getUShort("asColor", TFT_RED);
  analogDateColor     = prefs.getUShort("adColor", TFT_WHITE);
  analogFaceFillColor = prefs.getUShort("afColor", TFT_BLACK);

  tzIndex       = (int8_t)prefs.getChar("tzIndex", 1);
  tzOffsetSteps = (int8_t)prefs.getChar("tzOff", 0);

  // Config RESPIRACION
  respColorStart = prefs.getUShort("respC0", 0x0000);  // negro por defecto
  respColorEnd   = prefs.getUShort("respC1", 0xFFFF);  // blanco por defecto
  respCycleIndex = prefs.getUChar ("respCi", 4);       // por defecto índice 4 => 1.0 s
  if (respCycleIndex >= RESP_CYCLE_STEPS) respCycleIndex = RESP_CYCLE_STEPS - 1;

  // Config COMETA
  cometColorStart = prefs.getUShort("comC0", 0xF800); // rojo por defecto
  cometColorEnd   = prefs.getUShort("comC1", 0x001F); // azul por defecto
  cometCycleIndex = prefs.getUChar ("comCi", 2);
  if (cometCycleIndex > 9) cometCycleIndex = 9;       // 0..9 => 1..10 s

  // Config BARRIDO
  barridoColorStart = prefs.getUShort("barC0", 0xF800); // rojo por defecto
  barridoColorEnd   = prefs.getUShort("barC1", 0x001F); // azul por defecto
  barridoCycleIndex = prefs.getUChar ("barCi", 4);      // índice por defecto
  if (barridoCycleIndex >= BARRIDO_CYCLE_STEPS) barridoCycleIndex = BARRIDO_CYCLE_STEPS - 1;

  // Config PERSIANA
  persianaColorStart = prefs.getUShort("perC0", 0xF800); // rojo por defecto
  persianaColorEnd   = prefs.getUShort("perC1", 0x001F); // azul por defecto
  persianaCycleIndex = prefs.getUChar ("perCi", 4);      // índice por defecto 1 s
  if (persianaCycleIndex >= PERSIANA_CYCLE_STEPS) persianaCycleIndex = PERSIANA_CYCLE_STEPS - 1;

  prefs.end();

  if (tftBacklightLevel > 100) tftBacklightLevel = 100;
  if (clockMode > 1) clockMode = 0;

  if (tzIndex < 0 || tzIndex >= TIMEZONES_COUNT) tzIndex = 1;
  if (tzOffsetSteps < -4) tzOffsetSteps = -4;
  if (tzOffsetSteps > 4)  tzOffsetSteps = 4;

  firstManualBoot = !useAutoTime;
}

void saveConfigBasic() {
  if (!prefs.begin("lamp_cfg", false)) {
    return;
  }

  prefs.putUChar("R",   redValue);
  prefs.putUChar("G",   greenValue);
  prefs.putUChar("B",   blueValue);
  prefs.putUChar("Br",  brightness);
  prefs.putBool ("24h", use24hFormat);
  prefs.putBool ("autoTime", useAutoTime);
  prefs.putUChar("BLp", tftBacklightLevel);
  prefs.putUChar("clkMode", clockMode);

  prefs.putUShort("dhmColor", digitalHMColor);
  prefs.putUShort("ddColor",  digitalDateColor);

  prefs.putUShort("ahColor", analogHourHandColor);
  prefs.putUShort("amColor", analogMinHandColor);
  prefs.putUShort("asColor", analogSecHandColor);
  prefs.putUShort("adColor", analogDateColor);
  prefs.putUShort("afColor", analogFaceFillColor);

  prefs.putChar("tzIndex", (char)tzIndex);
  prefs.putChar("tzOff",   (char)tzOffsetSteps);

  // Config RESPIRACION
  prefs.putUShort("respC0", respColorStart);
  prefs.putUShort("respC1", respColorEnd);
  prefs.putUChar ("respCi", respCycleIndex);

  // Config COMETA
  prefs.putUShort("comC0", cometColorStart);
  prefs.putUShort("comC1", cometColorEnd);
  prefs.putUChar ("comCi", cometCycleIndex);

  // Config BARRIDO
  prefs.putUShort("barC0", barridoColorStart);
  prefs.putUShort("barC1", barridoColorEnd);
  prefs.putUChar ("barCi", barridoCycleIndex);

  // Config PERSIANA
  prefs.putUShort("perC0", persianaColorStart);
  prefs.putUShort("perC1", persianaColorEnd); 
  prefs.putUChar ("perCi", persianaCycleIndex);

  prefs.end();
}

// ----------------- WiFi: ofuscación y NVS -----------------

uint8_t wifiObfKey = 0x5A;

void wifiEncodePassword(const char* plain, char* outEnc, int outLen) {
  int n = strlen(plain);
  if (n > outLen - 1) n = outLen - 1;
  for (int i = 0; i < n; i++) {
    outEnc[i] = plain[i] ^ wifiObfKey;
  }
  outEnc[n] = '\0';
}

void wifiDecodePassword(const char* enc, char* outPlain, int outLen) {
  int n = strlen(enc);
  if (n > outLen - 1) n = outLen - 1;
  for (int i = 0; i < n; i++) {
    outPlain[i] = enc[i] ^ wifiObfKey;
  }
  outPlain[n] = '\0';
}

void wifiLoadCredentials() {
  Preferences wprefs;
  if (!wprefs.begin(WIFI_NAMESPACE, true)) {
    wifiSsid[0]  = '\0';
    wifiPwdEnc[0]= '\0';
    hasWifiCredentials = false;
    return;
  }

  String sSsid = wprefs.getString("ssid", "");
  String sPwd  = wprefs.getString("pwd",  "");
  wprefs.end();

  sSsid.toCharArray(wifiSsid, sizeof(wifiSsid));
  sPwd.toCharArray(wifiPwdEnc, sizeof(wifiPwdEnc));

  hasWifiCredentials = (wifiSsid[0] != '\0');
}

void wifiSaveCredentials(const char* ssid, const char* pwdEnc) {
  Preferences wprefs;
  if (!wprefs.begin(WIFI_NAMESPACE, false)) {
    return;
  }

  wprefs.putString("ssid", String(ssid));
  wprefs.putString("pwd",  String(pwdEnc));
  wprefs.end();

  hasWifiCredentials = (ssid[0] != '\0');
}

void wifiForgetCredentials() {
  Preferences wprefs;
  if (!wprefs.begin(WIFI_NAMESPACE, false)) {
    return;
  }

  wprefs.remove("ssid");
  wprefs.remove("pwd");
  wprefs.end();

  wifiSsid[0]  = '\0';
  wifiPwdEnc[0]= '\0';
  hasWifiCredentials = false;
}

int wifiRssiToPercent(long rssi) {
  const long RSSI_MAX = -30;
  const long RSSI_MIN = -90;

  if (rssi <= RSSI_MIN) return 0;
  if (rssi >= RSSI_MAX) return 100;

  return (int)((float)(rssi - RSSI_MIN) * 100.0f /
               (float)(RSSI_MAX - RSSI_MIN));
}

bool wifiConnectUsingStored(uint16_t timeoutMs = 5000) {
  if (!hasWifiCredentials || wifiSsid[0] == '\0') return false;

  char plainPwd[WIFI_PWD_BUF_LEN + 1];
  wifiDecodePassword(wifiPwdEnc, plainPwd, sizeof(plainPwd));

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, plainPwd[0] ? plainPwd : nullptr);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED &&
         (millis() - wifiStart < timeoutMs)) {
    delay(100);
  }
  return (WiFi.status() == WL_CONNECTED);
}

// ----------------- LEDs -----------------

void updateLeds() {
  if (!lampOn) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  } else if (rainbowMode) {
    fill_rainbow(leds, NUM_LEDS, rainbowHue, 8);
  } else {
    fill_solid(leds, NUM_LEDS, CRGB(redValue, greenValue, blueValue));
  }
  FastLED.setBrightness(brightness);
  FastLED.show();
}

// Apagado duro de todos los leds
void clearAllLedsAndShow() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

// Parar TODOS los efectos activos y dejar leds en negro
void stopAllEffects() {
  if (!anyEffectActive) return;

  // Desactivar flags de efectos actuales
  respEffectActive  = false;
  cometEffectActive = false;
  barridoEffectActive = false;
  persianaEffectActive = false;
  anyEffectActive   = false;

  // Apagado forzoso en hardware
  updateLeds();
}

// Iniciar RESPIRACION
void startRespEffect() {
  // Asegurarnos de que no quedan otros efectos
  stopAllEffects();
  respEffectActive  = true;
  anyEffectActive   = true;
  respEffectForward = true;
  respPhase         = 0;
  respLastUpdate    = millis();
}

// Parar RESPIRACION (usamos infraestructura común)
void stopRespEffect() {
  stopAllEffects();
}

void startCometEffect() {
  stopAllEffects();
  cometEffectActive = true;
  anyEffectActive   = true;
  cometPhase        = 0.0f;
  cometLastUpdate   = millis();
}

void stopCometEffect() {
  stopAllEffects();
}

void startBarridoEffect() {
  stopAllEffects();
  barridoEffectActive = true;
  anyEffectActive     = true;
  barridoPhase        = 0.0f;
  barridoLastUpdate   = millis();
}

void stopBarridoEffect() {
  stopAllEffects();
}

void startPersianaEffect() {
  stopAllEffects();
  persianaEffectActive = true;
  anyEffectActive      = true;
  persianaPhase        = 0.0f;
  persianaLastUpdate   = millis();
}

void stopPersianaEffect() {
  stopAllEffects();
}

// Actualizar RESPIRACION (se llamará desde loop)
void updateRespEffect() {
  if (!respEffectActive) return;

  unsigned long now = millis();
  if (now - respLastUpdate < respIntervalMs) return;
  unsigned long dtMs = now - respLastUpdate;
  respLastUpdate = now;

  // Duración total del ciclo (inicio -> final -> inicio) en milisegundos
  uint16_t tX10 = respCycleTimesX10[respCycleIndex];   // en décimas de segundo
  float cycleSeconds = tX10 / 10.0f;                   // en segundos
  if (cycleSeconds < 0.2f) cycleSeconds = 0.2f;        // seguridad: mínimo 0.2 s
  float cycleMs = cycleSeconds * 1000.0f;

  // Fase absoluta en ms dentro del ciclo (0 .. cycleMs)
  static float phaseMs = 0.0f;
  phaseMs += (float)dtMs;
  if (phaseMs >= cycleMs) {
    phaseMs -= cycleMs;  // wrap al principio del ciclo
  }

  // Fase normalizada 0..1 a lo largo de TODO el ciclo
  float phase = phaseMs / cycleMs;  // 0..1

  // Mapeo 0..1:
  //  0.0  -> color inicio
  //  0.5  -> color final
  //  1.0  -> color inicio
  float tt;
  if (phase <= 0.5f) {
    tt = phase / 0.5f;          // 0..1 (inicio -> final)
  } else {
    tt = (1.0f - phase) / 0.5f; // 1..0 (final -> inicio)
  }

  // Interpolar en HSV entre respColorStart y respColorEnd
  uint8_t r0, g0, b0;
  uint8_t r1, g1, b1;
  rgbFrom565(respColorStart, r0, g0, b0);
  rgbFrom565(respColorEnd,   r1, g1, b1);

  // Convertimos ambos a Hue(0..240) y Value(0..1)
  float h0, v0;
  float h1, v1;
  rgbToHueValue240(r0, g0, b0, h0, v0);
  rgbToHueValue240(r1, g1, b1, h1, v1);

  // Interpolamos tono y valor
  float hInterp, vInterp;

  // Elegimos el camino corto de hue entre h0 y h1
  float dh = h1 - h0;
  if (dh > 120.0f)      dh -= 240.0f;
  else if (dh < -120.0f) dh += 240.0f;

  hInterp = h0 + dh * tt;
  if (hInterp < 0.0f)   hInterp += 240.0f;
  if (hInterp > 240.0f) hInterp -= 240.0f;

  vInterp = v0 + (v1 - v0) * tt;

  // Saturación: suponemos siempre 1.0 para mantener colores vivos
  uint8_t r, g, b;
  {
    uint8_t rr, gg, bb;
    // hsvToRgb240 usa tono 0..240, saturación=1, valor=vInterp
    // La función actual asume S=1,V=1, así que adaptamos: escalamos luego por vInterp
    hsvToRgb240(hInterp, rr, gg, bb);
    float scale = vInterp;  // 0..1
    r = (uint8_t)roundf(rr * scale);
    g = (uint8_t)roundf(gg * scale);
    b = (uint8_t)roundf(bb * scale);
  }

  // Aplicar el color interpolado a todos los LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r, g, b);
  }
  FastLED.setBrightness(brightness);
  FastLED.show();
}

void updateCometEffect() {
  if (!cometEffectActive) return;

  unsigned long now = millis();
  // Para mantener fluidez, actualizamos cada 20 ms aprox (igual que RESPIRACION)
  const uint16_t intervalMs = 20;
  if (now - cometLastUpdate < intervalMs) return;
  unsigned long dtMs = now - cometLastUpdate;
  cometLastUpdate = now;

  // Duración total del ciclo COMETA en ms (de aros 1-4 hasta desaparecer en aro 9)
  uint16_t tX10 = cometCycleTimesX10[cometCycleIndex]; // en décimas de segundo
  float cycleSeconds = tX10 / 10.0f;                   // en segundos
  if (cycleSeconds < 1.0f) cycleSeconds = 1.0f;        // seguridad: mínimo 1 s
  float cycleMs = cycleSeconds * 1000.0f;

  // Fase normalizada 0..1 a lo largo de TODO el recorrido
  // cometPhase ya es 0..1; la avanzamos en función de dtMs / cycleMs
  float dPhase = (float)dtMs / cycleMs;
  cometPhase += dPhase;
  if (cometPhase > 1.0f) {
    cometPhase -= 1.0f; // wrap al principio del ciclo
  }

  // Definimos las "etapas" de aros activos (cada etapa = 4 aros consecutivos)
  // Etapa 0: aros 0-3 (1-4)
  // Etapa 1: aros 1-4 (2-5)
  // ...
  // Etapa 5: aros 5-8 (6-9)
  const int NUM_STAGES = 6;
  float stageDuration = 1.0f / (float)NUM_STAGES; // cada etapa ocupa la misma fracción del ciclo
  int stage = (int)(cometPhase / stageDuration);
  if (stage >= NUM_STAGES) stage = NUM_STAGES - 1;

  // Fase local dentro de la etapa actual (0..1)
  float stagePhaseStart = stage * stageDuration;
  float localPhase = (cometPhase - stagePhaseStart) / stageDuration;
  if (localPhase < 0.0f) localPhase = 0.0f;
  if (localPhase > 1.0f) localPhase = 1.0f;

  // Aros activos en esta etapa
  int baseRing = stage; // 0..5
  int rings[COMET_RING_WIDTH];
  for (int i = 0; i < COMET_RING_WIDTH; i++) {
    rings[i] = baseRing + i; // p.ej. etapa 0 => 0,1,2,3 (aros 1-4)
    if (rings[i] >= NUM_RINGS) rings[i] = NUM_RINGS - 1;
  }

  // Convertimos la fase local 0..1 en una posición angular 0..(maxLen-1)
  // Usamos el aro "más exterior" de la etapa como referencia para el ángulo
  int refRing = rings[0];            // aro más externo de los 4
  int refLen  = ringLength[refRing]; // número de leds de ese aro

  float pos = localPhase * (float)refLen; // 0..refLen
  int headCenterPos = (int)pos;           // LED central de la cabeza en ese aro

  // Limpiamos todos los LEDs antes de dibujar el cometa
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Convertimos colores de inicio/fin a RGB
  uint8_t rHead, gHead, bHead;
  uint8_t rTail, gTail, bTail;
  rgbFrom565(cometColorStart, rHead, gHead, bHead);
  rgbFrom565(cometColorEnd,   rTail, gTail, bTail);

  // Dibujar cometa en los 4 aros activos
  for (int ri = 0; ri < COMET_RING_WIDTH; ri++) {
    int ring = rings[ri];
    int len  = ringLength[ring];

    // Para cada aro, adaptamos la posición proporcionalmente a su longitud
    float scale = (float)len / (float)refLen;
    int localCenter = (int)(pos * scale);

    // Dibujar cabeza: desde (center - headLen/2) hasta (center + headLen/2)
    for (int h = -COMET_HEAD_LEN / 2; h <= COMET_HEAD_LEN / 2; h++) {
      int p = localCenter + h;
      int idx = ringLedIndex(ring, p);

      // Núcleo de la cabeza = color inicial puro
      float headFactor = 1.0f;
      // Podemos atenuar un poco los extremos de la cabeza
      if (h == -COMET_HEAD_LEN / 2 || h == COMET_HEAD_LEN / 2) {
        headFactor = 0.6f;
      }

      uint8_t rr = (uint8_t)(rHead * headFactor);
      uint8_t gg = (uint8_t)(gHead * headFactor);
      uint8_t bb = (uint8_t)(bHead * headFactor);

      leds[idx] = CRGB(rr, gg, bb);
    }

    // Dibujar cola detrás de la cabeza, en la misma dirección de avance (horaria)
    for (int t = 1; t <= COMET_TAIL_LEN; t++) {
      int p = localCenter - COMET_HEAD_LEN / 2 - t;
      int idx = ringLedIndex(ring, p);

      // t=1 muy cerca de la cabeza (casi color inicial),
      // t=COMET_TAIL_LEN al final de la cola (color final).
      float tailPos = (float)t / (float)COMET_TAIL_LEN; // 0..1
      float inv = 1.0f - tailPos;

      float rf = rHead * inv + rTail * tailPos;
      float gf = gHead * inv + gTail * tailPos;
      float bf = bHead * inv + bTail * tailPos;

      uint8_t rr = (uint8_t)rf;
      uint8_t gg = (uint8_t)gf;
      uint8_t bb = (uint8_t)bf;

      // Si ya hay algo dibujado (por solapamiento de aros), nos quedamos con el más brillante
      CRGB existing = leds[idx];
      if (existing.r + existing.g + existing.b < rr + gg + bb) {
        leds[idx] = CRGB(rr, gg, bb);
      }
    }
  }

  FastLED.setBrightness(brightness);
  FastLED.show();
}

void updateBarridoEffect() {
  if (!barridoEffectActive) return;

  unsigned long now = millis();

  // Intervalo base de refresco (similar a RESPIRACION/COMETA)
  const uint16_t intervalMs = 20;
  if (now - barridoLastUpdate < intervalMs) return;
  unsigned long dtMs = now - barridoLastUpdate;
  barridoLastUpdate = now;

  // Duración total del ciclo BARRIDO (subida + bajada) en ms
  uint16_t tX10 = barridoCycleTimesX10[barridoCycleIndex];   // en décimas de segundo
  float cycleSeconds = tX10 / 10.0f;                         // en segundos
  if (cycleSeconds < 0.2f) cycleSeconds = 0.2f;              // seguridad: mínimo 0.2 s
  float cycleMs = cycleSeconds * 1000.0f;

  // Avanzar fase normalizada 0..1 en función de dtMs / cycleMs
  float dPhase = (float)dtMs / cycleMs;
  barridoPhase += dPhase;
  if (barridoPhase > 1.0f) {
    barridoPhase -= 1.0f;  // wrap al principio del ciclo
  }

  // Fase local de subida/bajada:
  // 0.0..0.5 => sube (exterior -> interior)
  // 0.5..1.0 => baja (interior -> exterior)
  bool goingDown = false;
  float localPhase = barridoPhase;
  if (localPhase <= 0.5f) {
    // Subida: normalizamos 0..0.5 a 0..1
    localPhase = localPhase / 0.5f;
  } else {
    // Bajada: 0.5..1 -> 1..0
    localPhase = (1.0f - localPhase) / 0.5f;
    goingDown = true;
  }

  if (localPhase < 0.0f) localPhase = 0.0f;
  if (localPhase > 1.0f) localPhase = 1.0f;

  // Mapeamos localPhase (0..1) a una "fila" entre aro 0 y aro 8
  // (9 aros => 0..8). Usamos un float para permitir posiciones intermedias.
  float ringPos = localPhase * (float)(NUM_RINGS - 1); // 0..8
  // Centro de la cortina en índice de aro (float)
  float centerRingIndex = ringPos;

  // Ancho de la cortina en aros (p.ej. 4 aros de grosor)
  const int BARRIDO_RING_WIDTH = 4;

  // Limpiamos todos los LEDs antes de dibujar la cortina
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Convertimos colores inicio/fin a RGB
  uint8_t r0, g0, b0;
  uint8_t r1, g1, b1;
  rgbFrom565(barridoColorStart, r0, g0, b0);
  rgbFrom565(barridoColorEnd,   r1, g1, b1);

  // Recorremos todos los aros y los coloreamos según su "distancia" al centro de la cortina
  for (int ring = 0; ring < NUM_RINGS; ring++) {
    float dist = fabsf((float)ring - centerRingIndex); // 0 en el centro, mayor cuanto más lejos

    if (dist > (float)BARRIDO_RING_WIDTH) {
      // Fuera del grosor de la cortina: queda negro
      continue;
    }

    // factor 0 en el borde, 1 en el centro
    float factor = 1.0f - (dist / (float)BARRIDO_RING_WIDTH);
    if (factor < 0.0f) factor = 0.0f;
    if (factor > 1.0f) factor = 1.0f;

    // Mezclamos entre colorStart y colorEnd según "factor"
    float rf = r0 + (r1 - r0) * factor;
    float gf = g0 + (g1 - g0) * factor;
    float bf = b0 + (b1 - b0) * factor;
    uint8_t rr = (uint8_t)rf;
    uint8_t gg = (uint8_t)gf;
    uint8_t bb = (uint8_t)bf;

    // Aplicamos el color a TODO el aro
    int len = ringLength[ring];
    for (int p = 0; p < len; p++) {
      int idx = ringLedIndex(ring, p);
      leds[idx] = CRGB(rr, gg, bb);
    }
  }

  FastLED.setBrightness(brightness);
  FastLED.show();
}

void updatePersianaEffect();

// ----------------- Icono WiFi -----------------

const int WIFI_ICON_X = 215;
const int WIFI_ICON_Y = 4;
const int WIFI_ICON_W = 30;
const int WIFI_ICON_H = 20;

int  lastWifiBars    = -1;
bool lastWifiTachado = false;

void drawWifiSignalIcon() {
  if (!hasWifiCredentials) {
    tft.fillRect(WIFI_ICON_X, WIFI_ICON_Y, WIFI_ICON_W, WIFI_ICON_H, TFT_BLACK);
    lastWifiBars    = -1;
    lastWifiTachado = false;
    return;
  }

  bool conectado = (WiFi.status() == WL_CONNECTED);
  bool tachado   = (!conectado && useAutoTime);

  int bars = 0;
  if (conectado) {
    long rssi   = WiFi.RSSI();
    int  percent= wifiRssiToPercent(rssi);
    if (percent > 0)  bars = 1;
    if (percent > 25) bars = 2;
    if (percent > 50) bars = 3;
    if (percent > 75) bars = 4;
  } else {
    bars = 0;
  }

  if (bars == lastWifiBars && tachado == lastWifiTachado) return;

  lastWifiBars    = bars;
  lastWifiTachado = tachado;

  tft.fillRect(WIFI_ICON_X, WIFI_ICON_Y, WIFI_ICON_W, WIFI_ICON_H, TFT_BLACK);

  if (bars == 0 && !tachado) {
    return;
  }

  int barWidth  = 3;
  int space     = 2;
  int baseLineY = WIFI_ICON_Y + WIFI_ICON_H - 2;

  uint16_t barColor = conectado ? TFT_GREEN : TFT_DARKGREY;

  for (int i = 0; i < bars; i++) {
    int barHeight = 4 + i * 3;
    int x = WIFI_ICON_X + i * (barWidth + space);
    int y = baseLineY - barHeight;
    tft.fillRect(x, y, barWidth, barHeight, barColor);
  }

  if (tachado) {
    int x0 = WIFI_ICON_X;
    int y0 = WIFI_ICON_Y + WIFI_ICON_H;
    int x1 = WIFI_ICON_X + WIFI_ICON_W;
    int y1 = WIFI_ICON_Y;
    tft.drawLine(x0, y0, x1, y1, TFT_RED);
  }
}

// ----------------- Indicador modo hora (WiFi/Manual) -----------------

bool timeModeLabelInvalidated = true;

void invalidateTimeModeLabel() {
  timeModeLabelInvalidated = true;
}

void drawTimeModeLabel() {
  static bool firstDraw           = true;
  static bool lastTimeModeUseAuto = false;

  if (!firstDraw &&
      !timeModeLabelInvalidated &&
      lastTimeModeUseAuto == useAutoTime) return;

  firstDraw           = false;
  lastTimeModeUseAuto = useAutoTime;
  timeModeLabelInvalidated = false;

  int x = 5;
  int y = 30;
  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillRect(0, 30, 70, 25, TFT_BLACK);
  tft.drawString(useAutoTime ? "WiFi" : "Manual", x, y + 10);
}

// ----------------- Dibujo común -----------------

void drawHeaderText(const char* txt) {
  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString(txt, 120, 15);
  drawWifiSignalIcon();
}

// ----------------- SPLASH -----------------

void drawSplashScreen() {
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(3);
  tft.drawString("LAMP_Fun", 120, 55);

  tft.setTextSize(2);
  tft.drawString("V.2.6.4", 120, 85);

  tft.setTextSize(1);
  tft.drawString("Inicializando...", 120, 110);

  int barX = 20;
  int barY = 160;
  int barW = 200;
  int barH = 16;
  tft.drawRect(barX, barY, barW, barH, TFT_WHITE);

  drawWifiSignalIcon();
}

void drawSplashStatusText(float ratio) {
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);

  const char* msg = "Inicializando...";
  if      (ratio < 0.20f) msg = "HW...";
  else if (ratio < 0.40f) msg = "Config...";
  else if (ratio < 0.70f) msg = "WiFi...";
  else if (ratio < 0.90f) msg = "NTP...";
  else                    msg = hasWifiCredentials ? "Listo." : "Configurar...";

  tft.fillRect(0, 120, 240, 20, TFT_BLACK);
  tft.drawString(msg, 120, 130);
  drawWifiSignalIcon();
}

void updateSplashProgress() {
  unsigned long elapsed = millis() - splashStartMillis;
  if (elapsed > SPLASH_DURATION) elapsed = SPLASH_DURATION;

  float ratio = (float)elapsed / (float)SPLASH_DURATION;

  int barX = 20;
  int barY = 160;
  int barW = 200;
  int barH = 16;

  int fillW = (int)(barW * ratio);
  int w     = fillW - 2;
  if (w < 0) w = 0;

  tft.fillRect(barX + 1, barY + 1, w, barH - 2, TFT_GREEN);

  int percent = (int)(ratio * 100.0f);
  if (percent > 100) percent = 100;

  char buf[8];
  snprintf(buf, sizeof(buf), "%3d%%", percent);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(buf, 120, barY + barH + 12);

  drawSplashStatusText(ratio);
}

// ----------------- RELOJ (digital + analógico) -----------------

int lastSecond = -1;
int lastMinute = -1;
int lastHour   = -1;
int lastDay    = -1;
int lastMonth  = -1;
int lastYear   = -1;

bool analogFaceDrawn      = false;
bool forceFullClockRedraw = false;

void drawClockScreenFull();
void updateClockScreen();

void resetClockDrawingState() {
  analogFaceDrawn = false;
  lastSecond = -1;
  lastMinute = -1;
  lastHour   = -1;
  lastDay    = -1;
  lastMonth  = -1;
  lastYear   = -1;
}

// --- Corrección manual de zona horaria ---

bool getAdjustedLocalTime(struct tm &out) {
  struct tm tRaw;
  if (!getLocalTime(&tRaw)) {
    return false;
  }

  time_t base   = mktime(&tRaw);
  long   offset = (long)tzOffsetSteps * 1800L;
  time_t adj    = base + offset;

  struct tm* ptm = localtime(&adj);
  if (!ptm) return false;
  out = *ptm;
  return true;
}

// Sol del brillo (parte inferior)

void drawSunIcon(int cx, int cy, uint16_t fillColor) {
  int r = 8;
  tft.fillCircle(cx, cy, r, fillColor);
  tft.drawCircle(cx, cy, r, TFT_WHITE);

  const int rays = 12;
  int innerLong  = r + 2;
  int outerLong  = r + 10;
  int outerShort = r + (int)(10 * 0.75f);

  for (int i = 0; i < rays; i++) {
    float angle = (2.0f * 3.1415926f * i) / rays;
    float adj   = angle;

    int inner = innerLong;
    int outer = (i % 2 == 0) ? outerLong : outerShort;

    int x0 = cx + (int)(inner * cosf(adj));
    int y0 = cy + (int)(inner * sinf(adj));
    int x1 = cx + (int)(outer * cosf(adj));
    int y1 = cy + (int)(outer * sinf(adj));

    tft.drawLine(x0, y0, x1, y1, TFT_WHITE);
  }
}

void drawBottomBrightnessLine() {
  int infoYTop = 204;
  int lineH    = 34;
  int centerY  = infoYTop + lineH / 2;

  tft.fillRect(0, infoYTop, 240, lineH, TFT_BLACK);

  uint16_t sunFillColor = tft.color565(redValue, greenValue, blueValue);

  const char* prefix = " - Brillo: ";
  char numBuf[8];
  snprintf(numBuf, sizeof(numBuf), "%d", brightness);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);

  int prefixWidth = tft.textWidth(prefix);
  int numWidth    = tft.textWidth(numBuf);

  int sunVisualW         = 36;
  const int spacingSolText  = 6;
  const int spacingPrefixNum= 0;

  int totalW = sunVisualW + spacingSolText + prefixWidth + spacingPrefixNum + numWidth;

  int startX = (tft.width() - totalW) / 2;
  if (startX < 0) startX = 0;

  int sunCenterX = startX + sunVisualW / 2;
  int sunCenterY = centerY - 2;
  drawSunIcon(sunCenterX, sunCenterY, sunFillColor);

  int textBaseX = startX + sunVisualW + spacingSolText;
  int textBaseY = centerY - 8;
  tft.drawString(prefix, textBaseX, textBaseY);
  int prefixEndX = textBaseX + prefixWidth;
  tft.drawString(numBuf, prefixEndX + spacingPrefixNum, textBaseY);
}

void drawClockScreenFull() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;
  resetClockDrawingState();
  drawHeaderText("LAMP_Fun");
  invalidateTimeModeLabel();
  drawTimeModeLabel();
  drawBottomBrightnessLine();
  forceFullClockRedraw = true;
  updateClockScreen();
}

// ---------- Reloj digital ----------

void drawDigitalClock(const struct tm& timeinfo) {
  unsigned long hours24 = timeinfo.tm_hour;
  unsigned long minutes = timeinfo.tm_min;

  drawTimeModeLabel();

  tft.setTextSize(1);
  tft.setTextDatum(TR_DATUM);

  char modeBuf[4];
  char ampm[3] = "24";

  if (use24hFormat) {
    strcpy(modeBuf, "24H");
  } else {
    if (hours24 == 0) {
      strcpy(ampm, "AM");
    } else if (hours24 < 12) {
      strcpy(ampm, "AM");
    } else if (hours24 == 12) {
      strcpy(ampm, "PM");
    } else {
      strcpy(ampm, "PM");
    }
    snprintf(modeBuf, sizeof(modeBuf), "%s", ampm);
  }

  int modeX = 235;
  int modeY = 40;
  tft.fillRect(190, 30, 50, 25, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(modeBuf, modeX, modeY);

  unsigned long displayHour = hours24;
  if (!use24hFormat) {
    if      (hours24 == 0)  displayHour = 12;
    else if (hours24 < 12)  displayHour = hours24;
    else if (hours24 == 12) displayHour = 12;
    else                    displayHour = hours24 - 12;
  }

  char bufHour[3];
  char bufMin[3];
  snprintf(bufHour, sizeof(bufHour), "%02lu", displayHour);
  snprintf(bufMin,  sizeof(bufMin),  "%02lu", minutes);

  tft.setTextSize(6);
  tft.setTextDatum(MC_DATUM);

  int yTime = 90;
  tft.fillRect(0, 60, 240, 60, TFT_BLACK);

  int wHour  = tft.textWidth(bufHour);
  int wSep   = tft.textWidth(":");
  int wMin   = tft.textWidth(bufMin);
  int totalW = wHour + wSep + wMin;
  int xStart = (240 - totalW) / 2;

  tft.setTextColor(digitalHMColor, TFT_BLACK);
  tft.drawString(bufHour, xStart + wHour / 2, yTime);
  tft.drawString(":",     xStart + wHour + wSep / 2, yTime);
  tft.drawString(bufMin,  xStart + wHour + wSep + wMin / 2, yTime);

  int day   = timeinfo.tm_mday;
  int month = timeinfo.tm_mon + 1;
  int year  = timeinfo.tm_year + 1900;

  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  char dateBuf[20];
  snprintf(dateBuf, sizeof(dateBuf), "%02d/%02d/%04d", day, month, year);
  tft.fillRect(0, 130, 240, 30, TFT_BLACK);
  tft.setTextColor(digitalDateColor, TFT_BLACK);
  tft.drawString(dateBuf, 120, 140);
}

// ---------- Reloj analógico ----------

const int ANALOG_CENTER_X = 120;
const int ANALOG_CENTER_Y = 120;
const int ANALOG_RADIUS   = 80;

void drawAnalogClockFace() {
  if (analogFaceDrawn) return;

  tft.fillRect(0, 60, 240, 240 - 60 - 34, TFT_BLACK);
  tft.fillCircle(ANALOG_CENTER_X, ANALOG_CENTER_Y,
                 ANALOG_RADIUS - 2, analogFaceFillColor);

  for (int d = 0; d < 2; d++) {
    tft.drawCircle(ANALOG_CENTER_X, ANALOG_CENTER_Y,
                   ANALOG_RADIUS - d, TFT_WHITE);
  }

  for (int i = 0; i < 60; i++) {
    float angle = (2.0f * 3.1415926f * i) / 60.0f;
    float adj   = angle - 3.1415926f / 2.0f;

    bool isMajorHour = (i == 0 || i == 15 || i == 30 || i == 45);
    bool isHour      = (!isMajorHour && (i % 5 == 0));
    bool isMinute    = (!isMajorHour && !isHour);

    int inner, outer;

    if (isMajorHour) {
      inner = ANALOG_RADIUS - 22;
      outer = ANALOG_RADIUS - 2;
    } else if (isHour) {
      inner = ANALOG_RADIUS - 14;
      outer = ANALOG_RADIUS - 4;
    } else {
      inner = ANALOG_RADIUS - 8;
      outer = ANALOG_RADIUS - 6;
    }

    int x0 = ANALOG_CENTER_X + (int)(inner * cosf(adj));
    int y0 = ANALOG_CENTER_Y + (int)(inner * sinf(adj));
    int x1 = ANALOG_CENTER_X + (int)(outer * cosf(adj));
    int y1 = ANALOG_CENTER_Y + (int)(outer * sinf(adj));

    if (isMajorHour) {
      tft.drawLine(x0, y0, x1, y1, TFT_WHITE);
      tft.drawLine(x0 + 1, y0, x1 + 1, y1, TFT_WHITE);
      tft.drawLine(x0 - 1, y0, x1 - 1, y1, TFT_WHITE);
    } else {
      tft.drawLine(x0, y0, x1, y1, TFT_WHITE);
    }
  }

  analogFaceDrawn = true;
}

void eraseAnalogHand(int len, float angle) {
  int x = ANALOG_CENTER_X + (int)(len * cosf(angle));
  int y = ANALOG_CENTER_Y + (int)(len * sinf(angle));
  tft.drawLine(ANALOG_CENTER_X, ANALOG_CENTER_Y, x, y, analogFaceFillColor);
}

void drawAnalogHands(const struct tm& t, bool erasePrev) {
  static int   prevSecLen   = ANALOG_RADIUS - 18;
  static int   prevMinLen   = ANALOG_RADIUS - 18;
  static int   prevHourLen  = ANALOG_RADIUS - 40;
  static float prevSecAngle = 0;
  static float prevMinAngle = 0;
  static float prevHourAngle= 0;

  float sec  = t.tm_sec;
  float min  = t.tm_min + sec / 60.0f;
  float hour = (t.tm_hour % 12) + min / 60.0f;

  float angSec  = (sec  / 60.0f) * 2.0f * 3.1415926f - 3.1415926f / 2.0f;
  float angMin  = (min  / 60.0f) * 2.0f * 3.1415926f - 3.1415926f / 2.0f;
  float angHour = (hour / 12.0f) * 2.0f * 3.1415926f - 3.1415926f / 2.0f;

  int lenHour = ANALOG_RADIUS - 40;
  int lenMin  = ANALOG_RADIUS - 18;
  int lenSec  = ANALOG_RADIUS - 18;

  if (erasePrev) {
    eraseAnalogHand(prevSecLen,  prevSecAngle);
    eraseAnalogHand(prevMinLen,  prevMinAngle);
    eraseAnalogHand(prevHourLen, prevHourAngle);
  }

  int xh = ANALOG_CENTER_X + (int)(lenHour * cosf(angHour));
  int yh = ANALOG_CENTER_Y + (int)(lenHour * sinf(angHour));
  int xm = ANALOG_CENTER_X + (int)(lenMin  * cosf(angMin));
  int ym = ANALOG_CENTER_Y + (int)(lenMin  * sinf(angMin));
  int xs = ANALOG_CENTER_X + (int)(lenSec  * cosf(angSec));
  int ys = ANALOG_CENTER_Y + (int)(lenSec  * sinf(angSec));

  tft.drawLine(ANALOG_CENTER_X, ANALOG_CENTER_Y, xh, yh, analogHourHandColor);
  tft.drawLine(ANALOG_CENTER_X, ANALOG_CENTER_Y, xm, ym, analogMinHandColor);
  tft.drawLine(ANALOG_CENTER_X, ANALOG_CENTER_Y, xs, ys, analogSecHandColor);

  tft.fillCircle(ANALOG_CENTER_X, ANALOG_CENTER_Y, 3, TFT_WHITE);

  prevSecAngle  = angSec;
  prevMinAngle  = angMin;
  prevHourAngle = angHour;

  prevSecLen  = lenSec;
  prevMinLen  = lenMin;
  prevHourLen = lenHour;
}

// -------- Nueva fecha lateral para analógico --------

void drawAnalogSideDate(const struct tm& t, bool forceRedraw) {
  static int prevDay   = -1;
  static int prevMonth = -1;
  static int prevYear  = -1;

  int day   = t.tm_mday;
  int month = t.tm_mon + 1;
  int year  = t.tm_year + 1900;

  if (!forceRedraw &&
      day == prevDay && month == prevMonth && year == prevYear) return;

  tft.setTextColor(analogDateColor, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);

  // Día
  int xDay = 4;
  int yDay = 68; // bajo "WiFi/Manual"
  int wDay = 40;
  int hDay = 20;
  tft.fillRect(xDay, yDay, wDay, hDay, TFT_BLACK);
  tft.setTextSize(2);
  char dayBuf[4];
  snprintf(dayBuf, sizeof(dayBuf), "%02d", day);
  tft.drawString(dayBuf, xDay, yDay);

  // Mes
  int yMonthStart = yDay + hDay + 20;

  const char* fullName = MONTH_NAMES_ES[month - 1];
  char monthAbbr[4] = {0,0,0,0};
  for (int i = 0; i < 3; i++) {
    char c = fullName[i];
    if (c == 0) { monthAbbr[i] = 0; break; }
    monthAbbr[i] = c;
  }
  if (monthAbbr[0] >= 'a' && monthAbbr[0] <= 'z')
    monthAbbr[0] = monthAbbr[0] - 'a' + 'A';
  for (int i = 1; i < 3; i++) {
    if (monthAbbr[i] >= 'A' && monthAbbr[i] <= 'Z')
      monthAbbr[i] = monthAbbr[i] - 'A' + 'a';
  }

  int xMonth = 4;
  int wMonth = 20;
  int lineH  = 16;
  int hMonth = lineH * 3;
  tft.fillRect(xMonth, yMonthStart, wMonth, hMonth, TFT_BLACK);
  tft.setTextSize(2);
  int yM = yMonthStart;
  for (int i = 0; i < 3 && monthAbbr[i] != 0; i++) {
    char s[2] = { monthAbbr[i], 0 };
    tft.drawString(s, xMonth, yM);
    yM += lineH;
  }

  // Año
  int yYear = yMonthStart + hMonth + 20;
  int xYear = 4;
  int wYear = 60;
  int hYear = 20;
  tft.fillRect(xYear, yYear, wYear, hYear, TFT_BLACK);
  tft.setTextSize(2);
  char yearBuf[6];
  snprintf(yearBuf, sizeof(yearBuf), "%04d", year);
  tft.drawString(yearBuf, xYear, yYear);

  prevDay   = day;
  prevMonth = month;
  prevYear  = year;
}

void updateAnalogClock(const struct tm& t) {
  drawTimeModeLabel();
  drawAnalogClockFace();

  if (lastSecond < 0) {
    drawAnalogHands(t, false);
  } else {
    bool secChanged  = (t.tm_sec  != lastSecond);
    bool minChanged  = (t.tm_min  != lastMinute);
    bool hourChanged = (t.tm_hour != lastHour);

    if (secChanged || minChanged || hourChanged) {
      drawAnalogHands(t, true);
    }
  }

  lastSecond = t.tm_sec;
  lastMinute = t.tm_min;
  lastHour   = t.tm_hour;
  lastDay    = t.tm_mday;
  lastMonth  = t.tm_mon;
  lastYear   = t.tm_year;
}

void updateClockScreen() {
  struct tm timeinfo;
  if (!getAdjustedLocalTime(timeinfo)) {
    tft.setTextSize(6);
    tft.setTextDatum(MC_DATUM);
    tft.fillRect(0, 60, 240, 60, TFT_BLACK);
    tft.setTextColor(digitalHMColor, TFT_BLACK);
    tft.drawString("--:--", 120, 90);

    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.fillRect(0, 130, 240, 30, TFT_BLACK);
    tft.setTextColor(digitalDateColor, TFT_BLACK);
    tft.drawString("--/--/----", 120, 140);

    drawWifiSignalIcon();
    drawTimeModeLabel();
    return;
  }

  if (clockMode == 0) {
    static int lastMinuteDigital = -1;
    if (forceFullClockRedraw) {
      lastMinuteDigital   = -1;
      forceFullClockRedraw= false;
    }

    if (timeinfo.tm_min != lastMinuteDigital || lastMinuteDigital < 0) {
      drawDigitalClock(timeinfo);
      lastMinuteDigital = timeinfo.tm_min;
    } else {
      drawTimeModeLabel();
    }
  } else {
    bool dayChanged = (timeinfo.tm_mday != lastDay ||
                       timeinfo.tm_mon  != lastMonth ||
                       timeinfo.tm_year != lastYear);
    updateAnalogClock(timeinfo);
    drawAnalogSideDate(timeinfo, dayChanged || forceFullClockRedraw);
    forceFullClockRedraw = false;
  }

  drawWifiSignalIcon();
}

// ----------------- CONTROL LUZ -----------------

enum ControlIndex {
  CTRL_BTN_POWER = 0,
  CTRL_BAR_R,
  CTRL_BAR_G,
  CTRL_BAR_B,
  CTRL_BAR_BRIGHT,
  CTRL_COUNT
};

void drawVerticalBar(const char* label, uint8_t value,
                     int x, int y, int w, int h,
                     bool selected, uint16_t color) {
  uint16_t bg = TFT_BLACK;
  tft.fillRect(x, y, w, h, bg);

  int innerX = x + 1;
  int innerY = y + 1;
  int innerW = w - 2;
  int innerH = h - 2;

  int trackW = innerW / 3;
  if (trackW < 3) trackW = 3;
  if (trackW > innerW - 2) trackW = innerW - 2;
  int trackX = innerX + (innerW - trackW) / 2;

  int marginTop    = 14;
  int marginBottom = 16;

  int usableH = innerH - (marginTop + marginBottom);
  if (usableH < 0) usableH = 0;

  int yBottom = innerY + innerH - marginBottom;
  int yTop    = innerY + marginTop;

  int sliderH = (usableH > 0) ? (int)((value / 255.0f) * usableH) : 0;
  int knobY   = yBottom - sliderH;
  if (knobY < yTop)    knobY = yTop;
  if (knobY > yBottom) knobY = yBottom;

  int trackH = yBottom - knobY;
  if (trackH < 0) trackH = 0;
  tft.fillRect(trackX, knobY, trackW, trackH, color);

  int knobRadius = innerW / 2;
  if (knobRadius < 4) knobRadius = 4;
  if (knobRadius > 8) knobRadius = 8;
  int knobCenterX = trackX + trackW / 2;

  tft.fillCircle(knobCenterX, knobY, knobRadius, color);
  tft.drawCircle(knobCenterX, knobY, knobRadius, TFT_WHITE);

  if (selected) {
    tft.drawCircle(knobCenterX, knobY, knobRadius + 3, TFT_WHITE);
  }

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  char buf[8];
  snprintf(buf, sizeof(buf), "%3d", value);
  int valueY = innerY - 8;
  if (valueY < 0) valueY = 0;
  tft.drawString(buf, x + w / 2, valueY);

  int labelY = innerY + innerH + 10;
  if (labelY > 239) labelY = 239;
  tft.drawString(label, x + w / 2, labelY);
}

void drawButtonPower(bool selected) {
  int w = 90;
  int h = 32;
  int x = (240 - w) / 2;       // centrado horizontal
  int y = 240 - h - 8;

  uint16_t bg = lampOn ? TFT_NAVY : TFT_RED;
  uint16_t fg = TFT_WHITE;

  tft.fillRect(x, y, w, h, bg);

  if (selected) {
    tft.drawRect(x + 1, y + 1, w - 2, h - 2, TFT_WHITE);
  }

  tft.setTextColor(fg, bg);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);

  tft.drawString(lampOn ? "ON" : "OFF", x + w / 2, y + h / 2);
}

void drawLightScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  drawHeaderText("Luz");

  int barW = 30;
  int barH = 110;
  int barY = 60;

  int xR   = 25;
  int xG   = xR + barW + 16;
  int xB   = xG + barW + 16;
  int xBri = 240 - 25 - barW;

  drawVerticalBar("R",      redValue,   xR,   barY, barW, barH, currentControl == CTRL_BAR_R,      TFT_RED);
  drawVerticalBar("G",      greenValue, xG,   barY, barW, barH, currentControl == CTRL_BAR_G,      TFT_GREEN);
  drawVerticalBar("B",      blueValue,  xB,   barY, barW, barH, currentControl == CTRL_BAR_B,      TFT_BLUE);
  drawVerticalBar("Brillo", brightness, xBri, barY, barW, barH, currentControl == CTRL_BAR_BRIGHT, TFT_WHITE);

  drawButtonPower(currentControl == CTRL_BTN_POWER);

  drawWifiSignalIcon();
}

void redrawLightControls() {
  int barW = 30;
  int barH = 110;
  int barY = 60;

  int xR   = 25;
  int xG   = xR + barW + 16;
  int xB   = xG + barW + 16;
  int xBri = 240 - 25 - barW;

  drawVerticalBar("R",      redValue,   xR,   barY, barW, barH, currentControl == CTRL_BAR_R,      TFT_RED);
  drawVerticalBar("G",      greenValue, xG,   barY, barW, barH, currentControl == CTRL_BAR_G,      TFT_GREEN);
  drawVerticalBar("B",      blueValue,  xB,   barY, barW, barH, currentControl == CTRL_BAR_B,      TFT_BLUE);
  drawVerticalBar("Brillo", brightness, xBri, barY, barW, barH, currentControl == CTRL_BAR_BRIGHT, TFT_WHITE);

  drawButtonPower(currentControl == CTRL_BTN_POWER);

  drawWifiSignalIcon();
}

// ----------------- AJUSTES -----------------

int settingsClockIndex        = 0;
const int SETTINGS_CLOCK_ITEMS= 6;

int settingsFormatIndex   = 0;
int settingsDateModeIndex = 0;

int settingsMainIndex       = 0;
const int SETTINGS_MAIN_ITEMS= 7;

int settingsEffectsIndex = 0;
const int SETTINGS_EFFECTS_ITEMS = 4;

int settingsColorDigitalIndex = 0;
int settingsColorAnalogIndex  = 0;

int settingsTzIndexTemp  = 0;
int settingsTzOffsetTemp = 0;

int resetConfirmIndex = 1; // 0 = SI, 1 = NO

enum DateTimeField {
  FIELD_HOUR = 0,
  FIELD_MIN,
  FIELD_DAY,
  FIELD_MONTH,
  FIELD_YEAR,
  FIELD_DONE
};

DateTimeField editField = FIELD_HOUR;
int editHour   = 0;
int editMinute = 0;
int editDay    = 1;
int editMonth  = 1;
int editYear   = 2026;

// ---------- Menú Ajustes reloj ----------

void drawSettingsClockScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Ajustes reloj", 120, 15);

  drawWifiSignalIcon();

  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);

  const char* line1 = use24hFormat ? "Formato: (24H)" : "Formato: (AM/PM)";

  static char line2[24];
  snprintf(line2, sizeof(line2),
           "Fecha/hora: (%c)", useAutoTime ? 'A' : 'M');

  static char line3[32];
  snprintf(line3, sizeof(line3),
           "Tipo: (%s)", (clockMode == 0) ? "DIGITAL" : "ANALOGICO");

  static char line4[32];
  snprintf(line4, sizeof(line4),
           "Zona: %s", TIMEZONES[tzIndex].name);

  static char line5[32];
  float offH = tzOffsetSteps * 0.5f;
  char  sign = (offH > 0.0f) ? '+' : (offH < 0.0f ? '-' : ' ');
  float mag  = fabs(offH);
  snprintf(line5, sizeof(line5), "Ajuste: %c%.1fh", sign, mag);

  const char* line6 = "Colores reloj";

  const char* lines[SETTINGS_CLOCK_ITEMS] = {
    line1, line2, line3, line4, line5, line6
  };

  int startY = 60;
  int lineH  = 24;

  for (int i = 0; i < SETTINGS_CLOCK_ITEMS; i++) {
    int  y        = startY + i * lineH;
    bool selected = (i == settingsClockIndex);

    if (selected) {
      tft.fillRect(10, y - 2, 220, lineH, TFT_DARKGREY);
      tft.setTextColor(TFT_NAVY, TFT_DARKGREY);
    } else {
      tft.fillRect(10, y - 2, 220, lineH, TFT_BLACK);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    tft.drawString(lines[i], 14, y);
  }
}

// ---------- Menú principal Ajustes ----------

void drawSettingsMainScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Ajustes", 120, 15);

  drawWifiSignalIcon();

  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);

  const char* lines[SETTINGS_MAIN_ITEMS] = {
    "Ajustes reloj",
    "Backlight TFT",
    "Efectos",
    "WiFi",
    "Reinicio HW",
    "Acerca de...",
    "Salir"
  };

  int startY = 60;
  int lineH  = 24;

  for (int i = 0; i < SETTINGS_MAIN_ITEMS; i++) {
    int  y        = startY + i * lineH;
    bool selected = (i == settingsMainIndex);

    if (selected) {
      tft.fillRect(10, y - 2, 220, lineH, TFT_DARKGREY);
      tft.setTextColor(TFT_NAVY, TFT_DARKGREY);
    } else {
      tft.fillRect(10, y - 2, 220, lineH, TFT_BLACK);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    tft.drawString(lines[i], 14, y);
  }
}

// ---------- Lista de efectos ----------

void drawSettingsEffectsScreen() {
  tft.fillScreen(TFT_BLACK);

  drawHeaderText("Efectos");

  const int EFFECTS_ITEMS = 4;
  const char* lines[EFFECTS_ITEMS] = {
    "RESPIRACION",
    "COMETA",
    "BARRIDO",
    "PERSIANA"
  };

  tft.setTextSize(2);
  tft.setTextDatum(ML_DATUM);

  int startY = 55;   // un poco más abajo
  int lineH  = 24;   // altura de cada renglón, sin dejar “fila vacía”

  for (int i = 0; i < EFFECTS_ITEMS; i++) {
    int y = startY + i * lineH;

    bool selected = (i == settingsEffectsIndex);

    uint16_t bg   = selected ? TFT_DARKGREY : TFT_BLACK;
    uint16_t fg   = selected ? TFT_BLUE     : TFT_WHITE;

    tft.fillRect(0, y - 10, 240, lineH, bg);
    tft.setTextColor(fg, bg);
    tft.drawString(lines[i], 20, y);
  }
}

// ---------- Formato hora ----------

void drawSettingsFormatScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Formato hora", 120, 15);

  drawWifiSignalIcon();

  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);

  const char* lines[2] = { "24H", "AM/PM" };

  int startY = 60;
  int lineH  = 30;

  for (int i = 0; i < 2; i++) {
    int  y        = startY + i * lineH;
    bool selected = (i == settingsFormatIndex);

    if (selected) {
      tft.fillRect(10, y - 2, 220, lineH, TFT_DARKGREY);
      tft.setTextColor(TFT_NAVY, TFT_DARKGREY);
    } else {
      tft.fillRect(10, y - 2, 220, lineH, TFT_BLACK);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    tft.drawString(lines[i], 14, y);
  }
}

// ---------- Modo fecha/hora ----------

void drawSettingsDateModeScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Fecha/hora", 120, 15);

  drawWifiSignalIcon();

  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);

  const char* lines[2] = { "Auto (WiFi)", "Manual" };

  int startY = 60;
  int lineH  = 30;

  for (int i = 0; i < 2; i++) {
    int  y        = startY + i * lineH;
    bool selected = (i == settingsDateModeIndex);

    if (selected) {
      tft.fillRect(10, y - 2, 220, lineH, TFT_DARKGREY);
      tft.setTextColor(TFT_NAVY, TFT_DARKGREY);
    } else {
      tft.fillRect(10, y - 2, 220, lineH, TFT_BLACK);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    tft.drawString(lines[i], 14, y);
  }
}

// ---------- Zona horaria ----------

void drawSettingsTimezoneScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Zona horaria", 120, 15);

  drawWifiSignalIcon();

  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);

  int startY = 60;
  int lineH  = 22;

  for (int i = 0; i < TIMEZONES_COUNT; i++) {
    int y = startY + i * lineH;
    if (y > 210) break;

    bool selected = (i == settingsTzIndexTemp);

    if (selected) {
      tft.fillRect(10, y - 2, 220, lineH, TFT_DARKGREY);
      tft.setTextColor(TFT_NAVY, TFT_DARKGREY);
    } else {
      tft.fillRect(10, y - 2, 220, lineH, TFT_BLACK);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    tft.drawString(TIMEZONES[i].name, 14, y);
  }
}

// ---------- Ajuste hora (offset manual) ----------

void drawSettingsTzOffsetScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Ajuste hora", 120, 15);

  drawWifiSignalIcon();

  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  float offH = settingsTzOffsetTemp * 0.5f;
  char  sign = (offH > 0.0f) ? '+' : (offH < 0.0f ? '-' : ' ');
  float mag  = fabs(offH);

  char buf[16];
  snprintf(buf, sizeof(buf), "%c%.1fh", sign, mag);
  tft.drawString(buf, 120, 120);

  tft.setTextSize(1);
  tft.drawString("(-2.0h .. +2.0h, pasos 0.5h)", 120, 170);
}

// ---------- Edición manual fecha/hora ----------

void drawEditDateTimeField() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Hora manual", 120, 15);

  drawWifiSignalIcon();

  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  char buf[16];

  switch (editField) {
    case FIELD_HOUR:
      snprintf(buf, sizeof(buf), "HH: %02d", editHour);
      break;
    case FIELD_MIN:
      snprintf(buf, sizeof(buf), "MM: %02d", editMinute);
      break;
    case FIELD_DAY:
      snprintf(buf, sizeof(buf), "DD: %02d", editDay);
      break;
    case FIELD_MONTH:
      snprintf(buf, sizeof(buf), "Mes: %02d", editMonth);
      break;
    case FIELD_YEAR:
      snprintf(buf, sizeof(buf), "AA: %04d", editYear);
      break;
    default:
      snprintf(buf, sizeof(buf), "Fin");
      break;
  }

  tft.drawString(buf, 120, 140);
}

void applyManualDateTime() {
  struct tm tmset;
  memset(&tmset, 0, sizeof(tmset));

  tmset.tm_year = editYear - 1900;
  tmset.tm_mon  = editMonth - 1;
  tmset.tm_mday = editDay;
  tmset.tm_hour = editHour;
  tmset.tm_min  = editMinute;
  tmset.tm_sec  = 0;

  time_t t = mktime(&tmset);

  struct timeval tv;
  tv.tv_sec  = t;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
}

void initManualTime_2026() {
  struct tm tmset;
  memset(&tmset, 0, sizeof(tmset));

  tmset.tm_year = 2026 - 1900;
  tmset.tm_mon  = 0;
  tmset.tm_mday = 1;
  tmset.tm_hour = 0;
  tmset.tm_min  = 0;
  tmset.tm_sec  = 0;

  time_t t = mktime(&tmset);

  struct timeval tv;
  tv.tv_sec  = t;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
}

// ---------- Colores (slider HSV+N/B) ----------

uint8_t colorPos = 0;

uint16_t rgbTo565(uint8_t r, uint8_t g, uint8_t b) {
  return tft.color565(r, g, b);
}

void rgbFrom565(uint16_t c, uint8_t &r, uint8_t &g, uint8_t &b) {
  r = ((c >> 11) & 0x1F) * 255 / 31;
  g = ((c >> 5)  & 0x3F) * 255 / 63;
  b = ( c        & 0x1F) * 255 / 31;
}

void hsvToRgb240(float hDeg, uint8_t &r, uint8_t &g, uint8_t &b) {
  float h = hDeg / 60.0f;
  int   i = (int)floorf(h);
  float f = h - i;
  float q = 1.0f - f;
  float t = f;

  float rf = 0, gf = 0, bf = 0;
  switch (i) {
    case 0: rf = 1; gf = t; bf = 0; break;
    case 1: rf = q; gf = 1; bf = 0; break;
    case 2: rf = 0; gf = 1; bf = t; break;
    case 3: rf = 0; gf = q; bf = 1; break;
    default: rf = 0; gf = 0; bf = 1; break;
  }

  r = (uint8_t)(rf * 255);
  g = (uint8_t)(gf * 255);
  b = (uint8_t)(bf * 255);
}

uint16_t colorFromSlider(uint8_t pos, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (pos == 0) {
    r = g = b = 0;
    return rgbTo565(0,0,0);
  }
  if (pos == 255) {
    r = g = b = 255;
    return rgbTo565(255,255,255);
  }
  float h = (pos / 254.0f) * 240.0f;
  hsvToRgb240(h, r, g, b);
  return rgbTo565(r, g, b);
}

// Degradado para efectos: negro -> rojo -> verde -> azul -> blanco
uint16_t colorFromSliderEffects(uint8_t pos, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (pos == 0) {
    r = g = b = 0;
    return rgbTo565(0, 0, 0);   // negro
  }

  if (pos == 255) {
    r = g = b = 255;
    return rgbTo565(255, 255, 255);  // blanco
  }

  // Mapeo lineal en 4 tramos:
  // 0..64: negro -> rojo
  // 64..128: rojo -> verde
  // 128..192: verde -> azul
  // 192..255: azul -> blanco
  if (pos <= 64) {
    // Negro (0) -> Rojo (255,0,0)
    float t = pos / 64.0f;
    r = (uint8_t)(255.0f * t);
    g = 0;
    b = 0;
  } else if (pos <= 128) {
    // Rojo (255,0,0) -> Verde (0,255,0)
    float t = (pos - 64) / 64.0f;
    r = (uint8_t)(255.0f * (1.0f - t));
    g = (uint8_t)(255.0f * t);
    b = 0;
  } else if (pos <= 192) {
    // Verde (0,255,0) -> Azul (0,0,255)
    float t = (pos - 128) / 64.0f;
    r = 0;
    g = (uint8_t)(255.0f * (1.0f - t));
    b = (uint8_t)(255.0f * t);
  } else {
    // Azul (0,0,255) -> Blanco (255,255,255)
    // Queremos que en la penúltima posición de RESPIRACION (211) siga habiendo azul puro.
    // A partir de 212 empezamos a mezclar hacia blanco.
    uint8_t startPos = 211;  // azul puro
    uint8_t endPos   = 255;  // blanco
    if (pos <= startPos) {
      r = 0;
      g = 0;
      b = 255;
    } else {
      float t = (pos - startPos) / float(endPos - startPos); // 0..1 entre 211 y 255
      if (t < 0.0f) t = 0.0f;
      if (t > 1.0f) t = 1.0f;
      r = (uint8_t)(255.0f * t);
      g = (uint8_t)(255.0f * t);
      b = 255;
    }
  }

  return rgbTo565(r, g, b);
}

// Inverso aproximado para el degradado de efectos
uint8_t sliderPosFromColorEffects(uint16_t c) {
  uint8_t r, g, b;
  rgbFrom565(c, r, g, b);

  // Negro casi puro
  if (r < 8 && g < 8 && b < 8) return 0;

  // Blanco casi puro
  if (r > 247 && g > 247 && b > 247) return 255;

  // Normalizamos a float 0..1
  float fr = r / 255.0f;
  float fg = g / 255.0f;
  float fb = b / 255.0f;

  // Determinar tramo aproximado según componente dominante
  // Tramo rojo: negro->rojo (0..64) y rojo->verde (64..128)
  if (fr >= fg && fr >= fb) {
    if (fg < 0.1f && fb < 0.1f) {
      // Negro -> rojo
      float t = fr; // 0..1
      uint8_t pos = (uint8_t)roundf(t * 64.0f);
      if (pos > 64) pos = 64;
      return pos;
    } else {
      // Rojo -> verde
      float t = fg; // 0..1
      uint8_t pos = 64 + (uint8_t)roundf(t * 64.0f);
      if (pos < 64) pos = 64;
      if (pos > 128) pos = 128;
      return pos;
    }
  }

  // Tramo verde: rojo->verde (64..128) y verde->azul (128..192)
  if (fg >= fr && fg >= fb) {
    if (fr > fb) {
      // Rojo -> verde
      float t = 1.0f - (fr / (fr + fg + 0.0001f)); // más verde que rojo
      uint8_t pos = 64 + (uint8_t)roundf(t * 64.0f);
      if (pos < 64) pos = 64;
      if (pos > 128) pos = 128;
      return pos;
    } else {
      // Verde -> azul
      float t = fb; // 0..1
      uint8_t pos = 128 + (uint8_t)roundf(t * 64.0f);
      if (pos < 128) pos = 128;
      if (pos > 192) pos = 192;
      return pos;
    }
  }

  // Tramo azul: verde->azul (128..192) y azul->blanco (192..255)
  if (fb >= fr && fb >= fg) {
    if (fr < 0.1f && fg < 0.1f) {
      // Verde -> azul (casi pure blue)
      float t = fb; // 0..1
      uint8_t pos = 128 + (uint8_t)roundf(t * 64.0f);
      if (pos < 128) pos = 128;
      if (pos > 211) pos = 211; // azul puro en 211
      return pos;
    } else {
      // Azul -> blanco
      // Medimos cuánto blanco hay: min(r,g) respecto a b
      float w = (fr + fg) * 0.5f; // 0..1
      float t = w; // 0..1 => 211..255
      uint8_t pos = 211 + (uint8_t)roundf(t * (255 - 211));
      if (pos < 211) pos = 211;
      if (pos > 255) pos = 255;
      return pos;
    }
  }

  // Fallback
  return 0;
}

uint8_t sliderPosFromColor(uint16_t c) {
  uint8_t r,g,b;
  rgbFrom565(c, r, g, b);
  uint8_t maxc = max(r, max(g,b));
  uint8_t minc = min(r, min(g,b));

  if (maxc < 8)   return 0;
  if (minc > 247) return 255;

  float fr = r / 255.0f;
  float fg = g / 255.0f;
  float fb = b / 255.0f;
  float maxf = max(fr, max(fg,fb));
  float minf = min(fr, min(fg,fb));
  float delta = maxf - minf;
  if (delta < 0.001f) return 1;

  float h = 0.0f;
  if (maxf == fr) {
    h = 60.0f * fmodf(((fg - fb) / delta), 6.0f);
  } else if (maxf == fg) {
    h = 60.0f * (((fb - fr) / delta) + 2.0f);
  } else {
    h = 60.0f * (((fr - fg) / delta) + 4.0f);
  }

  if (h < 0)      h += 360.0f;
  if (h > 240.0f) h  = 240.0f;

  uint8_t pos = (uint8_t)roundf((h / 240.0f) * 254.0f);
  if (pos < 1)   pos = 1;
  if (pos > 254) pos = 254;
  return pos;
}

// Convertir RGB (0..255) a Hue (0..240) y Value (0..1) aproximados
void rgbToHueValue240(uint8_t r, uint8_t g, uint8_t b, float &h240, float &v) {
  float fr = r / 255.0f;
  float fg = g / 255.0f;
  float fb = b / 255.0f;

  float maxf = max(fr, max(fg, fb));
  float minf = min(fr, min(fg, fb));
  float delta = maxf - minf;

  v = maxf;  // valor 0..1

  if (delta < 0.001f) {
    // Gris/negro/blanco: tono indefinido, ponemos 0
    h240 = 0.0f;
    return;
  }

  float hDeg;
  if (maxf == fr) {
    hDeg = 60.0f * fmodf(((fg - fb) / delta), 6.0f);
  } else if (maxf == fg) {
    hDeg = 60.0f * (((fb - fr) / delta) + 2.0f);
  } else {
    hDeg = 60.0f * (((fr - fg) / delta) + 4.0f);
  }

  if (hDeg < 0.0f) hDeg += 360.0f;
  if (hDeg > 240.0f) hDeg = 240.0f;  // recortamos a 0..240

  h240 = hDeg;  // tono en grados 0..240 para usar con hsvToRgb240
}

void initRespSliderPositions() {
  respKnobStartPos = sliderPosFromColorEffects(respColorStart);
  respKnobEndPos   = sliderPosFromColorEffects(respColorEnd);

  if (respKnobStartPos < 0)   respKnobStartPos = 0;
  if (respKnobStartPos > 211) respKnobStartPos = 211;

  if (respKnobEndPos < 0)   respKnobEndPos = 0;
  if (respKnobEndPos > 211) respKnobEndPos = 211;
}

// Inicializar posiciones de sliders/knobs COMETA a partir de colores guardados
void initCometSliderPositions() {
  cometKnobStartPos = sliderPosFromColorEffects(cometColorStart);
  cometKnobEndPos   = sliderPosFromColorEffects(cometColorEnd);

  if (cometKnobStartPos < 0)   cometKnobStartPos = 0;
  if (cometKnobStartPos > 211) cometKnobStartPos = 211;

  if (cometKnobEndPos < 0)   cometKnobEndPos = 0;
  if (cometKnobEndPos > 211) cometKnobEndPos = 211;
}

// Inicializar posiciones de sliders/knobs BARRIDO a partir de colores guardados
void initBarridoSliderPositions() {
  barridoKnobStartPos = sliderPosFromColorEffects(barridoColorStart);
  barridoKnobEndPos   = sliderPosFromColorEffects(barridoColorEnd);

  if (barridoKnobStartPos < 0)   barridoKnobStartPos = 0;
  if (barridoKnobStartPos > 211) barridoKnobStartPos = 211;

  if (barridoKnobEndPos < 0)   barridoKnobEndPos = 0;
  if (barridoKnobEndPos > 211) barridoKnobEndPos = 211;
}

// Inicializar posiciones de sliders/knobs PERSIANA a partir de colores guardados
void initPersianaSliderPositions() {
  persianaKnobStartPos = sliderPosFromColorEffects(persianaColorStart);
  persianaKnobEndPos   = sliderPosFromColorEffects(persianaColorEnd);

  if (persianaKnobStartPos < 0)   persianaKnobStartPos = 0;
  if (persianaKnobStartPos > 211) persianaKnobStartPos = 211;

  if (persianaKnobEndPos < 0)   persianaKnobEndPos = 0;
  if (persianaKnobEndPos > 211) persianaKnobEndPos = 211;
}

void drawColorSliderScreen(const char* title, const char* label, uint8_t sliderPos) {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  // Cabecera
  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(title, 120, 15);

  drawWifiSignalIcon();

  // Texto "Horas / Minutos / Fecha / ..." centrado
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(label, 120, 70);

  // Zona del slider
  int sx = 20;
  int sy = 110;
  int sw = 200;
  int sh = 20;

  // Fondo del slider
  tft.drawRect(sx, sy, sw, sh, TFT_WHITE);
  tft.fillRect(sx + 1, sy + 1, sw - 2, sh - 2, TFT_BLACK);

  // Paleta completa
  for (int x = 0; x < sw; x++) {
    float    ratio = (float)x / (float)(sw - 1);
    uint8_t  pos   = (uint8_t)roundf(ratio * 255.0f);
    uint8_t  r,g,b;
    uint16_t c     = colorFromSlider(pos, r, g, b);
    tft.drawFastVLine(sx + x, sy + 1, sh - 2, c);
  }

  // Knob
  int knobX = sx + 1 + (int)((sliderPos / 255.0f) * (sw - 4));
  if (knobX < sx + 1)       knobX = sx + 1;
  if (knobX > sx + sw - 3)  knobX = sx + sw - 3;

  // Color actual
  uint8_t  R,G,B;
  uint16_t currentColor = colorFromSlider(sliderPos, R, G, B);

  // Bolita encima
  int ballY = sy - 14;
  int ballR = 8;
  tft.fillCircle(knobX, ballY, ballR, currentColor);
  tft.drawCircle(knobX, ballY, ballR, TFT_WHITE);

  // Barra fina
  tft.drawLine(knobX, sy + 1, knobX, sy + sh - 2, TFT_WHITE);

  // Marcas N/B
  int tickHeight = 20;
  int tickYTop   = sy - 4;

  int leftTickX  = sx;
  int rightTickX = sx + sw - 1;

  tft.drawLine(leftTickX,  tickYTop, leftTickX,  tickYTop - tickHeight, TFT_WHITE);
  tft.drawLine(rightTickX, tickYTop, rightTickX, tickYTop - tickHeight, TFT_WHITE);

  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  int labelY = tickYTop - tickHeight - 6;

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("N", leftTickX,  labelY);
  tft.drawString("B", rightTickX, labelY);

  // Info RGB
  char infoBuf[40];
  snprintf(infoBuf, sizeof(infoBuf), "R:%u G:%u B:%u", R, G, B);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(infoBuf, 120, sy + sh + 20);
}

void drawSettingsColorsDigitalScreen() {
  const char* label;
  switch (settingsColorDigitalIndex) {
    case 0: label = "Hora/min"; break;
    case 1: label = "Fecha";    break;
    default: label = "";        break;
  }
  drawColorSliderScreen("Digital", label, colorPos);
}

void drawSettingsColorsAnalogScreen() {
  const char* label;
  switch (settingsColorAnalogIndex) {
    case 0: label = "Horas";    break;
    case 1: label = "Minutos";  break;
    case 2: label = "Segundos"; break;
    case 3: label = "Fecha";    break;
    case 4: label = "Fondo";    break;
    default: label = "";        break;
  }
  drawColorSliderScreen("Analogico", label, colorPos);
}

// ---------- Backlight TFT ----------

void drawSettingsBacklightScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Backlight TFT", 120, 15);

  drawWifiSignalIcon();

  int barW = 40;
  int barH = 140;
  int x    = (240 - barW) / 2;
  int y    = 50;

  tft.fillRect(x, y, barW, barH, TFT_BLACK);

  int innerX = x + 1;
  int innerY = y + 1;
  int innerW = barW - 2;
  int innerH = barH - 2;

  int trackW = innerW / 3;
  if (trackW < 3) trackW = 3;
  if (trackW > innerW - 2) trackW = innerW - 2;
  int trackX = innerX + (innerW - trackW) / 2;

  int marginTop    = 10;
  int marginBottom = 20;

  int usableH = innerH - (marginTop + marginBottom);
  if (usableH < 0) usableH = 0;

  int yBottom = innerY + innerH - marginBottom;
  int yTop    = innerY + marginTop;

  int sliderH = (usableH > 0) ? (int)((tftBacklightLevel / 100.0f) * usableH) : 0;
  int knobY   = yBottom - sliderH;
  if (knobY < yTop)    knobY = yTop;
  if (knobY > yBottom) knobY = yBottom;

  int trackH = yBottom - knobY;
  if (trackH < 0) trackH = 0;

  uint16_t barColor = TFT_WHITE;
  tft.fillRect(trackX, knobY, trackW, trackH, barColor);

  int knobRadius = innerW / 2;
  if (knobRadius < 4)  knobRadius = 4;
  if (knobRadius > 10) knobRadius = 10;
  int knobCenterX = trackX + trackW / 2;

  tft.fillCircle(knobCenterX, knobY, knobRadius, barColor);
  tft.drawCircle(knobCenterX, knobY, knobRadius, TFT_BLACK);
  tft.drawCircle(knobCenterX, knobY, knobRadius + 3, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  char buf[16];
  snprintf(buf, sizeof(buf), "%3u %%", tftBacklightLevel);
  tft.drawString(buf, 120, y + barH + 18);
}

// ---------- Efectos (placeholder + RESPIRACION) ----------

void drawSettingsRespScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars = -1;
  lastWifiTachado = false;

  // Cabecera
  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("RESPIRACION", 120, 15);   // título en mayúsculas

  drawWifiSignalIcon();

  drawHeaderText("RESPIRACION");

  // --- Slider de color con dos knobs ---

  // Geometría del slider
  int sliderX = 14;
  int sliderY = 80;
  int sliderW = 212;
  int sliderH = 18;

  // Dibujar fondo del slider 
  for (int i = 0; i < sliderW; i++) {
    uint8_t rr, gg, bb; 
    uint16_t c = colorFromSliderEffects((uint8_t)i, rr, gg, bb);
    tft.drawFastVLine(sliderX + i, sliderY, sliderH, c);
  }

  // Contorno blanco del slider (igual que en relojes)
  tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);

  // Líneas "N" y "B" en extremos, más altas que los knobs
  int gapAboveSlider = 5;
  int markerHeight = 20;
  int bottomY = sliderY - gapAboveSlider;
  int topY = bottomY - markerHeight;

  tft.drawFastVLine(sliderX, topY, markerHeight, TFT_WHITE);
  tft.drawFastVLine(sliderX + sliderW - 1, topY, markerHeight, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("N", sliderX, topY - 8);
  tft.drawString("B", sliderX + sliderW - 1, topY - 8);

  // Altura donde dibujar las bolitas de los knobs
  int knobRadius = 7;
  int knobCenterY = sliderY - 14;

  // Knob inicio (izquierdo)
  int xStart = sliderX + respKnobStartPos;
  tft.drawFastVLine(xStart, sliderY, sliderH, TFT_WHITE);
  tft.drawCircle(xStart, knobCenterY, knobRadius, TFT_WHITE);
  {
    uint8_t rr, gg, bb;
    uint16_t c = colorFromSliderEffects((uint8_t)respKnobStartPos, rr, gg, bb);
    tft.fillCircle(xStart, knobCenterY, knobRadius - 1, c);
  }

  // Knob final (derecho)
  int xEnd = sliderX + respKnobEndPos;
  tft.drawFastVLine(xEnd, sliderY, sliderH, TFT_WHITE);
  tft.drawCircle(xEnd, knobCenterY, knobRadius, TFT_WHITE);
  {
    uint8_t rr2, gg2, bb2;
    uint16_t c2 = colorFromSliderEffects((uint8_t)respKnobEndPos, rr2, gg2, bb2);

    // Si el knob está en el extremo derecho del slider, forzamos blanco
    if (respKnobEndPos >= 211) {        // 211 = sliderW - 1
      rr2 = 255;
      gg2 = 255;
      bb2 = 255;
      c2  = tft.color565(rr2, gg2, bb2);
    }

    tft.fillCircle(xEnd, knobCenterY, knobRadius - 1, c2);
  }

  // --- Texto RGB de knob activo ---
  uint16_t activeColor = (respFocus == RESP_FOCUS_END) ? respColorEnd : respColorStart;
  uint8_t r, g, b;
  rgbFrom565(activeColor, r, g, b);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  char buf[32];
  snprintf(buf, sizeof(buf), "R:%d G:%d B:%d", r, g, b);
  tft.drawString(buf, 120, sliderY + sliderH + 14);

  // --- Cajitas de color inicio / final ---

  int boxW = 60;
  int boxH = 24;
  int boxY = sliderY + sliderH + 36;
  int boxX0 = 120 - boxW - 6;
  int boxX1 = 120 + 6;

  // Borrar fondo
  tft.fillRect(boxX0 - 12, boxY - 2, (boxW + 6) * 2, boxH + 4, TFT_BLACK);

  // Indicadores de foco alrededor de las cajas
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (respFocus == RESP_FOCUS_START) {
    tft.drawString(">", boxX0 - 4, boxY + boxH / 2);   // a la izquierda de caja izquierda
  }
  tft.setTextDatum(ML_DATUM);
  if (respFocus == RESP_FOCUS_END) {
    tft.drawString("<", boxX1 + boxW + 4, boxY + boxH / 2); // a la derecha de caja derecha
  }

  // Caja izquierda: color inicio
  tft.drawRect(boxX0, boxY, boxW, boxH, TFT_WHITE);
  tft.fillRect(boxX0 + 1, boxY + 1, boxW - 2, boxH - 2, respColorStart);

  // Caja derecha: color final
  tft.drawRect(boxX1, boxY, boxW, boxH, TFT_WHITE);
  uint16_t boxEndColor = respColorEnd;
  if (respKnobEndPos >= 211) {
    boxEndColor = tft.color565(255, 255, 255);
  }
  tft.fillRect(boxX1 + 1, boxY + 1, boxW - 2, boxH - 2, respColorEnd);

  // --- Ciclo: texto y foco ---

  int cycleY = boxY + boxH + 20;
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Borrar zona ciclo
  tft.fillRect(0, cycleY - 10, 240, 24, TFT_BLACK);

  // Indicador de foco en ciclo
  if (respFocus == RESP_FOCUS_CYCLE) {
    tft.setTextDatum(MR_DATUM);
    tft.drawString(">", 50, cycleY);
  }

  // Texto ciclo centrado
  uint16_t tX10 = respCycleTimesX10[respCycleIndex];
  char bufC[16];
  if (tX10 < 10) {
    snprintf(bufC, sizeof(bufC), "0.%d", tX10);
  } else if (tX10 < 100) {
    snprintf(bufC, sizeof(bufC), "%d.%d", tX10 / 10, tX10 % 10);
  } else {
    snprintf(bufC, sizeof(bufC), "%d", tX10 / 10);
  }

  // Construimos la cadena completa "Ciclo: X"
  char lineC[24];
  snprintf(lineC, sizeof(lineC), "Ciclo: %s", bufC);

  // Centramos la cadena completa
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(lineC, 120, cycleY);

  // --- Botón Iniciar ---

  int btnY = cycleY + 28;
  int btnW = 100;
  int btnH = 26;
  int btnX = (240 - btnW) / 2;

  bool focusedButton = (respFocus == RESP_FOCUS_BUTTON);

  uint16_t btnFill = focusedButton ? TFT_WHITE : TFT_DARKGREY;
  uint16_t btnText = focusedButton ? TFT_BLACK : TFT_WHITE;

  tft.fillRoundRect(btnX, btnY, btnW, btnH, 4, btnFill);
  tft.drawRoundRect(btnX, btnY, btnW, btnH, 4, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(btnText, btnFill);
  tft.drawString("Iniciar", btnX + btnW / 2, btnY + btnH / 2);
}

void drawSettingsCometScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars = -1;
  lastWifiTachado = false;

  // Cabecera
  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("COMETA", 120, 15);   // título en mayúsculas

  drawWifiSignalIcon();

  drawHeaderText("COMETA");

  // --- Slider de color con dos knobs ---

  // Geometría del slider (misma que RESPIRACION)
  int sliderX = 14;
  int sliderY = 80;
  int sliderW = 212;
  int sliderH = 18;

  // Dibujar fondo del slider 
  for (int i = 0; i < sliderW; i++) {
    uint8_t rr, gg, bb; 
    uint16_t c = colorFromSliderEffects((uint8_t)i, rr, gg, bb);
    tft.drawFastVLine(sliderX + i, sliderY, sliderH, c);
  }

  // Contorno blanco del slider
  tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);

  // Líneas "N" y "B" en extremos
  int gapAboveSlider = 5;
  int markerHeight   = 20;
  int bottomY        = sliderY - gapAboveSlider;
  int topY           = bottomY - markerHeight;

  tft.drawFastVLine(sliderX,               topY, markerHeight, TFT_WHITE);
  tft.drawFastVLine(sliderX + sliderW - 1, topY, markerHeight, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("N", sliderX,               topY - 8);
  tft.drawString("B", sliderX + sliderW - 1, topY - 8);

  // Altura donde dibujar las bolitas de los knobs
  int knobRadius = 7;
  int knobCenterY = sliderY - 14;

  // Knob inicio (izquierdo)
  int xStart = sliderX + cometKnobStartPos;
  tft.drawFastVLine(xStart, sliderY, sliderH, TFT_WHITE);
  tft.drawCircle(xStart, knobCenterY, knobRadius, TFT_WHITE);
  {
    uint8_t rr, gg, bb;
    uint16_t c = colorFromSliderEffects((uint8_t)cometKnobStartPos, rr, gg, bb);
    tft.fillCircle(xStart, knobCenterY, knobRadius - 1, c);
  }

  // Knob final (derecho)
  int xEnd = sliderX + cometKnobEndPos;
  tft.drawFastVLine(xEnd, sliderY, sliderH, TFT_WHITE);
  tft.drawCircle(xEnd, knobCenterY, knobRadius, TFT_WHITE);
  {
    uint8_t rr2, gg2, bb2;
    uint16_t c2 = colorFromSliderEffects((uint8_t)cometKnobEndPos, rr2, gg2, bb2);

    if (cometKnobEndPos >= 211) {
      rr2 = 255;
      gg2 = 255;
      bb2 = 255;
      c2  = tft.color565(rr2, gg2, bb2);
    }

    tft.fillCircle(xEnd, knobCenterY, knobRadius - 1, c2);
  }

  // --- Texto RGB de knob activo ---
  uint16_t activeColor = (cometFocus == COMET_FOCUS_END) ? cometColorEnd : cometColorStart;
  uint8_t r, g, b;
  rgbFrom565(activeColor, r, g, b);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  char buf[32];
  snprintf(buf, sizeof(buf), "R:%d G:%d B:%d", r, g, b);
  tft.drawString(buf, 120, sliderY + sliderH + 14);

  // --- Cajitas de color inicio / final ---

  int boxW = 60;
  int boxH = 24;
  int boxY = sliderY + sliderH + 36;
  int boxX0 = 120 - boxW - 6;
  int boxX1 = 120 + 6;

  tft.fillRect(boxX0 - 12, boxY - 2, (boxW + 6) * 2, boxH + 4, TFT_BLACK);

  // Indicadores de foco alrededor de las cajas
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (cometFocus == COMET_FOCUS_START) {
    tft.drawString(">", boxX0 - 4, boxY + boxH / 2);
  }
  tft.setTextDatum(ML_DATUM);
  if (cometFocus == COMET_FOCUS_END) {
    tft.drawString("<", boxX1 + boxW + 4, boxY + boxH / 2);
  }

  // Caja izquierda: color inicio
  tft.drawRect(boxX0, boxY, boxW, boxH, TFT_WHITE);
  tft.fillRect(boxX0 + 1, boxY + 1, boxW - 2, boxH - 2, cometColorStart);

  // Caja derecha: color final
  tft.drawRect(boxX1, boxY, boxW, boxH, TFT_WHITE);
  uint16_t boxEndColor = cometColorEnd;
  tft.fillRect(boxX1 + 1, boxY + 1, boxW - 2, boxH - 2, boxEndColor);

  // --- Ciclo: texto y foco ---

  int cycleY = boxY + boxH + 20;
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.fillRect(0, cycleY - 10, 240, 24, TFT_BLACK);

  if (cometFocus == COMET_FOCUS_CYCLE) {
    tft.setTextDatum(MR_DATUM);
    tft.drawString(">", 50, cycleY);
  }

  uint16_t tX10 = cometCycleTimesX10[cometCycleIndex];
  char bufC[16];
  if (tX10 < 10) {
    snprintf(bufC, sizeof(bufC), "0.%d", tX10);
  } else if (tX10 < 100) {
    snprintf(bufC, sizeof(bufC), "%d.%d", tX10 / 10, tX10 % 10);
  } else {
    snprintf(bufC, sizeof(bufC), "%d", tX10 / 10);
  }

  char lineC[24];
  snprintf(lineC, sizeof(lineC), "Ciclo: %s", bufC);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(lineC, 120, cycleY);

  // --- Botón Iniciar ---

  int btnY = cycleY + 28;
  int btnW = 100;
  int btnH = 26;
  int btnX = (240 - btnW) / 2;

  bool focusedButton = (cometFocus == COMET_FOCUS_BUTTON);

  uint16_t btnFill = focusedButton ? TFT_WHITE : TFT_DARKGREY;
  uint16_t btnText = focusedButton ? TFT_BLACK : TFT_WHITE;

  tft.fillRoundRect(btnX, btnY, btnW, btnH, 4, btnFill);
  tft.drawRoundRect(btnX, btnY, btnW, btnH, 4, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(btnText, btnFill);
  tft.drawString("Iniciar", btnX + btnW / 2, btnY + btnH / 2);
}

void drawSettingsBarridoScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars = -1;
  lastWifiTachado = false;

  // Cabecera
  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("BARRIDO", 120, 15);

  drawWifiSignalIcon();

  drawHeaderText("BARRIDO");

  // --- Slider de color con dos knobs ---

  int sliderX = 14;
  int sliderY = 80;
  int sliderW = 212;
  int sliderH = 18;

  // Fondo del slider (mismo gradiente de efectos)
  for (int i = 0; i < sliderW; i++) {
    uint8_t rr, gg, bb;
    uint16_t c = colorFromSliderEffects((uint8_t)i, rr, gg, bb);
    tft.drawFastVLine(sliderX + i, sliderY, sliderH, c);
  }

  // Contorno del slider
  tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);

  // Líneas "N" y "B" arriba
  int gapAboveSlider = 5;
  int markerHeight   = 20;
  int bottomY        = sliderY - gapAboveSlider;
  int topY           = bottomY - markerHeight;

  tft.drawFastVLine(sliderX,               topY, markerHeight, TFT_WHITE);
  tft.drawFastVLine(sliderX + sliderW - 1, topY, markerHeight, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("N", sliderX,               topY - 8);
  tft.drawString("B", sliderX + sliderW - 1, topY - 8);

  // Knobs
  int knobRadius = 7;
  int knobCenterY = sliderY - 14;

  // Knob inicio
  int xStart = sliderX + barridoKnobStartPos;
  tft.drawFastVLine(xStart, sliderY, sliderH, TFT_WHITE);
  tft.drawCircle(xStart, knobCenterY, knobRadius, TFT_WHITE);
  {
    uint8_t rr, gg, bb;
    uint16_t c = colorFromSliderEffects((uint8_t)barridoKnobStartPos, rr, gg, bb);
    tft.fillCircle(xStart, knobCenterY, knobRadius - 1, c);
  }

  // Knob final
  int xEnd = sliderX + barridoKnobEndPos;
  tft.drawFastVLine(xEnd, sliderY, sliderH, TFT_WHITE);
  tft.drawCircle(xEnd, knobCenterY, knobRadius, TFT_WHITE);
  {
    uint8_t rr2, gg2, bb2;
    uint16_t c2 = colorFromSliderEffects((uint8_t)barridoKnobEndPos, rr2, gg2, bb2);

    if (barridoKnobEndPos >= 211) {
      rr2 = 255;
      gg2 = 255;
      bb2 = 255;
      c2  = tft.color565(rr2, gg2, bb2);
    }

    tft.fillCircle(xEnd, knobCenterY, knobRadius - 1, c2);
  }

  // --- Texto RGB del knob activo ---
  uint16_t activeColor = (barridoFocus == BARRIDO_FOCUS_END) ? barridoColorEnd : barridoColorStart;
  uint8_t r, g, b;
  rgbFrom565(activeColor, r, g, b);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  char buf[32];
  snprintf(buf, sizeof(buf), "R:%d G:%d B:%d", r, g, b);
  tft.drawString(buf, 120, sliderY + sliderH + 14);

  // --- Cajitas de color inicio / final ---

  int boxW = 60;
  int boxH = 24;
  int boxY = sliderY + sliderH + 36;
  int boxX0 = 120 - boxW - 6;
  int boxX1 = 120 + 6;

  tft.fillRect(boxX0 - 12, boxY - 2, (boxW + 6) * 2, boxH + 4, TFT_BLACK);

  // Indicadores de foco
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (barridoFocus == BARRIDO_FOCUS_START) {
    tft.drawString(">", boxX0 - 4, boxY + boxH / 2);
  }
  tft.setTextDatum(ML_DATUM);
  if (barridoFocus == BARRIDO_FOCUS_END) {
    tft.drawString("<", boxX1 + boxW + 4, boxY + boxH / 2);
  }

  // Caja inicio
  tft.drawRect(boxX0, boxY, boxW, boxH, TFT_WHITE);
  tft.fillRect(boxX0 + 1, boxY + 1, boxW - 2, boxH - 2, barridoColorStart);

  // Caja final
  tft.drawRect(boxX1, boxY, boxW, boxH, TFT_WHITE);
  uint16_t boxEndColor = barridoColorEnd;
  if (barridoKnobEndPos >= 211) {
    boxEndColor = tft.color565(255, 255, 255);
  }
  tft.fillRect(boxX1 + 1, boxY + 1, boxW - 2, boxH - 2, boxEndColor);

  // --- Ciclo ---

  int cycleY = boxY + boxH + 20;
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.fillRect(0, cycleY - 10, 240, 24, TFT_BLACK);

  if (barridoFocus == BARRIDO_FOCUS_CYCLE) {
    tft.setTextDatum(MR_DATUM);
    tft.drawString(">", 50, cycleY);
  }

  uint16_t tX10 = barridoCycleTimesX10[barridoCycleIndex];
  char bufC[16];
  if (tX10 < 10) {
    snprintf(bufC, sizeof(bufC), "0.%d", tX10);
  } else if (tX10 < 100) {
    snprintf(bufC, sizeof(bufC), "%d.%d", tX10 / 10, tX10 % 10);
  } else {
    snprintf(bufC, sizeof(bufC), "%d", tX10 / 10);
  }

  char lineC[24];
  snprintf(lineC, sizeof(lineC), "Ciclo: %s", bufC);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(lineC, 120, cycleY);

  // --- Botón Iniciar ---

  int btnY = cycleY + 28;
  int btnW = 100;
  int btnH = 26;
  int btnX = (240 - btnW) / 2;

  bool focusedButton = (barridoFocus == BARRIDO_FOCUS_BUTTON);

  uint16_t btnFill = focusedButton ? TFT_WHITE : TFT_DARKGREY;
  uint16_t btnText = focusedButton ? TFT_BLACK : TFT_WHITE;

  tft.fillRoundRect(btnX, btnY, btnW, btnH, 4, btnFill);
  tft.drawRoundRect(btnX, btnY, btnW, btnH, 4, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(btnText, btnFill);
  tft.drawString("Iniciar", btnX + btnW / 2, btnY + btnH / 2);
}

void drawSettingsPersianaScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars = -1;
  lastWifiTachado = false;

  // Cabecera
  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("PERSIANA", 120, 15);

  drawWifiSignalIcon();

  drawHeaderText("PERSIANA");

  // --- Slider de color con dos knobs ---

  int sliderX = 14;
  int sliderY = 80;
  int sliderW = 212;
  int sliderH = 18;

  // Fondo del slider (mismo gradiente de efectos)
  for (int i = 0; i < sliderW; i++) {
    uint8_t rr, gg, bb;
    uint16_t c = colorFromSliderEffects((uint8_t)i, rr, gg, bb);
    tft.drawFastVLine(sliderX + i, sliderY, sliderH, c);
  }

  // Contorno del slider
  tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);

  // Líneas "N" y "B" arriba
  int gapAboveSlider = 5;
  int markerHeight   = 20;
  int bottomY        = sliderY - gapAboveSlider;
  int topY           = bottomY - markerHeight;

  tft.drawFastVLine(sliderX,               topY, markerHeight, TFT_WHITE);
  tft.drawFastVLine(sliderX + sliderW - 1, topY, markerHeight, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("N", sliderX,               topY - 8);
  tft.drawString("B", sliderX + sliderW - 1, topY - 8);

  // Knobs
  int knobRadius = 7;
  int knobCenterY = sliderY - 14;

  // Knob inicio
  int xStart = sliderX + persianaKnobStartPos;
  tft.drawFastVLine(xStart, sliderY, sliderH, TFT_WHITE);
  tft.drawCircle(xStart, knobCenterY, knobRadius, TFT_WHITE);
  {
    uint8_t rr, gg, bb;
    uint16_t c = colorFromSliderEffects((uint8_t)persianaKnobStartPos, rr, gg, bb);
    tft.fillCircle(xStart, knobCenterY, knobRadius - 1, c);
  }

  // Knob final
  int xEnd = sliderX + persianaKnobEndPos;
  tft.drawFastVLine(xEnd, sliderY, sliderH, TFT_WHITE);
  tft.drawCircle(xEnd, knobCenterY, knobRadius, TFT_WHITE);
  {
    uint8_t rr2, gg2, bb2;
    uint16_t c2 = colorFromSliderEffects((uint8_t)persianaKnobEndPos, rr2, gg2, bb2);

    if (persianaKnobEndPos >= 211) {
      rr2 = 255;
      gg2 = 255;
      bb2 = 255;
      c2  = tft.color565(rr2, gg2, bb2);
    }

    tft.fillCircle(xEnd, knobCenterY, knobRadius - 1, c2);
  }

  // --- Texto RGB del knob activo ---
  uint16_t activeColor = (persianaFocus == PERSIANA_FOCUS_END) ? persianaColorEnd : persianaColorStart;
  uint8_t r, g, b;
  rgbFrom565(activeColor, r, g, b);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  char buf[32];
  snprintf(buf, sizeof(buf), "R:%d G:%d B:%d", r, g, b);
  tft.drawString(buf, 120, sliderY + sliderH + 14);

  // --- Cajitas de color inicio / final ---

  int boxW = 60;
  int boxH = 24;
  int boxY = sliderY + sliderH + 36;
  int boxX0 = 120 - boxW - 6;
  int boxX1 = 120 + 6;

  tft.fillRect(boxX0 - 12, boxY - 2, (boxW + 6) * 2, boxH + 4, TFT_BLACK);

  // Indicadores de foco
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (persianaFocus == PERSIANA_FOCUS_START) {
    tft.drawString(">", boxX0 - 4, boxY + boxH / 2);
  }
  tft.setTextDatum(ML_DATUM);
  if (persianaFocus == PERSIANA_FOCUS_END) {
    tft.drawString("<", boxX1 + boxW + 4, boxY + boxH / 2);
  }

  // Caja inicio
  tft.drawRect(boxX0, boxY, boxW, boxH, TFT_WHITE);
  tft.fillRect(boxX0 + 1, boxY + 1, boxW - 2, boxH - 2, persianaColorStart);

  // Caja final
  tft.drawRect(boxX1, boxY, boxW, boxH, TFT_WHITE);
  uint16_t boxEndColor = persianaColorEnd;
  if (persianaKnobEndPos >= 211) {
    boxEndColor = tft.color565(255, 255, 255);
  }
  tft.fillRect(boxX1 + 1, boxY + 1, boxW - 2, boxH - 2, boxEndColor);

  // --- Ciclo ---

  int cycleY = boxY + boxH + 20;
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.fillRect(0, cycleY - 10, 240, 24, TFT_BLACK);

  if (persianaFocus == PERSIANA_FOCUS_CYCLE) {
    tft.setTextDatum(MR_DATUM);
    tft.drawString(">", 50, cycleY);
  }

  uint16_t tX10 = persianaCycleTimesX10[persianaCycleIndex];
  char bufC[16];
  if (tX10 < 10) {
    snprintf(bufC, sizeof(bufC), "0.%d", tX10);
  } else if (tX10 < 100) {
    snprintf(bufC, sizeof(bufC), "%d.%d", tX10 / 10, tX10 % 10);
  } else {
    snprintf(bufC, sizeof(bufC), "%d", tX10 / 10);
  }

  char lineC[24];
  snprintf(lineC, sizeof(lineC), "Ciclo: %s", bufC);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(lineC, 120, cycleY);

  // --- Botón Iniciar ---

  int btnY = cycleY + 28;
  int btnW = 100;
  int btnH = 26;
  int btnX = (240 - btnW) / 2;

  bool focusedButton = (persianaFocus == PERSIANA_FOCUS_BUTTON);

  uint16_t btnFill = focusedButton ? TFT_WHITE : TFT_DARKGREY;
  uint16_t btnText = focusedButton ? TFT_BLACK : TFT_WHITE;

  tft.fillRoundRect(btnX, btnY, btnW, btnH, 4, btnFill);
  tft.drawRoundRect(btnX, btnY, btnW, btnH, 4, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(btnText, btnFill);
  tft.drawString("Iniciar", btnX + btnW / 2, btnY + btnH / 2);
}

// ---------- Menús WiFi ----------

int  wifiMenuIndex    = 0;
int  wifiScanIndex    = 0;
int  wifiScanCount    = 0;
int  wifiKbIndex      = 0;
bool wifiKbUppercase  = false;
char wifiPwdPlain[WIFI_PWD_BUF_LEN + 1] = {0};

const char* wifiKbLetters = "abcdefghijklmnopqrstuvwxyz";
const char* wifiKbDigits  = "0123456789";

bool wifiScanDone = false;

void drawSettingsWifiScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("WiFi", 120, 15);

  drawWifiSignalIcon();

  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);

  char status[40];
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(status, sizeof(status), "Conectado: %s", WiFi.SSID().c_str());
  } else if (wifiSsid[0]) {
    snprintf(status, sizeof(status), "Ultimo: %s", wifiSsid);
  } else {
    snprintf(status, sizeof(status), "Sin WiFi");
  }
  tft.drawString(status, 10, 40);

  tft.setTextSize(2);
  const char* lines[3] = { "Buscar redes", "Olvidar red", "Volver" };
  int items  = 3;
  int startY = 80;
  int lineH  = 24;

  for (int i = 0; i < items; i++) {
    int  y        = startY + i * lineH;
    bool selected = (i == wifiMenuIndex);

    if (selected) {
      tft.fillRect(10, y - 2, 220, lineH, TFT_DARKGREY);
      tft.setTextColor(TFT_NAVY, TFT_DARKGREY);
    } else {
      tft.fillRect(10, y - 2, 220, lineH, TFT_BLACK);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    tft.drawString(lines[i], 14, y);
  }
}

void drawSettingsWifiScanScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Redes WiFi", 120, 15);

  drawWifiSignalIcon();

  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);

  if (!wifiScanDone) {
    tft.drawString("Escaneando...", 10, 40);
    wifiScanCount = WiFi.scanNetworks();
    wifiScanDone  = true;
    tft.fillRect(0, 40, 240, 200, TFT_BLACK);
  }

  if (wifiScanCount <= 0) {
    tft.setTextSize(2);
    tft.drawString("Sin redes", 10, 80);
    return;
  }

  if (wifiScanIndex >= wifiScanCount) wifiScanIndex = wifiScanCount - 1;
  if (wifiScanIndex < 0)              wifiScanIndex = 0;

  int startY   = 50;
  int lineH    = 18;
  int maxLines = 8;

  int first = wifiScanIndex - (wifiScanIndex % maxLines);
  for (int i = 0; i < maxLines; i++) {
    int idx = first + i;
    if (idx >= wifiScanCount) break;

    int  y        = startY + i * lineH;
    bool selected = (idx == wifiScanIndex);

    String ssid  = WiFi.SSID(idx);
    int32_t rssi = WiFi.RSSI(idx);
    bool    enc  = (WiFi.encryptionType(idx) != WIFI_AUTH_OPEN);

    char line[40];
    snprintf(line, sizeof(line), "%-15s %4ddBm %c",
             ssid.c_str(), (int)rssi, enc ? '*' : ' ');

    if (selected) {
      tft.fillRect(5, y - 1, 230, lineH, TFT_DARKGREY);
      tft.setTextColor(TFT_NAVY, TFT_DARKGREY);
    } else {
      tft.fillRect(5, y - 1, 230, lineH, TFT_BLACK);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    tft.drawString(line, 10, y);
  }
}

// ---------- Teclado WiFi ----------

void drawSettingsWifiPwdScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Clave WiFi", 120, 15);

  drawWifiSignalIcon();

  tft.setTextSize(1);
  tft.setTextDatum(TL_DATUM);

  char ssidLine[40];
  snprintf(ssidLine, sizeof(ssidLine), "SSID: %s", wifiSsid[0] ? wifiSsid : "(sin)");
  tft.drawString(ssidLine, 10, 40);

  char pwdLine[80];
  snprintf(pwdLine, sizeof(pwdLine), "Clave: %s", wifiPwdPlain);
  tft.drawString(pwdLine, 10, 60);

  int cols   = 10;
  int keyW   = 24;
  int keyH   = 24;
  int startX = 0;
  int startY = 96;

  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);

  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < cols; col++) {
      int i = row * cols + col;
      int x = startX + col * keyW;
      int y = startY + row * keyH;

      bool selected = (wifiKbIndex == i);

      char c = 0;
      if (row == 0) {
        int idx = col;
        c = wifiKbLetters[idx];
        if (wifiKbUppercase) c += ('A' - 'a');
      } else if (row == 1) {
        int idx = 10 + col;
        c = wifiKbLetters[idx];
        if (wifiKbUppercase) c += ('A' - 'a');
      } else if (row == 2) {
        if (col <= 5) {
          int idx = 20 + col;
          c = wifiKbLetters[idx];
          if (wifiKbUppercase) c += ('A' - 'a');
        } else {
          int d = col - 6;
          c = wifiKbDigits[d];
        }
      } else if (row == 3) {
        if (col <= 5) {
          int d = 4 + col;
          c = wifiKbDigits[d];
        } else if (col == 6) {
          c = '.';
        } else if (col == 7) {
          c = '-';
        } else {
          c = 0;
        }
      }

      uint16_t bg = TFT_BLACK;
      uint16_t fg = TFT_WHITE;
      if (selected) {
        bg = TFT_DARKGREY;
        fg = TFT_NAVY;
      }
      tft.fillRect(x, y, keyW, keyH, bg);
      if (c != 0) {
        char s[2] = { c, 0 };
        tft.setTextColor(fg, bg);
        tft.drawString(s, x + keyW/2, y + keyH/2);
      }
    }
  }

  const char* labels[5] = { "ESP", "A/a", "DEL", "OK", "X" };
  int ctrlRowY  = startY + 4 * keyH;

  int ctrlFullW = 240;
  int marginSide= 4;
  int usable    = ctrlFullW - 2 * marginSide;
  int blockW    = usable / 5;
  int x0        = marginSide + (usable - blockW * 5) / 2;

  for (int g = 0; g < 5; g++) {
    int bx = x0 + g * blockW;
    int by = ctrlRowY;
    int logicalIndex = 40 + g;
    bool selected    = (wifiKbIndex == logicalIndex);

    uint16_t bg = TFT_BLACK;
    uint16_t fg = TFT_WHITE;
    if (selected) {
      bg = TFT_DARKGREY;
      fg = TFT_NAVY;
    }
    tft.fillRect(bx, by, blockW, keyH, bg);
    tft.setTextColor(fg, bg);
    tft.drawString(labels[g], bx + blockW/2, by + keyH/2);
  }
}

// ---------- Pantalla confirmación Reinicio HW ----------

void drawSettingsResetConfirmScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Reinicio HW", 120, 15);

  drawWifiSignalIcon();

  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);

  tft.drawString("Se van a eliminar", 120, 60);
  tft.drawString("los ajustes y WiFi.", 120, 85);
  tft.drawString("Continuar?", 120, 115);

  int yBtns = 160;
  int wBtn  = 80;
  int hBtn  = 32;
  int xSi   = 40;
  int xNo   = 240 - 40 - wBtn;

  bool selSi = (resetConfirmIndex == 0);
  tft.fillRect(xSi, yBtns, wBtn, hBtn, selSi ? TFT_DARKGREY : TFT_BLACK);
  tft.setTextColor(selSi ? TFT_NAVY : TFT_WHITE,
                   selSi ? TFT_DARKGREY : TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString("Si", xSi + wBtn/2, yBtns + hBtn/2);

  bool selNo = (resetConfirmIndex == 1);
  tft.fillRect(xNo, yBtns, wBtn, hBtn, selNo ? TFT_DARKGREY : TFT_BLACK);
  tft.setTextColor(selNo ? TFT_NAVY : TFT_WHITE,
                   selNo ? TFT_DARKGREY : TFT_BLACK);
  tft.drawString("No", xNo + wBtn/2, yBtns + hBtn/2);
}

// ---------- Pantalla "Acerca de..." ----------

void drawSettingsAboutScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Acerca de...", 120, 15);

  drawWifiSignalIcon();

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);

  int y  = 60;
  int dy = 20;

  tft.drawString("LAMP_Fun V.2.6.4",     120, y); y += dy + 4;
  tft.drawString("J. L. Marcos Bezos",   120, y); y += dy;
  tft.drawString("Junio 2026",          120, y); y += dy;
  tft.drawString("ESP32 + TFT 240x240",   120, y); y += dy;
  tft.drawString("EC11 + Foco WS2812B", 120, y); y += dy;
  tft.drawString("INMP441 + Max98357A", 120, y); y += dy;
  tft.drawString("Proyecto DIY",        120, y);
}

// ----------------- setup() -----------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("LAMP_Fun V2.4.2");

  initBacklight();

  pinMode(BUTTON2_PIN, INPUT);
  pinMode(ENCODER_SW,  INPUT_PULLUP);
  pinMode(ENCODER_A,   INPUT_PULLUP);
  pinMode(ENCODER_B,   INPUT_PULLUP);

  lastEncA = digitalRead(ENCODER_A);

  loadConfig();
  wifiLoadCredentials();
  applyBacklightPWM();

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(brightness);
  updateLeds();

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  WiFi.mode(WIFI_STA);

  bool connected = false;
  if (hasWifiCredentials) {
    connected = wifiConnectUsingStored(15000);

    if (connected) {
      useAutoTime = true;
      saveConfigBasic();
      setenv("TZ", TIMEZONES[tzIndex].tzStr, 1);
      tzset();
      configTzTime(TIMEZONES[tzIndex].tzStr, NTP_SERVER);
      lastNtpSyncMillis = millis();
    } else {
      useAutoTime = false;
      saveConfigBasic();
      initManualTime_2026();
    }
  } else {
    useAutoTime = false;
    saveConfigBasic();
    initManualTime_2026();
  }

  lastWifiBars    = -1;
  lastWifiTachado = false;
  currentScreen   = SCREEN_SPLASH;
  splashStartMillis   = millis();
  lastWifiCheckMillis = millis();
  drawSplashScreen();
}

// ----------------- loop() -----------------

void loop() {
  int stepDir = readEncoderStep();

  static bool lastSw   = true;
  bool        sw       = digitalRead(ENCODER_SW);
  static bool lastBtn2 = true;
  bool        btn2     = digitalRead(BUTTON2_PIN);

  bool btn2Falling       = (!btn2 && lastBtn2);
  bool encButtonFalling  = (!sw  && lastSw);

  if (useAutoTime && hasWifiCredentials) {
    unsigned long now = millis();
    if (now - lastWifiCheckMillis >= WIFI_RETRY_INTERVAL) {
      lastWifiCheckMillis = now;
      if (WiFi.status() != WL_CONNECTED) {
        bool ok = wifiConnectUsingStored(3000);
        if (ok) {
          setenv("TZ", TIMEZONES[tzIndex].tzStr, 1);
          tzset();
          configTzTime(TIMEZONES[tzIndex].tzStr, NTP_SERVER);
          lastNtpSyncMillis = now;
        }
        lastWifiBars    = -1;
        lastWifiTachado = false;
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (now - lastNtpSyncMillis >= NTP_SYNC_INTERVAL) {
        setenv("TZ", TIMEZONES[tzIndex].tzStr, 1);
        tzset();
        configTzTime(TIMEZONES[tzIndex].tzStr, NTP_SERVER);
        lastNtpSyncMillis = now;
      }
    }
  }

  switch (currentScreen) {
    case SCREEN_SPLASH: {
      updateSplashProgress();

      if (millis() - splashStartMillis >= SPLASH_DURATION) {
        currentScreen = SCREEN_CLOCK;
        drawClockScreenFull();
      }
      break;
    }

    case SCREEN_CLOCK: {
      updateClockScreen();

      if (stepDir != 0 || encButtonFalling) {
        if (anyEffectActive) {
          stopAllEffects();
          updateLeds();
        }
        currentScreen  = SCREEN_LIGHT;
        currentControl = CTRL_BTN_POWER;
        editingBar     = false;
        drawLightScreen();
      }

      if (btn2Falling) {
        if (anyEffectActive) {
          stopAllEffects();
        }
        settingsMainIndex = 0;
        currentScreen = SCREEN_SETTINGS_MAIN;
        drawSettingsMainScreen();
      }
      break;
    }

    case SCREEN_LIGHT: {
      if (!editingBar) {
        if (stepDir != 0) {
          currentControl += (stepDir > 0 ? 1 : -1);
          if (currentControl < 0)          currentControl = CTRL_COUNT - 1;
          if (currentControl >= CTRL_COUNT) currentControl = 0;
          redrawLightControls();
        }
      } else {
        if (stepDir != 0) {
          int  delta   = (stepDir > 0 ? 5 : -5);
          bool changed = false;

          switch (currentControl) {
            case CTRL_BAR_R: {
              int v = (int)redValue + delta;
              if (v < 0)   v = 0;
              if (v > 255) v = 255;
              if (v != redValue) { redValue = (uint8_t)v; changed = true; }
              break;
            }
            case CTRL_BAR_G: {
              int v = (int)greenValue + delta;
              if (v < 0)   v = 0;
              if (v > 255) v = 255;
              if (v != greenValue) { greenValue = (uint8_t)v; changed = true; }
              break;
            }
            case CTRL_BAR_B: {
              int v = (int)blueValue + delta;
              if (v < 0)   v = 0;
              if (v > 255) v = 255;
              if (v != blueValue) { blueValue = (uint8_t)v; changed = true; }
              break;
            }
            case CTRL_BAR_BRIGHT: {
              int v = (int)brightness + delta;
              if (v < 0)   v = 0;
              if (v > 255) v = 255;
              if (v != brightness) { brightness = (uint8_t)v; changed = true; }
              break;
            }
            default:
              break;
          }

          if (changed) {
            updateLeds();
            redrawLightControls();
            saveConfigBasic();
          }
        }
      }

      if (encButtonFalling) {
        if (currentControl == CTRL_BTN_POWER) {
          lampOn = !lampOn;
          updateLeds();
          redrawLightControls();
        } else {
          editingBar = !editingBar;
          redrawLightControls();
        }
      }

      if (btn2Falling) {
        stopAllEffects();
        currentScreen = SCREEN_CLOCK;
        drawClockScreenFull();
      }
      break;
    }

    case SCREEN_SETTINGS_MAIN: {
      if (stepDir != 0) {
        settingsMainIndex += (stepDir > 0 ? 1 : -1);
        if (settingsMainIndex < 0) settingsMainIndex = SETTINGS_MAIN_ITEMS - 1;
        if (settingsMainIndex >= SETTINGS_MAIN_ITEMS) settingsMainIndex = 0;
        drawSettingsMainScreen();
      }

      if (encButtonFalling) {
        switch (settingsMainIndex) {
          case 0:
            settingsClockIndex = 0;
            currentScreen = SCREEN_SETTINGS_CLOCK;
            drawSettingsClockScreen();
            break;
          case 1:
            currentScreen = SCREEN_SETTINGS_BACKLIGHT;
            drawSettingsBacklightScreen();
            break;
          case 2:
            // Efectos -> SIEMPRE entra primero en la lista
            settingsEffectsIndex = 0;
            currentScreen = SCREEN_SETTINGS_EFFECTS;
            drawSettingsEffectsScreen();
            break;
          case 3:
            wifiMenuIndex = 0;
            currentScreen = SCREEN_SETTINGS_WIFI;
            drawSettingsWifiScreen();
            break;
          case 4:
            resetConfirmIndex = 1;
            currentScreen = SCREEN_SETTINGS_RESET_CONFIRM;
            drawSettingsResetConfirmScreen();
            break;
          case 5:
            currentScreen = SCREEN_SETTINGS_ABOUT;
            drawSettingsAboutScreen();
            break;
          case 6:
            currentScreen = SCREEN_CLOCK;
            drawClockScreenFull();
            break;
        }
      }

      if (btn2Falling) {
        currentScreen = SCREEN_CLOCK;
        drawClockScreenFull();
      }

      break;
    }

    case SCREEN_SETTINGS_CLOCK: {
      if (stepDir != 0) {
        settingsClockIndex += (stepDir > 0 ? 1 : -1);
        if (settingsClockIndex < 0)                      settingsClockIndex = SETTINGS_CLOCK_ITEMS - 1;
        if (settingsClockIndex >= SETTINGS_CLOCK_ITEMS)  settingsClockIndex = 0;
        drawSettingsClockScreen();
      }

      if (encButtonFalling) {
        if (settingsClockIndex == 0) {
          settingsFormatIndex = use24hFormat ? 0 : 1;
          currentScreen       = SCREEN_SETTINGS_FORMAT;
          drawSettingsFormatScreen();
        } else if (settingsClockIndex == 1) {
          settingsDateModeIndex = useAutoTime ? 0 : 1;
          currentScreen         = SCREEN_SETTINGS_DATEMODE;
          drawSettingsDateModeScreen();
        } else if (settingsClockIndex == 2) {
          clockMode = (clockMode == 0) ? 1 : 0;
          saveConfigBasic();
          analogFaceDrawn      = false;
          forceFullClockRedraw = true;
          drawSettingsClockScreen();
        } else if (settingsClockIndex == 3) {
          settingsTzIndexTemp = tzIndex;
          currentScreen       = SCREEN_SETTINGS_TIMEZONE;
          drawSettingsTimezoneScreen();
        } else if (settingsClockIndex == 4) {
          settingsTzOffsetTemp = tzOffsetSteps;
          currentScreen        = SCREEN_SETTINGS_TZOFFSET;
          drawSettingsTzOffsetScreen();
        } else if (settingsClockIndex == 5) {
          if (clockMode == 0) {
            currentScreen             = SCREEN_SETTINGS_COLORS_DIGITAL;
            settingsColorDigitalIndex = 0;
            uint16_t c = digitalHMColor;
            colorPos = sliderPosFromColor(c);
            drawSettingsColorsDigitalScreen();
          } else {
            currentScreen            = SCREEN_SETTINGS_COLORS_ANALOG;
            settingsColorAnalogIndex = 0;
            uint16_t c = analogHourHandColor;
            colorPos = sliderPosFromColor(c);
            drawSettingsColorsAnalogScreen();
          }
        }
      }

      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_MAIN;
        drawSettingsMainScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_FORMAT: {
      if (stepDir != 0) {
        settingsFormatIndex += (stepDir > 0 ? 1 : -1);
        if (settingsFormatIndex < 0) settingsFormatIndex = 1;
        if (settingsFormatIndex > 1) settingsFormatIndex = 0;
        drawSettingsFormatScreen();
      }

      if (encButtonFalling) {
        use24hFormat = (settingsFormatIndex == 0);
        saveConfigBasic();
        currentScreen = SCREEN_SETTINGS_CLOCK;
        drawSettingsClockScreen();
      }

      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_CLOCK;
        drawSettingsClockScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_DATEMODE: {
      if (stepDir != 0) {
        settingsDateModeIndex += (stepDir > 0 ? 1 : -1);
        if (settingsDateModeIndex < 0) settingsDateModeIndex = 1;
        if (settingsDateModeIndex > 1) settingsDateModeIndex = 0;
        drawSettingsDateModeScreen();
      }

      if (encButtonFalling) {
        if (settingsDateModeIndex == 0) {
          useAutoTime = true;
          saveConfigBasic();
          if (WiFi.status() == WL_CONNECTED) {
            setenv("TZ", TIMEZONES[tzIndex].tzStr, 1);
            tzset();
            configTzTime(TIMEZONES[tzIndex].tzStr, NTP_SERVER);
            lastNtpSyncMillis = millis();
          }
          currentScreen = SCREEN_SETTINGS_CLOCK;
          drawSettingsClockScreen();
        } else {
          useAutoTime = false;
          saveConfigBasic();

          struct tm ti;
          if (getAdjustedLocalTime(ti)) {
            editHour   = ti.tm_hour;
            editMinute = ti.tm_min;
            editDay    = ti.tm_mday;
            editMonth  = ti.tm_mon + 1;
            editYear   = ti.tm_year + 1900;
          } else {
            editHour   = 0;
            editMinute = 0;
            editDay    = 1;
            editMonth  = 1;
            editYear   = 2026;
          }

          editField     = FIELD_HOUR;
          currentScreen = SCREEN_SETTINGS_DATETIME;
          drawEditDateTimeField();
          invalidateTimeModeLabel();
        }
      }

      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_CLOCK;
        drawSettingsClockScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_TIMEZONE: {
      if (stepDir != 0) {
        settingsTzIndexTemp += (stepDir > 0 ? 1 : -1);
        if (settingsTzIndexTemp < 0)                  settingsTzIndexTemp = TIMEZONES_COUNT - 1;
        if (settingsTzIndexTemp >= TIMEZONES_COUNT)   settingsTzIndexTemp = 0;
        drawSettingsTimezoneScreen();
      }

      if (encButtonFalling) {
        tzIndex = settingsTzIndexTemp;
        saveConfigBasic();
        if (useAutoTime && WiFi.status() == WL_CONNECTED) {
          setenv("TZ", TIMEZONES[tzIndex].tzStr, 1);
          tzset();
          configTzTime(TIMEZONES[tzIndex].tzStr, NTP_SERVER);
          lastNtpSyncMillis = millis();
        }
        currentScreen = SCREEN_SETTINGS_CLOCK;
        drawSettingsClockScreen();
      }

      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_CLOCK;
        drawSettingsClockScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_TZOFFSET: {
      if (stepDir != 0) {
        settingsTzOffsetTemp += (stepDir > 0 ? 1 : -1);
        if (settingsTzOffsetTemp < -4) settingsTzOffsetTemp = -4;
        if (settingsTzOffsetTemp >  4) settingsTzOffsetTemp =  4;
        drawSettingsTzOffsetScreen();
      }

      if (encButtonFalling) {
        tzOffsetSteps = settingsTzOffsetTemp;
        saveConfigBasic();
        currentScreen = SCREEN_SETTINGS_CLOCK;
        drawSettingsClockScreen();
      }

      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_CLOCK;
        drawSettingsClockScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_DATETIME: {
      if (stepDir != 0) {
        int delta = (stepDir > 0 ? 1 : -1);
        switch (editField) {
          case FIELD_HOUR:
            editHour += delta;
            if (editHour < 0)  editHour = 23;
            if (editHour > 23) editHour = 0;
            break;
          case FIELD_MIN:
            editMinute += delta;
            if (editMinute < 0)  editMinute = 59;
            if (editMinute > 59) editMinute = 0;
            break;
          case FIELD_DAY:
            editDay += delta;
            if (editDay < 1)  editDay = 31;
            if (editDay > 31) editDay = 1;
            break;
          case FIELD_MONTH:
            editMonth += delta;
            if (editMonth < 1)  editMonth = 12;
            if (editMonth > 12) editMonth = 1;
            break;
          case FIELD_YEAR:
            editYear += delta;
            if (editYear < 2000) editYear = 2099;
            if (editYear > 2099) editYear = 2000;
            break;
          default:
            break;
        }
        drawEditDateTimeField();
      }

      if (encButtonFalling) {
        if (editField == FIELD_YEAR) {
          applyManualDateTime();
          currentScreen = SCREEN_SETTINGS_CLOCK;
          drawSettingsClockScreen();
        } else {
          editField = (DateTimeField)((int)editField + 1);
          drawEditDateTimeField();
        }
      }

      if (btn2Falling) {
        if (editField == FIELD_YEAR) {
          applyManualDateTime();
          currentScreen = SCREEN_CLOCK;
          drawClockScreenFull();
        } else {
          currentScreen = SCREEN_SETTINGS_CLOCK;
          drawSettingsClockScreen();
        }
      }
      break;
    }

    case SCREEN_SETTINGS_BACKLIGHT: {
      if (stepDir != 0) {
        int delta = (stepDir > 0 ? 1 : -1);
        int v     = (int)tftBacklightLevel + delta;
        if (v < 0)   v = 0;
        if (v > 100) v = 100;
        if (v != tftBacklightLevel) {
          tftBacklightLevel = (uint8_t)v;
          applyBacklightPWM();
          saveConfigBasic();
          drawSettingsBacklightScreen();
        }
      }

      if (btn2Falling || encButtonFalling) {
        currentScreen = SCREEN_SETTINGS_MAIN;
        drawSettingsMainScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_EFFECTS: {
      // Navegación por la lista de efectos
      if (stepDir != 0) {
        int dir = (stepDir > 0) ? 1 : -1;
        settingsEffectsIndex += dir;
        if (settingsEffectsIndex < 0) settingsEffectsIndex = SETTINGS_EFFECTS_ITEMS - 1;
        if (settingsEffectsIndex >= SETTINGS_EFFECTS_ITEMS) settingsEffectsIndex = 0;
        drawSettingsEffectsScreen();
      }

      // Pulsación del encoder: entrar en el efecto seleccionado
      if (encButtonFalling) {
        switch (settingsEffectsIndex) {
          case 0: // RESPIRACION
            respFocus = RESP_FOCUS_START;
            initRespSliderPositions();
            currentScreen = SCREEN_SETTINGS_RESP;
            drawSettingsRespScreen();
            break;

          case 1: // COMETA
            cometFocus = COMET_FOCUS_START;
            initCometSliderPositions();
            currentScreen = SCREEN_SETTINGS_COMET;
            drawSettingsCometScreen();
            break;
          
          case 2: // BARRIDO
            barridoFocus = BARRIDO_FOCUS_START;
            initBarridoSliderPositions();
            currentScreen = SCREEN_SETTINGS_BARRIDO;
            drawSettingsBarridoScreen();
            break;
          case 3: // PERSIANA
            persianaFocus = PERSIANA_FOCUS_START;
            initPersianaSliderPositions();
            currentScreen = SCREEN_SETTINGS_PERSIANA;
            drawSettingsPersianaScreen();
            break;

        }
      }

      // Botón 2: volver al menú de Ajustes principal
      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_MAIN;
        drawSettingsMainScreen();
      }

      break;
    }

    case SCREEN_SETTINGS_RESP: {
      // Pantalla de configuración de RESPIRACION

      if (stepDir != 0) {
        int dir = (stepDir > 0) ? 1 : -1;

        if (respFocus == RESP_FOCUS_START) {
          int step = 5;
          respKnobStartPos += dir * step;
          if (respKnobStartPos < 0)   respKnobStartPos = 0;
          if (respKnobStartPos > 211) respKnobStartPos = 211;

          uint8_t rr, gg, bb;
          respColorStart = colorFromSliderEffects((uint8_t)respKnobStartPos, rr, gg, bb);
          saveConfigBasic();
          drawSettingsRespScreen();
        }
        else if (respFocus == RESP_FOCUS_END) {
          int step = 5;
          respKnobEndPos += dir * step;
          if (respKnobEndPos < 0)   respKnobEndPos = 0;
          if (respKnobEndPos > 211) respKnobEndPos = 211;

          uint8_t rr2, gg2, bb2;
          respColorEnd = colorFromSliderEffects((uint8_t)respKnobEndPos, rr2, gg2, bb2);
          if (respKnobEndPos >= 211) {
            rr2 = 255;
            gg2 = 255; 
            bb2 = 255;
            respColorEnd = tft.color565(rr2, gg2, bb2);
          }
          saveConfigBasic();
          drawSettingsRespScreen();
        }
        else if (respFocus == RESP_FOCUS_CYCLE) {
          int idx = (int)respCycleIndex + dir;
          if (idx < 0) idx = 0;
          if (idx >= RESP_CYCLE_STEPS) idx = RESP_CYCLE_STEPS - 1;

          if (idx != respCycleIndex) {
            respCycleIndex = (uint8_t)idx;
            saveConfigBasic();
            drawSettingsRespScreen();
          }
        }
        // RESP_FOCUS_BUTTON: el giro no hace nada
      }

      if (encButtonFalling) {
        if (respFocus == RESP_FOCUS_BUTTON) {
          // Lanzar efecto y volver al reloj
          startRespEffect();
          currentScreen = SCREEN_CLOCK;
          drawClockScreenFull();
        } else {
          // Ciclo de focos START -> END -> CYCLE -> BUTTON -> START
          respFocus = (RespFocus)((respFocus + 1) % 4);
          drawSettingsRespScreen();
        }
      }

      if (btn2Falling) {
        saveConfigBasic();
        if (respEffectActive) {
          stopRespEffect();
        }
        // Volver a la lista de efectos
        currentScreen = SCREEN_SETTINGS_EFFECTS;
        drawSettingsEffectsScreen();
      }

      break;
    }

    case SCREEN_SETTINGS_COMET: {
      // Pantalla de configuración de COMETA

      if (stepDir != 0) {
        int dir = (stepDir > 0) ? 1 : -1;

        if (cometFocus == COMET_FOCUS_START) {
          int step = 5;
          cometKnobStartPos += dir * step;
          if (cometKnobStartPos < 0)   cometKnobStartPos = 0;
          if (cometKnobStartPos > 211) cometKnobStartPos = 211;

          uint8_t rr, gg, bb;
          cometColorStart = colorFromSliderEffects((uint8_t)cometKnobStartPos, rr, gg, bb);
          saveConfigBasic();
          drawSettingsCometScreen();
        }
        else if (cometFocus == COMET_FOCUS_END) {
          int step = 5;
          cometKnobEndPos += dir * step;
          if (cometKnobEndPos < 0)   cometKnobEndPos = 0;
          if (cometKnobEndPos > 211) cometKnobEndPos = 211;

          uint8_t rr2, gg2, bb2;
          cometColorEnd = colorFromSliderEffects((uint8_t)cometKnobEndPos, rr2, gg2, bb2);
          if (cometKnobEndPos >= 211) {
            rr2 = 255;
            gg2 = 255; 
            bb2 = 255;
            cometColorEnd = tft.color565(rr2, gg2, bb2);
          }
          saveConfigBasic();
          drawSettingsCometScreen();
        }
        else if (cometFocus == COMET_FOCUS_CYCLE) {
          int idx = (int)cometCycleIndex + dir;
          if (idx < 0) idx = 0;
          if (idx > 9) idx = 9;  // cometCycleTimesX10 tiene 10 entradas (0..9)

          if (idx != cometCycleIndex) {
            cometCycleIndex = (uint8_t)idx;
            saveConfigBasic();
            drawSettingsCometScreen();
          }
        }
        // COMET_FOCUS_BUTTON: el giro no hace nada
      }

      if (encButtonFalling) {
        if (cometFocus == COMET_FOCUS_BUTTON) {
          // Lanzar efecto COMETA y volver al reloj
          startCometEffect();
          currentScreen = SCREEN_CLOCK;
          drawClockScreenFull();
        } else {
          // Ciclo de focos START -> END -> CYCLE -> BUTTON -> START
          cometFocus = (CometFocus)((cometFocus + 1) % 4);
          drawSettingsCometScreen();
        }
      }

      if (btn2Falling) {
        saveConfigBasic();
        if (cometEffectActive) {
          stopCometEffect();
        }
        // Volver a la lista de efectos
        currentScreen = SCREEN_SETTINGS_EFFECTS;
        drawSettingsEffectsScreen();
      }

      break;
    }

    case SCREEN_SETTINGS_BARRIDO: {
      // Pantalla de configuración de BARRIDO

      if (stepDir != 0) {
        int dir = (stepDir > 0) ? 1 : -1;

        if (barridoFocus == BARRIDO_FOCUS_START) {
          int step = 5;
          barridoKnobStartPos += dir * step;
          if (barridoKnobStartPos < 0)   barridoKnobStartPos = 0;
          if (barridoKnobStartPos > 211) barridoKnobStartPos = 211;

          uint8_t rr, gg, bb;
          barridoColorStart = colorFromSliderEffects((uint8_t)barridoKnobStartPos, rr, gg, bb);
          saveConfigBasic();
          drawSettingsBarridoScreen();
        }
        else if (barridoFocus == BARRIDO_FOCUS_END) {
          int step = 5;
          barridoKnobEndPos += dir * step;
          if (barridoKnobEndPos < 0)   barridoKnobEndPos = 0;
          if (barridoKnobEndPos > 211) barridoKnobEndPos = 211;

          uint8_t rr2, gg2, bb2;
          barridoColorEnd = colorFromSliderEffects((uint8_t)barridoKnobEndPos, rr2, gg2, bb2);
          if (barridoKnobEndPos >= 211) {
            rr2 = 255;
            gg2 = 255; 
            bb2 = 255;
            barridoColorEnd = tft.color565(rr2, gg2, bb2);
          }
          saveConfigBasic();
          drawSettingsBarridoScreen();
        }
        else if (barridoFocus == BARRIDO_FOCUS_CYCLE) {
          int idx = (int)barridoCycleIndex + dir;
          if (idx < 0) idx = 0;
          if (idx >= BARRIDO_CYCLE_STEPS) idx = BARRIDO_CYCLE_STEPS - 1;

          if (idx != barridoCycleIndex) {
            barridoCycleIndex = (uint8_t)idx;
            saveConfigBasic();
            drawSettingsBarridoScreen();
          }
        }
        // BARRIDO_FOCUS_BUTTON: el giro no hace nada
      }

      if (encButtonFalling) {
        if (barridoFocus == BARRIDO_FOCUS_BUTTON) {
          // Lanzar efecto y volver al reloj
          startBarridoEffect();
          currentScreen = SCREEN_CLOCK;
          drawClockScreenFull();
        } else {
          // Ciclo de focos START -> END -> CYCLE -> BUTTON -> START
          barridoFocus = (BarridoFocus)((barridoFocus + 1) % 4);
          drawSettingsBarridoScreen();
        }
      }

      if (btn2Falling) {
        saveConfigBasic();
        if (barridoEffectActive) {
          stopBarridoEffect();
        }
        // Volver a la lista de efectos
        currentScreen = SCREEN_SETTINGS_EFFECTS;
        drawSettingsEffectsScreen();
      }

      break;
    }

    case SCREEN_SETTINGS_PERSIANA: {
      // Pantalla de configuración de PERSIANA

      if (stepDir != 0) {
        int dir = (stepDir > 0) ? 1 : -1;

        if (persianaFocus == PERSIANA_FOCUS_START) {
          int step = 5;
          persianaKnobStartPos += dir * step;
          if (persianaKnobStartPos < 0)   persianaKnobStartPos = 0;
          if (persianaKnobStartPos > 211) persianaKnobStartPos = 211;

          uint8_t rr, gg, bb;
          persianaColorStart = colorFromSliderEffects((uint8_t)persianaKnobStartPos, rr, gg, bb);
          saveConfigBasic();
          drawSettingsPersianaScreen();
        }
        else if (persianaFocus == PERSIANA_FOCUS_END) {
          int step = 5;
          persianaKnobEndPos += dir * step;
          if (persianaKnobEndPos < 0)   persianaKnobEndPos = 0;
          if (persianaKnobEndPos > 211) persianaKnobEndPos = 211;

          uint8_t rr2, gg2, bb2;
          persianaColorEnd = colorFromSliderEffects((uint8_t)persianaKnobEndPos, rr2, gg2, bb2);
          if (persianaKnobEndPos >= 211) {
            rr2 = 255;
            gg2 = 255; 
            bb2 = 255;
            persianaColorEnd = tft.color565(rr2, gg2, bb2);
          }
          saveConfigBasic();
          drawSettingsPersianaScreen();
        }
        else if (persianaFocus == PERSIANA_FOCUS_CYCLE) {
          int idx = (int)persianaCycleIndex + dir;
          if (idx < 0) idx = 0;
          if (idx >= PERSIANA_CYCLE_STEPS) idx = PERSIANA_CYCLE_STEPS - 1;

          if (idx != persianaCycleIndex) {
            persianaCycleIndex = (uint8_t)idx;
            saveConfigBasic();
            drawSettingsPersianaScreen();
          }
        }
        // PERSIANA_FOCUS_BUTTON: el giro no hace nada
      }

      if (encButtonFalling) {
        if (persianaFocus == PERSIANA_FOCUS_BUTTON) {
          // Lanzar efecto y volver al reloj
          startPersianaEffect();
          currentScreen = SCREEN_CLOCK;
          drawClockScreenFull();
        } else {
          // Ciclo de focos START -> END -> CYCLE -> BUTTON -> START
          persianaFocus = (PersianaFocus)((persianaFocus + 1) % 4);
          drawSettingsPersianaScreen();
        }
      }

      if (btn2Falling) {
        saveConfigBasic();
        if (persianaEffectActive) {
          stopPersianaEffect();
        }
        // Volver a la lista de efectos
        currentScreen = SCREEN_SETTINGS_EFFECTS;
        drawSettingsEffectsScreen();
      }

      break;
    }

    case SCREEN_SETTINGS_COLORS_DIGITAL: {
      if (stepDir != 0) {
        int delta = (stepDir > 0 ? 5 : -5);
        int v     = (int)colorPos + delta;
        if (v < 0)   v = 0;
        if (v > 255) v = 255;
        colorPos = (uint8_t)v;

        uint8_t  R,G,B;
        uint16_t newColor = colorFromSlider(colorPos, R, G, B);
        if (settingsColorDigitalIndex == 0) {
          digitalHMColor = newColor;
        } else {
          digitalDateColor = newColor;
        }
        drawSettingsColorsDigitalScreen();
      }

      if (encButtonFalling) {
        settingsColorDigitalIndex++;
        if (settingsColorDigitalIndex > 1) {
          saveConfigBasic();
          currentScreen = SCREEN_SETTINGS_CLOCK;
          drawSettingsClockScreen();
        } else {
          uint16_t c = (settingsColorDigitalIndex == 0) ? digitalHMColor : digitalDateColor;
          colorPos = sliderPosFromColor(c);
          drawSettingsColorsDigitalScreen();
        }
      }

      if (btn2Falling) {
        saveConfigBasic();
        currentScreen = SCREEN_SETTINGS_CLOCK;
        drawSettingsClockScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_COLORS_ANALOG: {
      if (stepDir != 0) {
        int delta = (stepDir > 0 ? 5 : -5);
        int v     = (int)colorPos + delta;
        if (v < 0)   v = 0;
        if (v > 255) v = 255;
        colorPos = (uint8_t)v;

        uint8_t  R,G,B;
        uint16_t newColor = colorFromSlider(colorPos, R, G, B);
        switch (settingsColorAnalogIndex) {
          case 0: analogHourHandColor = newColor; break;
          case 1: analogMinHandColor  = newColor; break;
          case 2: analogSecHandColor  = newColor; break;
          case 3: analogDateColor     = newColor; break;
          case 4: analogFaceFillColor = newColor; break;
        }
        drawSettingsColorsAnalogScreen();
      }

      if (encButtonFalling) {
        settingsColorAnalogIndex++;
        if (settingsColorAnalogIndex > 4) {
          saveConfigBasic();
          analogFaceDrawn      = false;
          lastSecond           = -1;
          forceFullClockRedraw = true;
          currentScreen        = SCREEN_SETTINGS_CLOCK;
          drawSettingsClockScreen();
        } else {
          uint16_t c = TFT_WHITE;
          switch (settingsColorAnalogIndex) {
            case 0: c = analogHourHandColor; break;
            case 1: c = analogMinHandColor;  break;
            case 2: c = analogSecHandColor;  break;
            case 3: c = analogDateColor;     break;
            case 4: c = analogFaceFillColor; break;
          }
          colorPos = sliderPosFromColor(c);
          drawSettingsColorsAnalogScreen();
        }
      }

      if (btn2Falling) {
        saveConfigBasic();
        analogFaceDrawn      = false;
        lastSecond           = -1;
        forceFullClockRedraw = true;
        currentScreen        = SCREEN_SETTINGS_CLOCK;
        drawSettingsClockScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_WIFI: {
      if (stepDir != 0) {
        wifiMenuIndex += (stepDir > 0 ? 1 : -1);
        if (wifiMenuIndex < 0) wifiMenuIndex = 2;
        if (wifiMenuIndex > 2) wifiMenuIndex = 0;
        drawSettingsWifiScreen();
      }

      if (encButtonFalling) {
        if (wifiMenuIndex == 0) {
          wifiScanIndex = 0;
          wifiScanDone  = false;
          currentScreen = SCREEN_SETTINGS_WIFI_SCAN;
          drawSettingsWifiScanScreen();
        } else if (wifiMenuIndex == 1) {
          wifiForgetCredentials();
          WiFi.disconnect(true, true);
          drawSettingsWifiScreen();
        } else if (wifiMenuIndex == 2) {
          currentScreen = SCREEN_SETTINGS_MAIN;
          drawSettingsMainScreen();
        }
      }

      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_MAIN;
        drawSettingsMainScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_WIFI_SCAN: {
      if (stepDir != 0 && wifiScanCount > 0) {
        wifiScanIndex += (stepDir > 0 ? 1 : -1);
        if (wifiScanIndex < 0)              wifiScanIndex = wifiScanCount - 1;
        if (wifiScanIndex >= wifiScanCount) wifiScanIndex = 0;
        drawSettingsWifiScanScreen();
      }

      if (encButtonFalling) {
        if (wifiScanCount > 0) {
          String sel = WiFi.SSID(wifiScanIndex);
          sel.toCharArray(wifiSsid, sizeof(wifiSsid));
          hasWifiCredentials = true;

          wifiPwdPlain[0]  = '\0';
          wifiKbIndex      = 0;
          wifiKbUppercase  = false;

          currentScreen = SCREEN_SETTINGS_WIFI_PWD;
          drawSettingsWifiPwdScreen();
        }
      }

      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_WIFI;
        drawSettingsWifiScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_WIFI_PWD: {
      const int maxIndex = 44;

      if (stepDir != 0) {
        wifiKbIndex += (stepDir > 0 ? 1 : -1);
        if (wifiKbIndex < 0)         wifiKbIndex = maxIndex;
        if (wifiKbIndex > maxIndex)  wifiKbIndex = 0;
        drawSettingsWifiPwdScreen();
      }

      if (encButtonFalling) {
        if (wifiKbIndex <= 39) {
          int cols = 10;
          int keyIndex = wifiKbIndex;
          int row = keyIndex / cols;
          int col = keyIndex % cols;

          char c = 0;
          if (row == 0) {
            int idx = col;
            c = wifiKbLetters[idx];
            if (wifiKbUppercase) c += ('A' - 'a');
          } else if (row == 1) {
            int idx = 10 + col;
            c = wifiKbLetters[idx];
            if (wifiKbUppercase) c += ('A' - 'a');
          } else if (row == 2) {
            if (col <= 5) {
              int idx = 20 + col;
              c = wifiKbLetters[idx];
              if (wifiKbUppercase) c += ('A' - 'a');
            } else {
              int d = col - 6;
              c = wifiKbDigits[d];
            }
          } else if (row == 3) {
            if (col <= 5) {
              int d = 4 + col;
              c = wifiKbDigits[d];
            } else if (col == 6) {
              c = '.';
            } else if (col == 7) {
              c = '-';
            } else {
              c = 0;
            }
          }

          if (c != 0) {
            int n = strlen(wifiPwdPlain);
            if (n < WIFI_PWD_BUF_LEN) {
              wifiPwdPlain[n]   = c;
              wifiPwdPlain[n+1] = '\0';
              drawSettingsWifiPwdScreen();
            }
          }
        } else {
          int g = wifiKbIndex - 40;
          if (g == 0) {          // ESP
            int n = strlen(wifiPwdPlain);
            if (n < WIFI_PWD_BUF_LEN) {
              wifiPwdPlain[n]   = ' ';
              wifiPwdPlain[n+1] = '\0';
              drawSettingsWifiPwdScreen();
            }
          } else if (g == 1) {   // A/a
            wifiKbUppercase = !wifiKbUppercase;
            drawSettingsWifiPwdScreen();
          } else if (g == 2) {   // DEL
            int n = strlen(wifiPwdPlain);
            if (n > 0) wifiPwdPlain[n-1] = '\0';
            drawSettingsWifiPwdScreen();
          } else if (g == 3) {   // OK
            char encBuf[WIFI_PWD_BUF_LEN + 1];
            wifiEncodePassword(wifiPwdPlain, encBuf, sizeof(encBuf));
            strcpy(wifiPwdEnc, encBuf);
            wifiSaveCredentials(wifiSsid, wifiPwdEnc);

            WiFi.disconnect(true, true);
            bool ok = wifiConnectUsingStored(10000);
            if (ok && useAutoTime) {
              setenv("TZ", TIMEZONES[tzIndex].tzStr, 1);
              tzset();
              configTzTime(TIMEZONES[tzIndex].tzStr, NTP_SERVER);
              lastNtpSyncMillis = millis();
            }

            currentScreen = SCREEN_SETTINGS_WIFI;
            drawSettingsWifiScreen();
          } else if (g == 4) {   // X
            currentScreen = SCREEN_SETTINGS_WIFI;
            drawSettingsWifiScreen();
          }
        }
      }

      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_WIFI;
        drawSettingsWifiScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_RESET_CONFIRM: {
      if (stepDir != 0) {
        resetConfirmIndex += (stepDir > 0 ? 1 : -1);
        if (resetConfirmIndex < 0) resetConfirmIndex = 1;
        if (resetConfirmIndex > 1) resetConfirmIndex = 0;
        drawSettingsResetConfirmScreen();
      }

      if (encButtonFalling) {
        if (resetConfirmIndex == 0) {
          if (prefs.begin("lamp_cfg", false)) {
            prefs.clear();
            prefs.end();
          }
          Preferences wp;
          if (wp.begin(WIFI_NAMESPACE, false)) {
            wp.clear();
            wp.end();
          }
          WiFi.disconnect(true, true);
          delay(200);
          ESP.restart();
        } else {
          currentScreen = SCREEN_SETTINGS_MAIN;
          drawSettingsMainScreen();
        }
      }

      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_MAIN;
        drawSettingsMainScreen();
      }
      break;
    }

    case SCREEN_SETTINGS_ABOUT: {
      if (btn2Falling || encButtonFalling) {
        currentScreen = SCREEN_SETTINGS_MAIN;
        drawSettingsMainScreen();
      }
      break;
    }

    default:
      break;
  }

  lastSw   = sw;
  lastBtn2 = btn2;

  if (lampOn && rainbowMode) {
    rainbowHue++;
    updateLeds();
    delay(20);
  }

  // --- Actualización de efectos (se ejecutan sólo si anyEffectActive == true) ---
  if (anyEffectActive && lampOn) {
    if (respEffectActive) {
      updateRespEffect();
    } else if (cometEffectActive) {
      updateCometEffect();
    } else if (barridoEffectActive) {
      updateBarridoEffect();
    }
    // futuros efectos: else if (otroEffectActive) update...
  }

}

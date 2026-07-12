// LAMP_Fun V.2.5.3
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
CRGB leds[NUM_LEDS];

void clearAllLedsAndShow() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
}

// Distribución de anillos de LEDs (orden físico 1..9 en la tira)
const int RING_COUNT = 9;
const int ringLen[RING_COUNT]   = { 60, 48, 40, 32, 24, 16, 12, 8, 1 };
const int ringStart[RING_COUNT] = {
  0,                     // Aro 1: LEDs 0..59
  60,                    // Aro 2: 60..107
  60 + 48,               // Aro 3: 108..147
  60 + 48 + 40,          // Aro 4: 148..179
  60 + 48 + 40 + 32,     // Aro 5: 180..203
  60 + 48 + 40 + 32 + 24,// Aro 6: 204..219
  60 + 48 + 40 + 32 + 24 + 16,     // Aro 7: 220..231
  60 + 48 + 40 + 32 + 24 + 16 + 12,// Aro 8: 232..239
  60 + 48 + 40 + 32 + 24 + 16 + 12 + 8 // Aro 9: 240
};

Preferences prefs;

bool lampOn = true;
bool rainbowMode = false;
uint8_t brightness = 5;
uint8_t redValue = 50;
uint8_t greenValue = 50;
uint8_t blueValue = 50;
uint8_t rainbowHue = 0;

// --------- Efecto RESPIRACION ---------

bool respEffectActive   = false;  // si el efecto está corriendo
bool respEffectForward  = true;   // subiendo (true) o bajando (false)
unsigned long respLastUpdate = 0; // último millis usado

// Intensidad actual 0..255 para interpolar entre color inicial/final
uint8_t respPhase = 0;

// Config persistente del efecto (guardada en NVS)
uint16_t respCfgColorIni565  = TFT_RED;   // por defecto si no hay NVS
uint16_t respCfgColorFin565  = TFT_BLUE;  // por defecto
uint8_t  respCfgCicloIndex   = 8;         // 1.0 s en RESP_CICLO_VALUES

// Estado de la pantalla de configuración RESPIRACION
enum RespiracionConfigStep {
  RESP_STEP_COLOR_INICIAL = 0,
  RESP_STEP_COLOR_FINAL,
  RESP_STEP_CICLO,
  RESP_STEP_INICIAR
};

RespiracionConfigStep respiracionStep = RESP_STEP_COLOR_INICIAL;

// Estado visual de colores para RESPIRACION (pantalla de config)
uint8_t respSliderPosInicial = 0;    // 0..255, posición knob izquierdo
uint8_t respSliderPosFinal   = 255;  // 0..255, posición knob derecho

uint16_t respColorInicialPreview = 0; // color actual del knob izquierdo (565)
uint16_t respColorFinalPreview   = 0; // color actual del knob derecho (565)

uint16_t respColorInicialSaved = 0;   // color confirmado para caja izquierda
uint16_t respColorFinalSaved   = 0;   // color confirmado para caja derecha

// Tabla de valores de ciclo en segundos
const int RESP_CICLO_COUNT = 28;
const float RESP_CICLO_VALUES[RESP_CICLO_COUNT] = {
  0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f,
  1.0f,
  2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f,
  10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f,
  17.0f, 18.0f, 19.0f, 20.0f
};

// Índice actual y valor visible del ciclo
int   respCicloIndex    = 8;          // 8 -> 1.0 s
float respCicloSegundos = 1.0f;       // sincronizado con RESP_CICLO_VALUES[respCicloIndex]

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

// --------- Efecto COMETA (V.2.5.3) ---------

// Estado de ejecución
bool cometaEffectActive   = false;
float cometaHeadPos       = 0.0f;      // 0.0..1.0 vuelta completa
float cometaLength        = 0.20f;     // fracción de vuelta ocupada por la cola
unsigned long cometaLastUpdate = 0;

// Config persistente del efecto (guardada en NVS)
uint16_t cometaCfgColorHead565 = TFT_BLACK; // color cabeza por defecto
uint16_t cometaCfgColorTail565 = TFT_WHITE; // color cola por defecto
uint8_t  cometaCfgCicloIndex   = 4;         // índice por defecto en COMETA_CICLO_VALUES

// Estado de la pantalla de configuración COMETA
enum CometaConfigStep {
  COMETA_STEP_COLOR_INICIAL = 0,
  COMETA_STEP_COLOR_FINAL,
  COMETA_STEP_CICLO,
  COMETA_STEP_INICIAR
};

CometaConfigStep cometaStep = COMETA_STEP_COLOR_INICIAL;

// Estado visual de sliders para COMETA
uint8_t  cometaSliderPosInicial = 0;   // 0..255, knob izquierdo (cabeza)
uint8_t  cometaSliderPosFinal   = 255; // 0..255, knob derecho (cola)

uint16_t cometaColorHeadPreview = 0;   // color actual del knob cabeza
uint16_t cometaColorTailPreview = 0;   // color actual del knob cola

uint16_t cometaColorHeadSaved   = 0;   // color confirmado para caja izquierda
uint16_t cometaColorTailSaved   = 0;   // color confirmado para caja derecha

// Tabla de valores de ciclo (segundos por vuelta completa)
const int COMETA_CICLO_COUNT = 12;
const float COMETA_CICLO_VALUES[COMETA_CICLO_COUNT] = {
  0.3f, 0.5f, 0.7f,
  1.0f, 1.5f, 2.0f,
  3.0f, 4.0f, 5.0f,
  7.0f, 8.0f, 10.0f
};

// Índice actual y valor visible del ciclo
int   cometaCicloIndex    = 4;         // por defecto 1.5 s por vuelta
float cometaCicloSegundos = 1.5f;

// --------- Efecto CUADRANTE (V.2.5.3) ---------

// Estado de ejecución
bool cuadranteEffectActive   = false;
unsigned long cuadranteLastUpdate = 0;

// Índice del cuadrante actual (0..3: 1º, 2º, 3º, 4º)
uint8_t cuadranteIndex = 0;

// Progreso radial dentro del cuadrante actual (0.0 = centro, 1.0 = aro exterior)
float cuadranteProgress = 0.0f;

// Config persistente del efecto (guardada en NVS)
// color inicial = color de la "cabeza" que sale del centro
// color final   = color en el exterior antes de apagarse
uint16_t cuadranteCfgColorIni565 = TFT_BLACK;
uint16_t cuadranteCfgColorFin565 = TFT_WHITE;
uint8_t  cuadranteCfgCicloIndex  = 4;  // índice por defecto en CUADRANTE_CICLO_VALUES

// Estado de la pantalla de configuración CUADRANTE
enum CuadranteConfigStep {
  CUAD_STEP_COLOR_INICIAL = 0,
  CUAD_STEP_COLOR_FINAL,
  CUAD_STEP_CICLO,
  CUAD_STEP_INICIAR
};

CuadranteConfigStep cuadranteStep = CUAD_STEP_COLOR_INICIAL;

// Estado visual de sliders para CUADRANTE
uint8_t  cuadranteSliderPosInicial = 0;   // 0..255, knob izquierdo (color inicial)
uint8_t  cuadranteSliderPosFinal   = 255; // 0..255, knob derecho  (color final)

uint16_t cuadranteColorIniPreview  = 0;   // color actual del knob inicial
uint16_t cuadranteColorFinPreview  = 0;   // color actual del knob final

uint16_t cuadranteColorIniSaved    = 0;   // color confirmado para caja izquierda
uint16_t cuadranteColorFinSaved    = 0;   // color confirmado para caja derecha

// Tabla de valores de ciclo (segundos por recorrido completo de 4 cuadrantes)
// NUMÉRICAMENTE iguales a COMETA, pero independientes
const int CUADRANTE_CICLO_COUNT = 12;
const float CUADRANTE_CICLO_VALUES[CUADRANTE_CICLO_COUNT] = {
  0.3f, 0.5f, 0.7f,
  1.0f, 1.5f, 2.0f,
  3.0f, 4.0f, 5.0f,
  7.0f, 8.0f, 10.0f
};

// Índice actual y valor visible del ciclo
int   cuadranteCicloIndex    = 4;
float cuadranteCicloSegundos = 1.5f;

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
  SCREEN_SETTINGS_LAMP,
  SCREEN_SETTINGS_LAMP_CONFIG,
  SCREEN_SETTINGS_COMETA_CONFIG,
  SCREEN_SETTINGS_CUADRANTE_CONFIG,
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

    // ... ya lees R,G,B, brillo, formatos, colores de reloj, etc.

  // --- Config efecto RESPIRACION ---
  respCfgColorIni565 = prefs.getUShort("respIni", TFT_RED);
  respCfgColorFin565 = prefs.getUShort("respFin", TFT_BLUE);
  respCfgCicloIndex  = (uint8_t)prefs.getUChar("respCix", 8);

  if (respCfgCicloIndex >= RESP_CICLO_COUNT) {
    respCfgCicloIndex = 8;
  }

  // Config COMETA (valores por defecto si no existen aún en NVS)
  cometaCfgColorHead565 = prefs.getUShort("cmHead", TFT_BLACK);
  cometaCfgColorTail565 = prefs.getUShort("cmTail", TFT_WHITE);
  cometaCfgCicloIndex   = prefs.getUChar ("cmCIdx", 4);
  if (cometaCfgCicloIndex >= COMETA_CICLO_COUNT) cometaCfgCicloIndex = 4;

  // Config CUADRANTE (independiente de COMETA)
  cuadranteCfgColorIni565 = prefs.getUShort("cqIni", TFT_BLACK);
  cuadranteCfgColorFin565 = prefs.getUShort("cqFin", TFT_WHITE);
  cuadranteCfgCicloIndex  = prefs.getUChar ("cqCIdx", 4);
  if (cuadranteCfgCicloIndex >= CUADRANTE_CICLO_COUNT) cuadranteCfgCicloIndex = 4;

  prefs.end();

  if (tftBacklightLevel > 100) tftBacklightLevel = 100;
  if (clockMode > 1) clockMode = 0;

  if (tzIndex < 0 || tzIndex >= TIMEZONES_COUNT) tzIndex = 1;
  if (tzOffsetSteps < -4) tzOffsetSteps = -4;
  if (tzOffsetSteps > 4)  tzOffsetSteps = 4;

  // Inicializar valor visible del ciclo COMETA a partir del índice cargado
  cometaCicloIndex    = cometaCfgCicloIndex;
  if (cometaCicloIndex >= COMETA_CICLO_COUNT) cometaCicloIndex = 4;
  cometaCicloSegundos = COMETA_CICLO_VALUES[cometaCicloIndex];

  cuadranteCicloIndex = cuadranteCfgCicloIndex;
  if (cuadranteCicloIndex >= CUADRANTE_CICLO_COUNT) cuadranteCicloIndex = 4;
  cuadranteCicloSegundos = CUADRANTE_CICLO_VALUES[cuadranteCicloIndex];

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

  // --- Config efecto RESPIRACION ---
  prefs.putUShort("respIni", respCfgColorIni565);
  prefs.putUShort("respFin", respCfgColorFin565);
  prefs.putUChar ("respCix", respCfgCicloIndex);

  // Config COMETA
  prefs.putUShort("cmHead", cometaCfgColorHead565);
  prefs.putUShort("cmTail", cometaCfgColorTail565);
  prefs.putUChar ("cmCIdx", cometaCfgCicloIndex);

  // Config CUADRANTE
  prefs.putUShort("cqIni", cuadranteCfgColorIni565);
  prefs.putUShort("cqFin", cuadranteCfgColorFin565);
  prefs.putUChar ("cqCIdx", cuadranteCfgCicloIndex);

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

// Convierte color 565 a componentes RGB 8 bits
void color565ToRGB(uint16_t c, uint8_t &r, uint8_t &g, uint8_t &b) {
  r = ((c >> 11) & 0x1F) << 3;
  g = ((c >> 5)  & 0x3F) << 2;
  b = (c & 0x1F) << 3;
}

// Interpola linealmente entre dos colores 565, t=0..255
uint16_t lerpColor565(uint16_t c1, uint16_t c2, uint8_t t) {
  uint8_t r1, g1, b1, r2, g2, b2;
  color565ToRGB(c1, r1, g1, b1);
  color565ToRGB(c2, r2, g2, b2);

  uint8_t r = (uint8_t)(((uint16_t)r1 * (255 - t) + (uint16_t)r2 * t) / 255);
  uint8_t g = (uint8_t)(((uint16_t)g1 * (255 - t) + (uint16_t)g2 * t) / 255);
  uint8_t b = (uint8_t)(((uint16_t)b1 * (255 - t) + (uint16_t)b2 * t) / 255);

  return tft.color565(r, g, b);
}

// Inicia el efecto RESPIRACION
void startRespEffect() {
  respEffectActive  = true;
  respEffectForward = true;
  respPhase         = 0;
  respLastUpdate    = millis();
}

// Detiene el efecto RESPIRACION
void stopRespEffect() {
  respEffectActive = false;
  clearAllLedsAndShow();
  updateLeds();
}

// Inicia el efecto COMETA
void startCometaEffect() {
  // Asegurarse de que otros efectos de LEDs están parados
  respEffectActive = false;
  rainbowMode      = false;

  cometaEffectActive = true;
  cometaHeadPos      = 0.0f;
  cometaLastUpdate   = millis();

  // Opcional: longitud de cola fija o dependiente del ciclo
  cometaLength = 0.25f; // 25% de la vuelta. Ajustable si quieres más corta/larga.
}

// Detiene el efecto COMETA y restaura la luz fija
void stopCometaEffect() {
  cometaEffectActive = false;
  clearAllLedsAndShow();
  updateLeds();
}

void startCuadranteEffect() {
  // Asegurarse de que otros efectos de LEDs están parados
  respEffectActive    = false;
  cometaEffectActive  = false;
  rainbowMode         = false;

  cuadranteEffectActive = true;
  cuadranteLastUpdate   = millis();

  // Empezamos por el primer cuadrante (entre 12 y 3)
  cuadranteIndex    = 0;
  cuadranteProgress = 0.0f;
}

void stopCuadranteEffect() {
  cuadranteEffectActive = false;
  clearAllLedsAndShow();
  updateLeds();
}

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
  tft.drawString("V.2.5.3", 120, 85);

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

int settingsColorDigitalIndex = 0;
int settingsColorAnalogIndex  = 0;

int settingsTzIndexTemp  = 0;
int settingsTzOffsetTemp = 0;

int resetConfirmIndex = 1; // 0 = SI, 1 = NO

// Submenú Efectos
int settingsLampIndex = 0;
const int SETTINGS_LAMP_ITEMS = 1; // de momento solo RESPIRACION

int efectosMenuIndex = 0;   // 0 = RESPIRACION, 1 = COMETA, 2 = CUADRANTE

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

// ---------- Configuración efecto RESPIRACION ----------

void drawRespiracionConfigScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  // Cabecera
  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("RESPIRACION", 120, 15);

  drawWifiSignalIcon();

  // --- Slider de color (similar al de colores de reloj) ---

  int sx = 20;
  int sy = 70;
  int sw = 200;
  int sh = 20;

  // Marco del slider
  tft.drawRect(sx, sy, sw, sh, TFT_WHITE);
  tft.fillRect(sx + 1, sy + 1, sw - 2, sh - 2, TFT_BLACK);

  // Paleta completa dentro del slider
  for (int x = 0; x < sw; x++) {
    float    ratio = (float)x / (float)(sw - 1);
    uint8_t  pos   = (uint8_t)roundf(ratio * 255.0f);
    uint8_t  r,g,b;
    uint16_t c     = colorFromSlider(pos, r, g, b);
    tft.drawFastVLine(sx + x, sy + 1, sh - 2, c);
  }

  // Barritas verticales en los extremos + etiquetas N y B

  int markHeight = 12;
  int markYTop   = sy - markHeight - 6;

  // Extremo izquierdo = color inicial (N)
  int xLeftMark = sx;
  tft.drawLine(xLeftMark, markYTop, xLeftMark, markYTop + markHeight, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("N", xLeftMark, markYTop - 12);

  // Extremo derecho = color final (B)
  int xRightMark = sx + sw - 1;
  tft.drawLine(xRightMark, markYTop, xRightMark, markYTop + markHeight, TFT_WHITE);
  tft.drawString("B", xRightMark, markYTop - 12);

  // --- Knobs tipo "barra blanca + bolita de color" ---

  // Posiciones de knobs en coordenadas X
  int knobXInicial = sx + 1 + (int)((respSliderPosInicial / 255.0f) * (sw - 4));
  if (knobXInicial < sx + 1)       knobXInicial = sx + 1;
  if (knobXInicial > sx + sw - 3)  knobXInicial = sx + sw - 3;

  int knobXFinal = sx + 1 + (int)((respSliderPosFinal / 255.0f) * (sw - 4));
  if (knobXFinal < sx + 1)       knobXFinal = sx + 1;
  if (knobXFinal > sx + sw - 3)  knobXFinal = sx + sw - 3;

  // Colores preview de cada knob
  uint8_t rN, gN, bN;
  uint8_t rB, gB, bB;
  respColorInicialPreview = colorFromSlider(respSliderPosInicial, rN, gN, bN);
  respColorFinalPreview   = colorFromSlider(respSliderPosFinal,   rB, gB, bB);

  // Solo bolitas de color, sin barra vertical
  int knobBallR = 7;                       // tamaño similar al de los sliders de reloj
  int knobBallY = sy - knobBallR - 5;     // unos píxeles por encima del borde superior

  // Knob inicial
  tft.fillCircle(knobXInicial, knobBallY, knobBallR, respColorInicialPreview);
  tft.drawCircle(knobXInicial, knobBallY, knobBallR, TFT_WHITE);

  // Knob final
  tft.fillCircle(knobXFinal, knobBallY, knobBallR, respColorFinalPreview);
  tft.drawCircle(knobXFinal, knobBallY, knobBallR, TFT_WHITE);

  // --- Valores RGB del color activo, en tamaño más grande ---

  uint8_t rSel = 0, gSel = 0, bSel = 0;
  if (respiracionStep == RESP_STEP_COLOR_FINAL) {
    rSel = rB; gSel = gB; bSel = bB;
  } else {
    rSel = rN; gSel = gN; bSel = bN;
  }

  tft.setTextSize(2);                  // tamaño más grande
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);

  char buf[32];
  int rgbY = sy + sh + 20;

  char partR[8], partG[8], partB[8];
  snprintf(partR, sizeof(partR), "R:%d", rSel);
  snprintf(partG, sizeof(partG), "G:%d", gSel);
  snprintf(partB, sizeof(partB), "B:%d", bSel);

  snprintf(buf, sizeof(buf), "%s %s %s", partR, partG, partB);
  tft.drawString(buf, 120, rgbY);

  // --- Cajas de color inicial y final ---

  int boxY    = rgbY + 26;
  int boxW    = 80;
  int boxH    = 24;
  int boxGap  = 20;

  //Centrar las dos cajas en la pantalla
  int totalBoxesWidth = boxW * 2 + boxGap;
  int boxXIni = (240 - totalBoxesWidth) /2;
  int boxXFin = boxXIni + boxW + boxGap;

  // Fondo de ambas cajas
  tft.fillRect(boxXIni, boxY, boxW, boxH, TFT_BLACK);
  tft.fillRect(boxXFin, boxY, boxW, boxH, TFT_BLACK);

  // Relleno con colores guardados (de momento, usamos preview como placeholder)
  uint16_t cIni = (respColorInicialSaved != 0) ? respColorInicialSaved : respColorInicialPreview;
  uint16_t cFin = (respColorFinalSaved   != 0) ? respColorFinalSaved   : respColorFinalPreview;

  tft.fillRect(boxXIni + 1, boxY + 1, boxW - 2, boxH - 2, cIni);
  tft.fillRect(boxXFin + 1, boxY + 1, boxW - 2, boxH - 2, cFin);

  tft.drawRect(boxXIni, boxY, boxW, boxH, TFT_WHITE);
  tft.drawRect(boxXFin, boxY, boxW, boxH, TFT_WHITE);

  // Flechitas indicando foco (aún solo dibujamos según respiracionStep)

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  if (respiracionStep == RESP_STEP_COLOR_INICIAL) {
    // Flecha -> a la izquierda de la caja inicial
    tft.setTextDatum(MR_DATUM);
    tft.drawString(">", boxXIni - 4, boxY + boxH / 2);
  } else if (respiracionStep == RESP_STEP_COLOR_FINAL) {
    // Flecha <- a la derecha de la caja final
    tft.setTextDatum(ML_DATUM);
    tft.drawString("<", boxXFin + boxW + 4, boxY + boxH / 2);
  } else {
    // En otros pasos (CICLO, INICIAR) no dibujamos flechas en estas cajas
  }

  // --- Línea de Ciclo: X.X s ---

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int cicloY = boxY + boxH + 26;
  char bufC[24];
  snprintf(bufC, sizeof(bufC), "Ciclo: %.1f s", respCicloSegundos);

  // Flecha de foco cuando estamos en RESP_STEP_CICLO
  if (respiracionStep == RESP_STEP_CICLO) {
    // Flecha a la izquierda
    tft.setTextDatum(MR_DATUM);
    tft.drawString(">", 40, cicloY);  // X fijo a la izquierda
  }

  // Texto centrado
  tft.setTextDatum(MC_DATUM);
  tft.drawString(bufC, 120, cicloY);

  // --- Botón "Iniciar" en la parte baja ---

  int btnW = 120;
  int btnH = 32;
  int btnX = (240 - btnW) / 2;
  int btnY = 200;  // ajusta si quieres más arriba/abajo

  bool selIniciar = (respiracionStep == RESP_STEP_INICIAR);

  uint16_t bgBtn = selIniciar ? TFT_DARKGREY : TFT_BLACK;
  uint16_t fgBtn = selIniciar ? TFT_NAVY     : TFT_WHITE;

  tft.fillRect(btnX, btnY, btnW, btnH, bgBtn);
  tft.setTextColor(fgBtn, bgBtn);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Iniciar", btnX + btnW / 2, btnY + btnH / 2);

}

void drawCometaConfigScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  // Cabecera
  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("COMETA", 120, 15);

  drawWifiSignalIcon();

  // --- Slider de color (igual que RESPIRACION) ---

  int sx = 20;
  int sy = 70;
  int sw = 200;
  int sh = 20;

  // Marco del slider
  tft.drawRect(sx, sy, sw, sh, TFT_WHITE);
  tft.fillRect(sx + 1, sy + 1, sw - 2, sh - 2, TFT_BLACK);

  // Paleta completa dentro del slider
  for (int x = 0; x < sw; x++) {
    float    ratio = (float)x / (float)(sw - 1);
    uint8_t  pos   = (uint8_t)roundf(ratio * 255.0f);
    uint8_t  r,g,b;
    uint16_t c     = colorFromSlider(pos, r, g, b);
    tft.drawFastVLine(sx + x, sy + 1, sh - 2, c);
  }

  // Marcas N/B
  int markHeight = 12;
  int markYTop   = sy - markHeight - 6;

  int xLeftMark = sx;
  tft.drawLine(xLeftMark, markYTop, xLeftMark, markYTop + markHeight, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("N", xLeftMark, markYTop - 12);

  int xRightMark = sx + sw - 1;
  tft.drawLine(xRightMark, markYTop, xRightMark, markYTop + markHeight, TFT_WHITE);
  tft.drawString("B", xRightMark, markYTop - 12);

  // --- Knobs ---

  int knobXInicial = sx + 1 + (int)((cometaSliderPosInicial / 255.0f) * (sw - 4));
  if (knobXInicial < sx + 1)       knobXInicial = sx + 1;
  if (knobXInicial > sx + sw - 3)  knobXInicial = sx + sw - 3;

  int knobXFinal = sx + 1 + (int)((cometaSliderPosFinal / 255.0f) * (sw - 4));
  if (knobXFinal < sx + 1)       knobXFinal = sx + 1;
  if (knobXFinal > sx + sw - 3)  knobXFinal = sx + sw - 3;

  uint8_t rN, gN, bN;
  uint8_t rB, gB, bB;
  cometaColorHeadPreview = colorFromSlider(cometaSliderPosInicial, rN, gN, bN);
  cometaColorTailPreview = colorFromSlider(cometaSliderPosFinal,   rB, gB, bB);

  int knobBallR = 7;
  int knobBallY = sy - knobBallR - 5;

  tft.fillCircle(knobXInicial, knobBallY, knobBallR, cometaColorHeadPreview);
  tft.drawCircle(knobXInicial, knobBallY, knobBallR, TFT_WHITE);

  tft.fillCircle(knobXFinal, knobBallY, knobBallR, cometaColorTailPreview);
  tft.drawCircle(knobXFinal, knobBallY, knobBallR, TFT_WHITE);

  // --- Valores RGB del color activo ---

  uint8_t rSel = 0, gSel = 0, bSel = 0;

  // Qeremos que, una vez hemos pasado por COLOR_FINAL,
  // tanto en CICLO como en INICIAR se siga mostrando el color final.
  if (cometaStep == COMETA_STEP_COLOR_FINAL ||
      cometaStep == COMETA_STEP_CICLO ||
      cometaStep == COMETA_STEP_INICIAR) {
    rSel = rB; gSel = gB; bSel = bB;
  } else { // COMETA_STEP_COLOR_INICIAL
    rSel = rN; gSel = gN; bSel = bN;
  }

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);

  char buf[32];
  int rgbY = sy + sh + 20;

  char partR[8], partG[8], partB[8];
  snprintf(partR, sizeof(partR), "R:%d", rSel);
  snprintf(partG, sizeof(partG), "G:%d", gSel);
  snprintf(partB, sizeof(partB), "B:%d", bSel);

  snprintf(buf, sizeof(buf), "%s %s %s", partR, partG, partB);
  tft.drawString(buf, 120, rgbY);

  // --- Cajas de color cabeza y cola ---

  int boxY    = rgbY + 26;
  int boxW    = 80;
  int boxH    = 24;
  int boxGap  = 20;

  int totalBoxesWidth = boxW * 2 + boxGap;
  int boxXIni = (240 - totalBoxesWidth) / 2;
  int boxXFin = boxXIni + boxW + boxGap;

  tft.fillRect(boxXIni, boxY, boxW, boxH, TFT_BLACK);
  tft.fillRect(boxXFin, boxY, boxW, boxH, TFT_BLACK);

  uint16_t cIni = (cometaColorHeadSaved != 0) ? cometaColorHeadSaved : cometaColorHeadPreview;
  uint16_t cFin = (cometaColorTailSaved != 0) ? cometaColorTailSaved : cometaColorTailPreview;

  tft.fillRect(boxXIni + 1, boxY + 1, boxW - 2, boxH - 2, cIni);
  tft.fillRect(boxXFin + 1, boxY + 1, boxW - 2, boxH - 2, cFin);

  tft.drawRect(boxXIni, boxY, boxW, boxH, TFT_WHITE);
  tft.drawRect(boxXFin, boxY, boxW, boxH, TFT_WHITE);

  // Flechas de foco
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  if (cometaStep == COMETA_STEP_COLOR_INICIAL) {
    tft.setTextDatum(MR_DATUM);
    tft.drawString(">", boxXIni - 4, boxY + boxH / 2);
  } else if (cometaStep == COMETA_STEP_COLOR_FINAL) {
    tft.setTextDatum(ML_DATUM);
    tft.drawString("<", boxXFin + boxW + 4, boxY + boxH / 2);
  }

  // --- Línea de Ciclo ---

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int cicloY = boxY + boxH + 26;
  char bufC[24];
  snprintf(bufC, sizeof(bufC), "Ciclo: %.1f s", cometaCicloSegundos);

  if (cometaStep == COMETA_STEP_CICLO) {
    tft.setTextDatum(MR_DATUM);
    tft.drawString(">", 40, cicloY);
  }

  tft.setTextDatum(MC_DATUM);
  tft.drawString(bufC, 120, cicloY);

  // --- Botón "Iniciar" ---

  int btnW = 120;
  int btnH = 32;
  int btnX = (240 - btnW) / 2;
  int btnY = 200;

  bool selIniciar = (cometaStep == COMETA_STEP_INICIAR);

  uint16_t bgBtn = selIniciar ? TFT_DARKGREY : TFT_BLACK;
  uint16_t fgBtn = selIniciar ? TFT_NAVY     : TFT_WHITE;

  tft.fillRect(btnX, btnY, btnW, btnH, bgBtn);
  tft.setTextColor(fgBtn, bgBtn);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Iniciar", btnX + btnW / 2, btnY + btnH / 2);
}

void drawCuadranteConfigScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars    = -1;
  lastWifiTachado = false;

  // Cabecera
  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("CUADRANTE", 120, 15);

  drawWifiSignalIcon();

  // --- Slider de color (igual que RESPIRACION/COMETA) ---

  int sx = 20;
  int sy = 70;
  int sw = 200;
  int sh = 20;

  // Marco del slider
  tft.drawRect(sx, sy, sw, sh, TFT_WHITE);
  tft.fillRect(sx + 1, sy + 1, sw - 2, sh - 2, TFT_BLACK);

  // Paleta completa dentro del slider
  for (int x = 0; x < sw; x++) {
    float    ratio = (float)x / (float)(sw - 1);
    uint8_t  pos   = (uint8_t)roundf(ratio * 255.0f);
    uint8_t  r,g,b;
    uint16_t c     = colorFromSlider(pos, r, g, b);
    tft.drawFastVLine(sx + x, sy + 1, sh - 2, c);
  }

  // Marcas N/B
  int markHeight = 12;
  int markYTop   = sy - markHeight - 6;

  int xLeftMark = sx;
  tft.drawLine(xLeftMark, markYTop, xLeftMark, markYTop + markHeight, TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("N", xLeftMark, markYTop - 12);

  int xRightMark = sx + sw - 1;
  tft.drawLine(xRightMark, markYTop, xRightMark, markYTop + markHeight, TFT_WHITE);
  tft.drawString("B", xRightMark, markYTop - 12);

  // --- Knobs ---

  int knobXInicial = sx + 1 + (int)((cuadranteSliderPosInicial / 255.0f) * (sw - 4));
  if (knobXInicial < sx + 1)       knobXInicial = sx + 1;
  if (knobXInicial > sx + sw - 3)  knobXInicial = sx + sw - 3;

  int knobXFinal = sx + 1 + (int)((cuadranteSliderPosFinal / 255.0f) * (sw - 4));
  if (knobXFinal < sx + 1)       knobXFinal = sx + 1;
  if (knobXFinal > sx + sw - 3)  knobXFinal = sx + sw - 3;

  uint8_t rN, gN, bN;
  uint8_t rB, gB, bB;
  cuadranteColorIniPreview = colorFromSlider(cuadranteSliderPosInicial, rN, gN, bN);
  cuadranteColorFinPreview = colorFromSlider(cuadranteSliderPosFinal,   rB, gB, bB);

  int knobBallR = 7;
  int knobBallY = sy - knobBallR - 5;

  tft.fillCircle(knobXInicial, knobBallY, knobBallR, cuadranteColorIniPreview);
  tft.drawCircle(knobXInicial, knobBallY, knobBallR, TFT_WHITE);

  tft.fillCircle(knobXFinal, knobBallY, knobBallR, cuadranteColorFinPreview);
  tft.drawCircle(knobXFinal, knobBallY, knobBallR, TFT_WHITE);

  // --- Valores RGB del color activo ---

  uint8_t rSel = 0, gSel = 0, bSel = 0;

  if (cuadranteStep == CUAD_STEP_COLOR_FINAL ||
      cuadranteStep == CUAD_STEP_CICLO ||
      cuadranteStep == CUAD_STEP_INICIAR) {
    rSel = rB; gSel = gB; bSel = bB;
  } else {
    rSel = rN; gSel = gN; bSel = bN;
  }

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);

  char buf[32];
  int rgbY = sy + sh + 20;

  char partR[8], partG[8], partB[8];
  snprintf(partR, sizeof(partR), "R:%d", rSel);
  snprintf(partG, sizeof(partG), "G:%d", gSel);
  snprintf(partB, sizeof(partB), "B:%d", bSel);

  snprintf(buf, sizeof(buf), "%s %s %s", partR, partG, partB);
  tft.drawString(buf, 120, rgbY);

  // --- Cajas de color inicial y final ---

  int boxY    = rgbY + 26;
  int boxW    = 80;
  int boxH    = 24;
  int boxGap  = 20;

  int totalBoxesWidth = boxW * 2 + boxGap;
  int boxXIni = (240 - totalBoxesWidth) / 2;
  int boxXFin = boxXIni + boxW + boxGap;

  tft.fillRect(boxXIni, boxY, boxW, boxH, TFT_BLACK);
  tft.fillRect(boxXFin, boxY, boxW, boxH, TFT_BLACK);

  uint16_t cIni = (cuadranteColorIniSaved != 0) ? cuadranteColorIniSaved : cuadranteColorIniPreview;
  uint16_t cFin = (cuadranteColorFinSaved != 0) ? cuadranteColorFinSaved : cuadranteColorFinPreview;

  tft.fillRect(boxXIni + 1, boxY + 1, boxW - 2, boxH - 2, cIni);
  tft.fillRect(boxXFin + 1, boxY + 1, boxW - 2, boxH - 2, cFin);

  tft.drawRect(boxXIni, boxY, boxW, boxH, TFT_WHITE);
  tft.drawRect(boxXFin, boxY, boxW, boxH, TFT_WHITE);

  // Flechas de foco
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  if (cuadranteStep == CUAD_STEP_COLOR_INICIAL) {
    tft.setTextDatum(MR_DATUM);
    tft.drawString(">", boxXIni - 4, boxY + boxH / 2);
  } else if (cuadranteStep == CUAD_STEP_COLOR_FINAL) {
    tft.setTextDatum(ML_DATUM);
    tft.drawString("<", boxXFin + boxW + 4, boxY + boxH / 2);
  }

  // --- Línea de Ciclo ---

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int cicloY = boxY + boxH + 26;
  char bufC[24];
  snprintf(bufC, sizeof(bufC), "Ciclo: %.1f s", cuadranteCicloSegundos);

  if (cuadranteStep == CUAD_STEP_CICLO) {
    tft.setTextDatum(MR_DATUM);
    tft.drawString(">", 40, cicloY);
  }

  tft.setTextDatum(MC_DATUM);
  tft.drawString(bufC, 120, cicloY);

  // --- Botón "Iniciar" ---

  int btnW = 120;
  int btnH = 32;
  int btnX = (240 - btnW) / 2;
  int btnY = 200;

  bool selIniciar = (cuadranteStep == CUAD_STEP_INICIAR);

  uint16_t bgBtn = selIniciar ? TFT_DARKGREY : TFT_BLACK;
  uint16_t fgBtn = selIniciar ? TFT_NAVY     : TFT_WHITE;

  tft.fillRect(btnX, btnY, btnW, btnH, bgBtn);
  tft.setTextColor(fgBtn, bgBtn);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Iniciar", btnX + btnW / 2, btnY + btnH / 2);
}

// ---------- Submenú Efectos ----------

void drawSettingsLampScreen() {
  tft.fillScreen(TFT_BLACK);
  lastWifiBars = -1;
  lastWifiTachado = false;

  tft.fillRect(0, 0, 240, 30, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Efectos luz", 120, 15);

  drawWifiSignalIcon();

  tft.setTextSize(2);
  tft.setTextDatum(TL_DATUM);

  const char* lines[] = {
    "RESPIRACION",
    "COMETA",
    "CUADRANTE"
  };
  const int items = 3;

  int startY = 60;
  int lineH  = 24;

  for (int i = 0; i < items; i++) {
    int y = startY + i * lineH;
    bool selected = (i == efectosMenuIndex);

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

uint8_t sliderPosFromColor(uint16_t c) {
  uint8_t r,g,b;
  rgbFrom565(c, r, g, b);
  uint8_t maxc = max(r, max(g,b));
  uint8_t minc = min(r, min(g,b));

  // Casos exactos: negro y blanco puros
  if (r == 0 && g == 0 && b == 0)   return 0;   // negro puro -> extremo izquierdo
  if (r == 255 && g == 255 && b == 255) return 255; // blanco puro -> extremo derecho

  //Casos "casi" negro/blanco, como ya tenías
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

  tft.drawString("LAMP_Fun V.2.5.3",     120, y); y += dy + 4;
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
  Serial.println("LAMP_Fun V2.5.1");

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

      // 1) Giro de encoder o pulsación encoder -> parar RESPIRACION y pasar a LUZ
      if (stepDir != 0 || encButtonFalling) {
        if (respEffectActive) {
          stopRespEffect();   // apaga el efecto y deja los LEDs en el RGB de Luz
        }
        if (cometaEffectActive) {
          stopCometaEffect();
        }
        if (cuadranteEffectActive){
          stopCuadranteEffect();
        }
        currentScreen = SCREEN_LIGHT;
        currentControl = CTRL_BTN_POWER;
        editingBar = false;
        drawLightScreen();
      }

      // 2) Pulsador 2 -> parar RESPIRACION y pasar a Ajustes
      if (btn2Falling) {
        if (respEffectActive) {
          stopRespEffect(); // apaga el efecto y deja los LEDs en el RGB de Luz
        }
        if (cometaEffectActive) {
          stopCometaEffect();
        }
        if (cuadranteEffectActive) {
          stopCuadranteEffect();
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
        currentScreen = SCREEN_CLOCK;
        drawClockScreenFull();
      }
      break;
    }

    case SCREEN_SETTINGS_MAIN: {
      if (stepDir != 0) {
        settingsMainIndex += (stepDir > 0 ? 1 : -1);
        if (settingsMainIndex < 0)                    settingsMainIndex = SETTINGS_MAIN_ITEMS - 1;
        if (settingsMainIndex >= SETTINGS_MAIN_ITEMS) settingsMainIndex = 0;
        drawSettingsMainScreen();
      }

      if (encButtonFalling) {
        switch (settingsMainIndex) {
          case 0:
            settingsClockIndex = 0;
            currentScreen      = SCREEN_SETTINGS_CLOCK;
            drawSettingsClockScreen();
            break;
          case 1:
            currentScreen = SCREEN_SETTINGS_BACKLIGHT;
            drawSettingsBacklightScreen();
            break;
          case 2:
            efectosMenuIndex = 0;
            currentScreen = SCREEN_SETTINGS_LAMP;
            drawSettingsLampScreen();
            break;
          case 3:
            wifiMenuIndex = 0;
            currentScreen = SCREEN_SETTINGS_WIFI;
            drawSettingsWifiScreen();
            break;
          case 4:
            resetConfirmIndex = 1;
            currentScreen     = SCREEN_SETTINGS_RESET_CONFIRM;
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
    
    case SCREEN_SETTINGS_LAMP: {
      // Menú de efectos de luz: RESPIRACION, COMETA, CUADRANTE

      if (stepDir != 0) {
        int delta = (stepDir > 0 ? 1 : -1);
        efectosMenuIndex += delta;
        if (efectosMenuIndex < 0) efectosMenuIndex = 2;
        if (efectosMenuIndex > 2) efectosMenuIndex = 0;
        drawSettingsLampScreen();
      }

      if (encButtonFalling) {
        if (efectosMenuIndex == 0) {
          // RESPIRACION
          respiracionStep = RESP_STEP_COLOR_INICIAL;

          respSliderPosInicial = sliderPosFromColor(respCfgColorIni565);
          respColorInicialSaved = respCfgColorIni565;

          respSliderPosFinal   = sliderPosFromColor(respCfgColorFin565);
          respColorFinalSaved  = respCfgColorFin565;

          respCicloIndex = respCfgCicloIndex;
          if (respCicloIndex >= RESP_CICLO_COUNT) respCicloIndex = 8;
          respCicloSegundos = RESP_CICLO_VALUES[respCicloIndex];

          currentScreen = SCREEN_SETTINGS_LAMP_CONFIG;
          drawRespiracionConfigScreen();
        }
        else if (efectosMenuIndex == 1) {
          // COMETA
          cometaStep = COMETA_STEP_COLOR_INICIAL;

          cometaSliderPosInicial = sliderPosFromColor(cometaCfgColorHead565);
          cometaColorHeadSaved   = cometaCfgColorHead565;

          cometaSliderPosFinal   = sliderPosFromColor(cometaCfgColorTail565);
          cometaColorTailSaved   = cometaCfgColorTail565;

          cometaCicloIndex    = cometaCfgCicloIndex;
          if (cometaCicloIndex >= COMETA_CICLO_COUNT) cometaCicloIndex = 4;
          cometaCicloSegundos = COMETA_CICLO_VALUES[cometaCicloIndex];

          currentScreen = SCREEN_SETTINGS_COMETA_CONFIG;
          drawCometaConfigScreen();
        }
        else if (efectosMenuIndex == 2) {
          // CUADRANTE
          cuadranteStep = CUAD_STEP_COLOR_INICIAL;

          cuadranteSliderPosInicial = sliderPosFromColor(cuadranteCfgColorIni565);
          cuadranteColorIniSaved    = cuadranteCfgColorIni565;

          cuadranteSliderPosFinal   = sliderPosFromColor(cuadranteCfgColorFin565);
          cuadranteColorFinSaved    = cuadranteCfgColorFin565;

          cuadranteCicloIndex    = cuadranteCfgCicloIndex;
          if (cuadranteCicloIndex >= CUADRANTE_CICLO_COUNT) cuadranteCicloIndex = 4;
          cuadranteCicloSegundos = CUADRANTE_CICLO_VALUES[cuadranteCicloIndex];

          currentScreen = SCREEN_SETTINGS_CUADRANTE_CONFIG;
          drawCuadranteConfigScreen();
        }
      }

      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_MAIN;
        drawSettingsMainScreen();
      }

      break;
    }

    case SCREEN_SETTINGS_LAMP_CONFIG: {
      // Pantalla de configuración del efecto RESPIRACION.

      // 1) Manejo del encoder
      if (stepDir != 0) {
        int delta = (stepDir > 0 ? 5 : -5);

        if (respiracionStep == RESP_STEP_COLOR_INICIAL) {
          // Mover knob izquierdo
          int v = (int)respSliderPosInicial + delta;
          if (v < 0)   v = 0;
          if (v > 255) v = 255;
          respSliderPosInicial = (uint8_t)v;

          drawRespiracionConfigScreen();
        }
        else if (respiracionStep == RESP_STEP_COLOR_FINAL) {
          // Mover knob derecho
          int v = (int)respSliderPosFinal + delta;
          if (v < 0)   v = 0;
          if (v > 255) v = 255;
          respSliderPosFinal = (uint8_t)v;

          drawRespiracionConfigScreen();
        }
        else if (respiracionStep == RESP_STEP_CICLO) {
          // Mover el índice del ciclo dentro de la tabla RESP_CICLO_VALUES
          int step = (stepDir > 0 ? 1 : -1);   // un click de encoder = un índice
          int idx  = respCicloIndex + step;

          if (idx < 0) idx = 0;
          if (idx >= RESP_CICLO_COUNT) idx = RESP_CICLO_COUNT - 1;

          if (idx != respCicloIndex) {
            respCicloIndex    = idx;
            respCicloSegundos = RESP_CICLO_VALUES[respCicloIndex];
            drawRespiracionConfigScreen();
          }
        }

        // En RESP_STEP_INICIAR aún no hacemos nada.
      }

      // 2) Pulsación del encoder
      if (encButtonFalling) {
        if (respiracionStep == RESP_STEP_COLOR_INICIAL) {
          // Guardar el color inicial en la cajita izquierda
          uint8_t rN, gN, bN;
          respColorInicialSaved = colorFromSlider(respSliderPosInicial, rN, gN, bN);

          // Pasar al siguiente paso: selección de color final
          respiracionStep = RESP_STEP_COLOR_FINAL;
          drawRespiracionConfigScreen();
        }
        else if (respiracionStep == RESP_STEP_COLOR_FINAL) {
          // Guardar el color final en la cajita derecha
          uint8_t rB, gB, bB;
          respColorFinalSaved = colorFromSlider(respSliderPosFinal, rB, gB, bB);

          // Pasar al siguiente paso: selección de ciclo
          respiracionStep = RESP_STEP_CICLO;
          drawRespiracionConfigScreen();
        }
        else if (respiracionStep == RESP_STEP_CICLO) {
          // Fijar el ciclo actual y pasar al paso de "Iniciar"
          respiracionStep = RESP_STEP_INICIAR;
          drawRespiracionConfigScreen();
        }
        else if (respiracionStep == RESP_STEP_INICIAR) {
          // 1) Volcar ajustes actuales a la config persistente
          respCfgColorIni565 = respColorInicialSaved;
          respCfgColorFin565 = respColorFinalSaved;
          respCfgCicloIndex  = (uint8_t)respCicloIndex;
          saveConfigBasic();   // guarda en NVS junto con el resto

          // 2) Iniciar efecto RESPIRACION
          startRespEffect();

          // 3) Salir a la pantalla principal de luz
          currentScreen = SCREEN_CLOCK;   // por ejemplo
          drawClockScreenFull();

        }
      }

      // 3) Botón 2 = salir SIN guardar cambios: vuelve al submenú Efectos
      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_LAMP;
        drawSettingsLampScreen();
      }

      break;
    }

    case SCREEN_SETTINGS_COMETA_CONFIG: {
      // Pantalla de configuración del efecto COMETA.

      // 1) Manejo del encoder
      if (stepDir != 0) {
        int delta = (stepDir > 0 ? 5 : -5);

        if (cometaStep == COMETA_STEP_COLOR_INICIAL) {
          int v = (int)cometaSliderPosInicial + delta;
          if (v < 0)   v = 0;
          if (v > 255) v = 255;
          cometaSliderPosInicial = (uint8_t)v;
          drawCometaConfigScreen();
        }
        else if (cometaStep == COMETA_STEP_COLOR_FINAL) {
          int v = (int)cometaSliderPosFinal + delta;
          if (v < 0)   v = 0;
          if (v > 255) v = 255;
          cometaSliderPosFinal = (uint8_t)v;
          drawCometaConfigScreen();
        }
        else if (cometaStep == COMETA_STEP_CICLO) {
          int step = (stepDir > 0 ? 1 : -1);
          int idx  = cometaCicloIndex + step;

          if (idx < 0) idx = 0;
          if (idx >= COMETA_CICLO_COUNT) idx = COMETA_CICLO_COUNT - 1;

          if (idx != cometaCicloIndex) {
            cometaCicloIndex    = idx;
            cometaCicloSegundos = COMETA_CICLO_VALUES[cometaCicloIndex];
            drawCometaConfigScreen();
          }
        }
      }

      // 2) Pulsación del encoder
      if (encButtonFalling) {
        if (cometaStep == COMETA_STEP_COLOR_INICIAL) {
          uint8_t rN, gN, bN;
          cometaColorHeadSaved = colorFromSlider(cometaSliderPosInicial, rN, gN, bN);
          cometaStep = COMETA_STEP_COLOR_FINAL;
          drawCometaConfigScreen();
        }
        else if (cometaStep == COMETA_STEP_COLOR_FINAL) {
          uint8_t rB, gB, bB;
          cometaColorTailSaved = colorFromSlider(cometaSliderPosFinal, rB, gB, bB);
          cometaStep = COMETA_STEP_CICLO;
          drawCometaConfigScreen();
        }
        else if (cometaStep == COMETA_STEP_CICLO) {
          cometaStep = COMETA_STEP_INICIAR;
          drawCometaConfigScreen();
        }
        else if (cometaStep == COMETA_STEP_INICIAR) {
          // Guardar config COMETA en NVS
          cometaCfgColorHead565 = cometaColorHeadSaved;
          cometaCfgColorTail565 = cometaColorTailSaved;
          cometaCfgCicloIndex   = (uint8_t)cometaCicloIndex;
          saveConfigBasic();

          // Iniciar efecto COMETA
          startCometaEffect();

          // Ir al reloj
          currentScreen = SCREEN_CLOCK;
          drawClockScreenFull();
        }
      }

      // 3) Botón 2 = salir SIN guardar cambios: vuelve al menú Efectos
      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_LAMP;
        drawSettingsLampScreen();
      }

      break;
    }

    case SCREEN_SETTINGS_CUADRANTE_CONFIG: {
      // Pantalla de configuración del efecto CUADRANTE.

      // 1) Manejo del encoder
      if (stepDir != 0) {
        int delta = (stepDir > 0 ? 5 : -5);

        if (cuadranteStep == CUAD_STEP_COLOR_INICIAL) {
          int v = (int)cuadranteSliderPosInicial + delta;
          if (v < 0)   v = 0;
          if (v > 255) v = 255;
          cuadranteSliderPosInicial = (uint8_t)v;
          drawCuadranteConfigScreen();
        }
        else if (cuadranteStep == CUAD_STEP_COLOR_FINAL) {
          int v = (int)cuadranteSliderPosFinal + delta;
          if (v < 0)   v = 0;
          if (v > 255) v = 255;
          cuadranteSliderPosFinal = (uint8_t)v;
          drawCuadranteConfigScreen();
        }
        else if (cuadranteStep == CUAD_STEP_CICLO) {
          int step = (stepDir > 0 ? 1 : -1);
          int idx  = cuadranteCicloIndex + step;

          if (idx < 0) idx = 0;
          if (idx >= CUADRANTE_CICLO_COUNT) idx = CUADRANTE_CICLO_COUNT - 1;

          if (idx != cuadranteCicloIndex) {
            cuadranteCicloIndex    = idx;
            cuadranteCicloSegundos = CUADRANTE_CICLO_VALUES[cuadranteCicloIndex];
            drawCuadranteConfigScreen();
          }
        }
      }

      // 2) Pulsación del encoder
      if (encButtonFalling) {
        if (cuadranteStep == CUAD_STEP_COLOR_INICIAL) {
          uint8_t rN, gN, bN;
          cuadranteColorIniSaved = colorFromSlider(cuadranteSliderPosInicial, rN, gN, bN);
          cuadranteStep = CUAD_STEP_COLOR_FINAL;
          drawCuadranteConfigScreen();
        }
        else if (cuadranteStep == CUAD_STEP_COLOR_FINAL) {
          uint8_t rB, gB, bB;
          cuadranteColorFinSaved = colorFromSlider(cuadranteSliderPosFinal, rB, gB, bB);
          cuadranteStep = CUAD_STEP_CICLO;
          drawCuadranteConfigScreen();
        }
        else if (cuadranteStep == CUAD_STEP_CICLO) {
          cuadranteStep = CUAD_STEP_INICIAR;
          drawCuadranteConfigScreen();
        }
        else if (cuadranteStep == CUAD_STEP_INICIAR) {
          // Guardar config CUADRANTE en NVS
          cuadranteCfgColorIni565 = cuadranteColorIniSaved;
          cuadranteCfgColorFin565 = cuadranteColorFinSaved;
          cuadranteCfgCicloIndex  = (uint8_t)cuadranteCicloIndex;
          saveConfigBasic();

          // Iniciar efecto CUADRANTE (lo implementaremos en el siguiente paso)
          startCuadranteEffect();

          // Ir al reloj
          currentScreen = SCREEN_CLOCK;
          drawClockScreenFull();
        }
      }

      // 3) Botón 2 = salir SIN guardar cambios: volver al menú Efectos
      if (btn2Falling) {
        currentScreen = SCREEN_SETTINGS_LAMP;
        drawSettingsLampScreen();
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


  // --- Actualizar efecto RESPIRACION si está activo ---

  if (respEffectActive) {
    unsigned long now = millis();

    // Duración total de un ciclo subida+bajada en ms
    float cicloSeg = RESP_CICLO_VALUES[respCfgCicloIndex];
    if (cicloSeg < 0.1f) cicloSeg = 0.1f;
    float halfCycleMs = (cicloSeg * 1000.0f) / 2.0f;

    if (now != respLastUpdate) {
      float dt = (float)(now - respLastUpdate);
      respLastUpdate = now;

      float deltaPhase = (255.0f * dt) / halfCycleMs;
      int phase = respPhase;

      if (respEffectForward) {
        phase += (int)deltaPhase;
        if (phase >= 255) {
          phase = 255;
          respEffectForward = false;
        }
      } else {
        phase -= (int)deltaPhase;
        if (phase <= 0) {
          phase = 0;
          respEffectForward = true;
        }
      }

      if (phase < 0) phase = 0;
      if (phase > 255) phase = 255;

      respPhase = (uint8_t)phase;

      // Calcular color actual interpolado
      uint16_t cNow = lerpColor565(respCfgColorIni565, respCfgColorFin565, respPhase);

      uint8_t r,g,b;
      color565ToRGB(cNow, r, g, b);
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(r, g, b);
      }
      FastLED.setBrightness(brightness);
      FastLED.show();
    }
  }

  lastSw = sw;
  lastBtn2 = btn2;

  // --- Actualizar efecto arco iris si está activo ---
  if (lampOn && rainbowMode) {
    rainbowHue++;
    updateLeds();
  }

  // --- Actualizar efecto RESPIRACION si está activo ---
  if (lampOn && respEffectActive) {
    unsigned long now = millis();

    // Duración total de un ciclo subida+bajada en ms
    float cicloSeg = RESP_CICLO_VALUES[respCfgCicloIndex];
    if (cicloSeg < 0.1f) cicloSeg = 0.1f;
    float halfCycleMs = (cicloSeg * 1000.0f) / 2.0f;

    if (now != respLastUpdate) {
      float dt = (float)(now - respLastUpdate);
      respLastUpdate = now;

      float deltaPhase = (255.0f * dt) / halfCycleMs;
      int phase = respPhase;

      if (respEffectForward) {
        phase += (int)deltaPhase;
        if (phase >= 255) {
          phase = 255;
          respEffectForward = false;
        }
      } else {
        phase -= (int)deltaPhase;
        if (phase <= 0) {
          phase = 0;
          respEffectForward = true;
        }
      }

      if (phase < 0) phase = 0;
      if (phase > 255) phase = 255;

      respPhase = (uint8_t)phase;

      // Calcular color actual interpolado
      uint16_t cNow = lerpColor565(respCfgColorIni565, respCfgColorFin565, respPhase);

      uint8_t r,g,b;
      color565ToRGB(cNow, r, g, b);
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(r, g, b);
      }
      FastLED.setBrightness(brightness);
      FastLED.show();
    }
  }

  // --- Actualizar efecto COMETA si está activo ---
  if (lampOn && cometaEffectActive) {
    unsigned long now = millis();
    if (now != cometaLastUpdate) {
      float dt = (float)(now - cometaLastUpdate);
      cometaLastUpdate = now;

      // Duración de una vuelta completa en ms
      float cicloMs = cometaCicloSegundos * 1000.0f;
      if (cicloMs < 10.0f) cicloMs = 10.0f;

      // Avance de la cabeza en fracción de vuelta global
      float deltaHead = dt / cicloMs;
      cometaHeadPos += deltaHead;
      cometaHeadPos -= floorf(cometaHeadPos);
      if (cometaHeadPos < 0.0f) cometaHeadPos += 1.0f;
      if (cometaHeadPos > 1.0f) cometaHeadPos -= 1.0f;

      // Limpiar todos los LEDs (fondo negro para COMETA)
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
      }

      // Fases de 3 aros:
      // F0: 1-2-3
      // F1: 2-3-4
      // F2: 3-4-5
      // F3: 4-5-6
      // F4: 5-6-7
      // F5: 6-7-8
      // F6: 7-8-9
      const int GROUP_COUNT = 7;
      const int groupRingA[GROUP_COUNT] = { 0, 1, 2, 3, 4, 5, 6 }; // aro "bajo"
      const int groupRingB[GROUP_COUNT] = { 1, 2, 3, 4, 5, 6, 7 }; // aro central
      const int groupRingC[GROUP_COUNT] = { 2, 3, 4, 5, 6, 7, 8 }; // aro "alto"

      // Posición de la cabeza en el ciclo de 0..1, mapeada a 7 fases
      float phase = cometaHeadPos * (float)GROUP_COUNT;
      int   phaseIdx = (int)floorf(phase);      // 0..6
      float phaseFrac = phase - (float)phaseIdx;// 0..1 dentro de la fase

      if (phaseIdx < 0) phaseIdx = 0;
      if (phaseIdx >= GROUP_COUNT) phaseIdx = GROUP_COUNT - 1;

      int ringA = groupRingA[phaseIdx];
      int ringB = groupRingB[phaseIdx]; // centro (punta)
      int ringC = groupRingC[phaseIdx];

      // Procesamos solo los 3 aros de esta fase: A, B (centro), C
      int rings[3] = { ringA, ringB, ringC };

      for (int ri = 0; ri < 3; ri++) {
        int r = rings[ri];
        int start = ringStart[r];
        int len   = ringLen[r];
        if (len <= 0) continue;

        // ¿Es el aro central del grupo?
        bool isCenterRing = (r == ringB);

        for (int k = 0; k < len; k++) {
          int idxLed = start + k;

          // u = ángulo local 0..1 a lo largo del aro, sentido horario
          float u = (float)k / (float)len;

          // Distancia "hacia atrás" desde la cabeza dentro de esta fase
          float dist = phaseFrac - u;
          if (dist < 0.0f) dist += 1.0f;

          if (dist > cometaLength) {
            // Fuera de la cola: se queda apagado
            continue;
          }

          // Dentro de la cola: t=0 en la cabeza, t=1 en el final de cola
          float t = (cometaLength > 0.0f) ? (dist / cometaLength) : 1.0f;
          if (t < 0.0f) t = 0.0f;
          if (t > 1.0f) t = 1.0f;

          // Hacer la cabeza "en punta": solo el aro central puede tener t=0.
          // En los aros A y C, forzamos t>0 para que nunca haya cabeza pura.
          if (!isCenterRing && t < 0.08f) {
            t = 0.08f; // mínimo para que sea solo cola suave
          }

          // Interpolación de color entre cabeza y cola
          uint8_t tt = (uint8_t)roundf(t * 255.0f);
          uint16_t cNow = lerpColor565(cometaCfgColorHead565, cometaCfgColorTail565, tt);

          // Curva de brillo: cabeza más brillante, cola se va apagando
          float bFactor = 1.0f - (t * t);
          if (bFactor < 0.0f) bFactor = 0.0f;
          if (bFactor > 1.0f) bFactor = 1.0f;

          uint8_t rCol, gCol, bCol;
          color565ToRGB(cNow, rCol, gCol, bCol);
          rCol = (uint8_t)(rCol * bFactor);
          gCol = (uint8_t)(gCol * bFactor);
          bCol = (uint8_t)(bCol * bFactor);

          leds[idxLed] = CRGB(rCol, gCol, bCol);
        }
      }

      FastLED.setBrightness(brightness);
      FastLED.show();
    }
  }

  // --- Actualizar efecto CUADRANTE si está activo ---
  if (lampOn && cuadranteEffectActive) {
    unsigned long now = millis();
    if (now != cuadranteLastUpdate) {
      float dt = (float)(now - cuadranteLastUpdate);
      cuadranteLastUpdate = now;

      // Tiempo total para recorrer los 4 cuadrantes = cuadranteCicloSegundos
      float totalMs = cuadranteCicloSegundos * 1000.0f;
      if (totalMs < 40.0f) totalMs = 40.0f;
      float cuadranteMs = totalMs / 4.0f;
      if (cuadranteMs < 10.0f) cuadranteMs = 10.0f;

      // Progreso 0..1 dentro del cuadrante actual
      float p = cuadranteProgress + dt / cuadranteMs;
      if (p >= 1.0f) {
        p -= 1.0f;
        cuadranteIndex = (cuadranteIndex + 1) & 0x03; // 0..3
      }
      cuadranteProgress = p;

      // Limpiar todos los LEDs
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
      }

      // Definición de cuadrantes en grados (después de ajustar a 12 arriba)
      float qStart = 0.0f;
      float qEnd   = 90.0f;

      switch (cuadranteIndex) {
        case 0: // 12 -> 3
          qStart =   0.0f; qEnd =  90.0f;
          break;
        case 1: // 3 -> 6
          qStart =  90.0f; qEnd = 180.0f;
          break;
        case 2: // 6 -> 9
          qStart = 180.0f; qEnd = 270.0f;
          break;
        default: // 9 -> 12
          qStart = 270.0f; qEnd = 360.0f;
          break;
      }

      // Progreso de llenado/vaciado:
      // - pNorm in [0,1] recorre todo el cuadrante.
      // - primera mitad (pNorm<0.5): llenado desde centro a exterior
      // - segunda mitad: vaciado desde centro a exterior
      float pNorm = p; // ya está 0..1
      float filledMax;  // radio máximo iluminado (0 centro, 1 exterior)
      float filledMin;  // radio mínimo iluminado (para vaciado)
      if (pNorm < 0.5f) {
        // Llenado: 0->0.5 => filledMax 0->1, filledMin=0
        float tFill = pNorm / 0.5f;       // 0..1
        filledMax = tFill;
        if (filledMax > 1.0f) filledMax = 1.0f;
        filledMin = 0.0f;
      } else {
        // Vaciado: 0.5->1 => filledMin 0->1, filledMax=1
        float tEmpty = (pNorm - 0.5f) / 0.5f; // 0..1
        if (tEmpty > 1.0f) tEmpty = 1.0f;
        filledMax = 1.0f;
        filledMin = tEmpty;
      }

      // Recorremos todos los anillos (0=aro1 exterior .. 8=aro9 centro)
      for (int r = 0; r < RING_COUNT; r++) {
        int start = ringStart[r];
        int len   = ringLen[r];
        if (len <= 0) continue;

        int ringIndexFromOutside = r;                 // 0..8
        int ringIndexFromCenter  = (RING_COUNT - 1) - ringIndexFromOutside; // 8..0
        float ringRadiusNorm = (float)ringIndexFromCenter / (float)(RING_COUNT - 1);
        // ringRadiusNorm = 0 en centro (aro9), 1 en exterior (aro1)

        // ¿Este aro está dentro de la banda iluminada actual?
        if (ringRadiusNorm < filledMin || ringRadiusNorm > filledMax) {
          continue;
        }

        // tRadial = 0 en borde exterior (color inicial), 1 en centro (color final)
        float tRadial = ringRadiusNorm; // ya va 0 centro ->1 exterior, invertimos:
        tRadial = 1.0f - tRadial;       // 0 en centro, 1 en exterior
        // Para que cumpla: centro = color final, exterior = color inicial
        // así que usamos 1-tRadial como interpolante hacia color final
        if (tRadial < 0.0f) tRadial = 0.0f;
        if (tRadial > 1.0f) tRadial = 1.0f;

        for (int k = 0; k < len; k++) {
          int idxLed = start + k;

          // Ángulo local de este LED
          float angle = (2.0f * 3.1415926f * (float)k) / (float)len;
          float adj   = angle - 3.1415926f / 2.0f; // 0 rad en 12

          // Pasar a grados 0..360
          float deg = adj * (180.0f / 3.1415926f);
          while (deg < 0.0f)    deg += 360.0f;
          while (deg >= 360.0f) deg -= 360.0f;

          // ¿Está este LED dentro del cuadrante actual?
          bool inQuad = false;
          if (qStart <= qEnd) {
            inQuad = (deg >= qStart && deg < qEnd);
          } else {
            inQuad = (deg >= qStart || deg < qEnd);
          }
          if (!inQuad) continue;

          // Interpolación de color radial: 0=cabeza (exterior, color inicial), 1=cola (centro, color final)
          uint8_t tt = (uint8_t)roundf(tRadial * 255.0f);
          uint16_t cNow = lerpColor565(cuadranteCfgColorIni565, cuadranteCfgColorFin565, tt);

          // Brillo uniforme dentro de la banda; si quisieras, aquí podríamos
          // añadir una pequeña curva de borde, pero de momento lo dejamos plano.
          float bFactor = 1.0f;
          uint8_t rCol, gCol, bCol;
          color565ToRGB(cNow, rCol, gCol, bCol);
          rCol = (uint8_t)(rCol * bFactor);
          gCol = (uint8_t)(gCol * bFactor);
          bCol = (uint8_t)(bCol * bFactor);

          leds[idxLed] = CRGB(rCol, gCol, bCol);
        }
      }

      FastLED.setBrightness(brightness);
      FastLED.show();
    }
  }

}

// ============================================================
// ESP32 + TMC2208 + NEMA 17: einfacher Test
// Eine Umdrehung vorwaerts, Pause, eine Umdrehung rueckwaerts, Pause, ...
//  - TMCStepper:       stellt den Treiber per UART ein
//  - FastAccelStepper: erzeugt die STEP-Pulse mit Beschleunigungsrampe
// ============================================================

#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

// ---------- Pinbelegung ----------
constexpr uint8_t STEP_PIN = 4; // D4  -> STEP
constexpr uint8_t DIR_PIN = 22; // D22 -> DIR
constexpr uint8_t EN_PIN = 23;  // D23 -> EN (LOW = Treiber aktiv)
constexpr uint8_t UART_RX = 16; // GPIO16 (RX2) <- PDN_UART (direkt)
constexpr uint8_t UART_TX = 17; // GPIO17 (TX2) -> 1 kOhm -> PDN_UART

// ---------- Treiber-Einstellungen ----------
constexpr float R_SENSE = 0.11f;          // Sense-Widerstand deines Moduls in Ohm (meist 0,11)
constexpr uint16_t RMS_CURRENT_MA = 1500; // Motorstrom, NIE ueber dem Nennstrom des Motors
constexpr uint16_t MICROSTEPS = 8;        // 1 = Vollschritt, 8 = 1/8, 16 = 1/16, ...
constexpr bool USE_SPREADCYCLE = true;    // true = SpreadCycle, false = StealthChop (leise)
constexpr bool USE_INTERPOLATION = true;  // true = Glättung auf 1/256

// ---------- Bewegung ----------
constexpr uint16_t MOTOR_FULL_STEPS = 200; // 1,8 Grad Motor = 200 Vollschritte
constexpr uint32_t STEPS_PER_REV = (uint32_t)MOTOR_FULL_STEPS * MICROSTEPS;
constexpr float SPEED_RPS = 12.0f;    // Drehzahl in Umdrehungen pro Sekunde
constexpr float ACCEL_REV_S2 = 40.0f; // Beschleunigung in Umdrehungen pro Sekunde^2
constexpr uint32_t PAUSE_MS = 1000;   // Pause zwischen den Bewegungen

// ---------- Objekte ----------
TMC2208Stepper driver(&Serial2, R_SENSE);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

// Programm anhalten und Meldung ausgeben (bei Fehlern)
void halt(const char *text)
{
  Serial.println(text);
  while (true)
  {
    delay(1000);
  }
}

// Wartet, bis der Motor seine Bewegung beendet hat
void waitUntilStopped()
{
  while (stepper->isRunning())
  {
    delay(10);
  }
}

void setup()
{
  // EN zuerst auf HIGH: Endstufe bleibt aus, bis alles eingerichtet ist
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  Serial.begin(115200);
  delay(500);
  Serial.println("Motortest gestartet");

  // UART zum Treiber
  Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  driver.begin();

  if (driver.test_connection() != 0)
  {
    halt("UART-Verbindung FEHLER: 12 V vor dem Start an? PDN_UART und 1-kOhm-Widerstand pruefen, dann EN/RST druecken.");
  }
  Serial.println("UART-Verbindung OK");

  // Treiber einrichten
  driver.pdn_disable(true);      // PDN_UART-Pin dient als UART
  driver.mstep_reg_select(true); // Mikroschritte aus dem Register, nicht von MS1/MS2
  driver.I_scale_analog(false);  // Strom aus dem Register, nicht vom Vref-Poti
  driver.toff(5);                // Endstufe freigeben
  driver.blank_time(24);
  driver.rms_current(RMS_CURRENT_MA); // Motorstrom in mA
  driver.pwm_autoscale(true);
  driver.intpol(USE_INTERPOLATION);
  driver.microsteps(MICROSTEPS);
  driver.en_spreadCycle(USE_SPREADCYCLE);

  // FastAccelStepper einrichten
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper == nullptr)
  {
    halt("FEHLER: STEP-Pin konnte nicht verbunden werden.");
  }
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(EN_PIN); // LOW = aktiv
  stepper->setAutoEnable(false); // Motor bleibt bestromt (Haltemoment)
  stepper->enableOutputs();
  stepper->setSpeedInHz((uint32_t)(SPEED_RPS * STEPS_PER_REV));
  stepper->setAcceleration((int32_t)(ACCEL_REV_S2 * STEPS_PER_REV));

  delay(1000);
}

void loop()
{
  Serial.println("Vorwaerts (1 Umdrehung)");
  stepper->move((int32_t)STEPS_PER_REV * 7); // relative Bewegung: positive Schritte = eine Richtung
  waitUntilStopped();
  // delay(PAUSE_MS);

  Serial.println("Rueckwaerts (1 Umdrehung)");
  stepper->move(-(int32_t)STEPS_PER_REV * 7); // negative Schritte = Gegenrichtung
  waitUntilStopped();
  // delay(PAUSE_MS);
}
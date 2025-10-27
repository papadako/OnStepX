// Mappings for the Mesu-200 on MaxPCB4 (Teensy 4.1)
// George Korres & Panagiotis Papadakos modifications
// Notes:
// - PH1 = direction (logic), PH2 = PWM (duty) for the PE driver
// - Axis1 STEP/DIR pins (38/39) are used here as the *encoder* inputs
// - Teensy pin numbers are shown in comments for clarity


#if defined(ARDUINO_TEENSY41)

// ---- Identify this pinmap as Mesu on MaxPCB4 ----
#define MESU_MAXPCB4 1  // I use this in PE.cpp  so that we do not mess with pin38 PWM emulation if not needed.

// ---------- Axis 1 (RA/Azm) ----------
#define AXIS1_ENCODER_A_PIN   AXIS1_DIR_PIN   // Teensy 39
#define AXIS1_ENCODER_B_PIN   AXIS1_STEP_PIN  // Teensy 38

// PE driver signals
#define AXIS1_SERVO_PH1_PIN   AXIS1_M0_PIN    // Teensy 37  (direction line)
#define AXIS1_SERVO_PH2_PIN   AXIS1_M1_PIN    // Teensy 36  (PWM output)

// FYI (unchanged MaxPCB4 pins):
// #define AXIS1_ENABLE_PIN    33
// #define AXIS1_M2_PIN        35
// #define AXIS1_M3_PIN        34

// ---------- Axis 2 (Dec/Alt) ----------
#define AXIS2_ENCODER_A_PIN   AXIS2_DIR_PIN   // Teensy 16
#define AXIS2_ENCODER_B_PIN   AXIS2_STEP_PIN  // Teensy 15

// PE driver signals
#define AXIS2_SERVO_PH1_PIN   AXIS2_M2_PIN    // Teensy 14  (direction line)
#define AXIS2_SERVO_PH2_PIN   AXIS2_M1_PIN    // Teensy 13  (PWM output)

// FYI (unchanged MaxPCB4 pins):
// #define AXIS2_ENABLE_PIN    40
// #define AXIS2_M0_PIN        41
// #define AXIS2_M3_PIN        34

// ---------- Original values (from MaxPCB4 pinmap) ----------
// AXIS1_STEP_PIN = 38, AXIS1_DIR_PIN = 39
// AXIS1_M0_PIN   = 37, AXIS1_M1_PIN  = 36, AXIS1_M2_PIN = 35, AXIS1_M3_PIN = 34
// AXIS2_STEP_PIN = 15, AXIS2_DIR_PIN = 16
// AXIS2_M0_PIN   = 41, AXIS2_M1_PIN  = 13, AXIS2_M2_PIN = 14, AXIS2_M3_PIN = 34

#endif // ARDUINO_TEENSY41

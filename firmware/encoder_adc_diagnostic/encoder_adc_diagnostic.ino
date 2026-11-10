// Two-channel AS5600 encoder acquisition (hardware test sketch)
// Reads the AS5600 analogue outputs on two ADC channels at 12-bit
// resolution and prints the raw codes every 100 ms at 115200 baud.
// Built with the Arduino core for STM32 (STM32duino).

#if defined(ARDUINO_ARCH_STM32)
  // STM32 NUCLEO board
  #define M1_ADC PA3
  #define M2_ADC PC0
#else
  // Arduino Uno / Nano / Uno R4
  #define M1_ADC A0
  #define M2_ADC A1
#endif

uint16_t motor1_ADC = 0;
uint16_t motor2_ADC = 0;

void setup() {
  Serial.begin(115200);
  // 12-bit ADC for STM32 and Arduino Uno R4
#if defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_RENESAS)
  analogReadResolution(12);
#endif
  pinMode(M1_ADC, INPUT);
  pinMode(M2_ADC, INPUT);
  delay(1000);
  Serial.println("ADC Feedback Started");
}

void loop() {
  motor1_ADC = analogRead(M1_ADC);      // read encoder 1
  motor2_ADC = analogRead(M2_ADC);      // read encoder 2
  Serial.print("M1 ADC = ");
  Serial.print(motor1_ADC);
  Serial.print(" | M2 ADC = ");
  Serial.println(motor2_ADC);
  delay(100);                           // 100 ms between samples
}

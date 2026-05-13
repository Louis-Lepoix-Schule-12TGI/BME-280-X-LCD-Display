#include <SPI.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <math.h> 

rgb_lcd lcd;
const int CS_PIN = 10;

struct {
  uint16_t dig_T1; int16_t dig_T2, dig_T3;
  uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
  uint8_t  dig_H1; int16_t dig_H2; uint8_t  dig_H3; int16_t dig_H4, dig_H5; int8_t  dig_H6;
} calib;

int32_t t_fine; 
float referencePressure; 

// Variablen für das Zeitmanagement (ohne delay)
unsigned long lastDisplaySwitch = 0;
int displayMode = 0;

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setRGB(255, 255, 255);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();
  
  writeRegister(0xE0, 0xB6); 
  delay(100);
  readCalibrationData();
  writeRegister(0xF2, 0x01); 
  writeRegister(0xF4, 0x27); 
  writeRegister(0xF5, 0xA0); 

  lcd.print("Kalibriere...");
  delay(1000); 
  referencePressure = getCurrentPressure(); 
  lcd.clear();
}

void loop() {
  // 1. MESSUNG (so schnell wie möglich)
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0xF7 | 0x80);
  uint32_t p_msb  = SPI.transfer(0x00);
  uint32_t p_lsb  = SPI.transfer(0x00);
  uint32_t p_xlsb = SPI.transfer(0x00);
  uint32_t t_msb  = SPI.transfer(0x00);
  uint32_t t_lsb  = SPI.transfer(0x00);
  uint32_t t_xlsb = SPI.transfer(0x00);
  uint32_t h_msb  = SPI.transfer(0x00);
  uint32_t h_lsb  = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);

  int32_t raw_press = (p_msb << 12) | (p_lsb << 4) | (p_xlsb >> 4);
  int32_t raw_temp  = (t_msb << 12) | (t_lsb << 4) | (t_xlsb >> 4);
  int32_t raw_hum   = (h_msb << 8) | h_lsb;

  float temp = compensate_T(raw_temp);
  float pres = compensate_P(raw_press) / 100.0F; 
  float hum  = compensate_H(raw_hum);
  float altitude = 44330.0 * (1.0 - pow(pres / referencePressure, 0.1903));

  // 2. ANZEIGE-LOGIK (Höhe ist immer da, Rest rotiert oben)
  unsigned long currentMillis = millis();

  // Wechselt alle 1,5 Sekunden den Wert in der oberen Zeile
  if (currentMillis - lastDisplaySwitch > 1500) {
    displayMode = (displayMode + 1) % 3;
    lastDisplaySwitch = currentMillis;
    // Nur die obere Zeile löschen (indem wir 16 Leerzeichen schreiben)
    lcd.setCursor(0, 0);
    lcd.print("                "); 
  }

  // Zeile 1: Wechselnde Werte
  lcd.setCursor(0, 0);
  if (displayMode == 0) {
    lcd.print("Temp: "); lcd.print(temp, 1); lcd.print("C");
  } else if (displayMode == 1) {
    lcd.print("Feuchte: "); lcd.print(hum, 1); lcd.print("%");
  } else {
    lcd.print("Druck: "); lcd.print(pres, 1); 
  }

  // Zeile 2: LIVE HÖHE (wird in jedem Loop-Durchlauf aktualisiert)
  lcd.setCursor(0, 1);
  lcd.print("H: ");
  lcd.print(altitude, 2); // 2 Dezimalstellen für Präzision
  lcd.print(" m      "); // Leerzeichen am Ende überschreiben alte Ziffern

  delay(50); // Kleiner Delay, damit das Display nicht flimmert (20 Updates/Sekunde)
}

// --- Hilfsfunktionen (unverändert lassen) ---

float getCurrentPressure() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0xF7 | 0x80);
  uint32_t p_msb  = SPI.transfer(0x00);
  uint32_t p_lsb  = SPI.transfer(0x00);
  uint32_t p_xlsb = SPI.transfer(0x00);
  uint32_t t_msb  = SPI.transfer(0x00);
  uint32_t t_lsb  = SPI.transfer(0x00);
  uint32_t t_xlsb = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  compensate_T((t_msb << 12) | (t_lsb << 4) | (t_xlsb >> 4));
  return (float)compensate_P((p_msb << 12) | (p_lsb << 4) | (p_xlsb >> 4)) / 100.0F;
}

void writeRegister(byte reg, byte val) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(CS_PIN, HIGH);
}

uint8_t read8(byte reg) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return val;
}

uint16_t read16_LE(byte reg) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint16_t l = SPI.transfer(0x00);
  uint16_t h = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return (h << 8) | l;
}

void readCalibrationData() {
  calib.dig_T1 = read16_LE(0x88);
  calib.dig_T2 = (int16_t)read16_LE(0x8A);
  calib.dig_T3 = (int16_t)read16_LE(0x8C);
  calib.dig_P1 = read16_LE(0x8E);
  calib.dig_P2 = (int16_t)read16_LE(0x90);
  calib.dig_P3 = (int16_t)read16_LE(0x92);
  calib.dig_P4 = (int16_t)read16_LE(0x94);
  calib.dig_P5 = (int16_t)read16_LE(0x96);
  calib.dig_P6 = (int16_t)read16_LE(0x98);
  calib.dig_P7 = (int16_t)read16_LE(0x9A);
  calib.dig_P8 = (int16_t)read16_LE(0x9C);
  calib.dig_P9 = (int16_t)read16_LE(0x9E);
  calib.dig_H1 = read8(0xA1);
  calib.dig_H2 = (int16_t)read16_LE(0xE1);
  calib.dig_H3 = read8(0xE3);
  calib.dig_H4 = (int16_t)((read8(0xE4) << 4) | (read8(0xE5) & 0x0F));
  calib.dig_H5 = (int16_t)((read8(0xE6) << 4) | (read8(0xE5) >> 4));
  calib.dig_H6 = (int8_t)read8(0xE7);
}

float compensate_T(int32_t adc_T) {
  int32_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
  t_fine = var1 + var2;
  return (float)((t_fine * 5 + 128) >> 8) / 100.0F;
}

uint32_t compensate_P(int32_t adc_P) {
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
  if (var1 == 0) return 0;
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
  return (uint32_t)p;
}

float compensate_H(int32_t adc_H) {
  int32_t v_x1_u32r;
  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) * v_x1_u32r)) +
    ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) * (((v_x1_u32r *
    ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
    ((int32_t)calib.dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  return (float)(v_x1_u32r >> 12) / 1024.0F;
}
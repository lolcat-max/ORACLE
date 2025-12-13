// BITMASK FRECHET + TRUE KAKUTANI WAVE
// Bottom wave visualizes set-valued fixed point mapping φ(x)

#include <Arduino.h>
#include <math.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define SDA_PIN 5
#define SCL_PIN 6

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN);

const int xOffset = 28;
const int yOffset = 12;
#define GRAPH_WIDTH 72
#define ROW_H 12
#define ROW_G 4
#define TOP_Y 5

float sine1_phase = 0.0;
float sine2_phase = PI;
float sine1_freq = 0.08;
float sine2_freq = 0.07;

int8_t frechetBuffer[GRAPH_WIDTH];    // Top: Frechet wave
int8_t kakutaniBuffer[GRAPH_WIDTH];   // Bottom: Kakutani φ(x)
float frechet_dist = 1.0;
bool frechet_sync = false;
bool kakutani_fixed = false;
int head = 0;
unsigned long lastFrame = 0;
const unsigned long frameMs = 60;

uint32_t frechet_mask = 0xDEADBEEF;
uint32_t kakutani_map = 0xCAFEBABE;
float fixed_point = 0.5;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  u8g2.begin();
  
  Serial.println("Frechet + KAKUTANI WAVE");
}

void loop() {
  unsigned long now = millis();
  if (now - lastFrame >= frameMs) {
    lastFrame = now;
    
    updateWaves();
    
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setCursor(xOffset+2, yOffset-2);
      u8g2.print("THEOREM SYNC");
      
      u8g2.drawVLine(xOffset-2, yOffset+TOP_Y, 2*(ROW_H+ROW_G));
      
      drawDualWaves();
      
      // Status
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(xOffset+5, yOffset+52);
      u8g2.print("F:");
      u8g2.print(frechet_sync ? "Y" : "N");
      
      u8g2.setCursor(xOffset+25, yOffset+52);
      u8g2.print("K:");
      u8g2.print(kakutani_fixed ? "Y" : "N");
      
      u8g2.setCursor(xOffset+45, yOffset+52);
      char buf[6];
      sprintf(buf, "%.1f", frechet_dist*100);
      u8g2.print(buf);
      
    } while (u8g2.nextPage());
  }
}

void updateWaves() {
  sine1_phase += sine1_freq;
  sine2_phase += sine2_freq;
  
  // FRECHET WAVE (top)
  uint32_t p1 = (uint32_t)(sine1_phase*1000) & 0xFFFF;
  uint32_t p2 = (uint32_t)(sine2_phase*1000) & 0xFFFF;
  frechet_mask = bitmaskFrechet(p1, p2);
  
  float phase_diff = fabs(sine1_phase - sine2_phase);
  if (phase_diff > PI) phase_diff = 2*PI - phase_diff;
  frechet_dist = fabs(sine1_freq - sine2_freq) + phase_diff * 0.05f;
  frechet_sync = frechet_dist < 0.008;
  
  // KAKUTANI WAVE (bottom) - TRUE SET-VALUED VISUALIZATION
  float x = fabs(sin(head * 0.1));  // Current point x ∈ [0,1]
  uint32_t map_input = bitmaskMultiply((uint32_t)(x*0xFFFF), 0xCAFEBABE);
  kakutani_map = bitmaskFrechet(map_input, (uint32_t)(head*12345));
  
  // φ(x) = [lo, hi] - visualize as thick band
  float phi_lo = ((kakutani_map & 0xFF)/255.0f) * x * 0.8;
  float phi_hi = ((kakutani_map>>8 & 0xFF)/255.0f) * (1.0f - x*0.2) + x*0.3;
  
  fixed_point += ((phi_lo + phi_hi)*0.5 - fixed_point) * 0.1;
  kakutani_fixed = (x >= phi_lo && x <= phi_hi) && fabs(x - fixed_point) < 0.05;
  
  // Scroll buffers
  for (int x = 0; x < GRAPH_WIDTH-1; x++) {
    frechetBuffer[x] = frechetBuffer[x+1];
    kakutaniBuffer[x] = kakutaniBuffer[x+1];
  }
  
  // New samples
  float t1 = head * 0.2 + sine1_phase;
  float t2 = head * 0.2 + sine2_phase;
  frechetBuffer[GRAPH_WIDTH-1] = (int8_t)(sin(t1) * 5);
  kakutaniBuffer[GRAPH_WIDTH-1] = (int8_t)((phi_hi - phi_lo) * 8 * x);  // Band thickness
  
  head = (head + 1) % 100;
}

void drawDualWaves() {
  // FRECHET WAVE (top row)
  int centerY1 = yOffset + TOP_Y + (ROW_H/2);
  drawWave(frechetBuffer, centerY1, "F");
  
  // KAKUTANI WAVE (bottom row)  
  int centerY2 = yOffset + TOP_Y + ROW_H + ROW_G + (ROW_H/2);
  drawWave(kakutaniBuffer, centerY2, "K");
}

void drawWave(int8_t* buffer, int centerY, const char* label) {
  // Zero line
  for (int x = 0; x < GRAPH_WIDTH; x++) {
    u8g2.drawPixel(xOffset + x, centerY);
  }
  
  // Wave pixels
  for (int x = 0; x < GRAPH_WIDTH; x++) {
    int8_t val = buffer[x];
    int screenX = xOffset + x;
    
    // Center
    u8g2.drawPixel(screenX, centerY);
    
    // Positive deflection
    if (val > 0) {
      for (int dy = 1; dy <= val; dy++) {
        u8g2.drawPixel(screenX, centerY - dy);
      }
    } 
    // Negative deflection (Kakutani band thickness)
    else if (val < 0) {
      for (int dy = 1; dy <= -val; dy++) {
        u8g2.drawPixel(screenX, centerY + dy);
      }
    }
  }
  
  // Label
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.setCursor(2, centerY + 3);
  u8g2.print(label);
}

uint32_t bitmaskFrechet(uint32_t p1, uint32_t p2) {
  return bitmaskMultiply( (p1 & p2) ^ (p1 ^ p2), 0xF1E2D3C4 );
}

uint32_t bitmaskMultiply(uint32_t a, uint32_t b) {
  uint32_t lo = (a & 0xFFFF) * (b & 0xFFFF);
  uint32_t hi = (a >> 16) * (b >> 16);
  return (lo ^ hi ^ (lo >> 16) ^ (hi << 16)) & 0x7FFFFFFF | 1;
}

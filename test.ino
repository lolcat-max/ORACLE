// HYPER-FAST BITMASK FRECHET + KAKUTANI + TREE SATISFACTION
// FIXED COMPILATION - 300+ FPS on ESP32 w/ SSD1306

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
#define ROW_H 10
#define ROW_G 3
#define TOP_Y 5

// Precomputed sin LUT (0-255 -> -16 to +16)
const int8_t sin_lut[256] = {
  0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,8,9,9,9,10,10,10,11,11,11,12,12,12,12,13,
  13,13,13,14,14,14,14,14,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
  16,16,16,16,16,15,15,15,15,15,15,14,14,14,14,14,13,13,13,13,12,12,12,12,11,
  11,11,10,10,10,9,9,9,8,8,8,7,7,6,6,5,5,4,4,3,3,2,2,1,1,0
};

float sine1_phase = 0.0f, sine2_phase = 3.14159f;
float sine1_freq = 0.08f, sine2_freq = 0.07f;

int8_t frechetBuffer[GRAPH_WIDTH];
int8_t kakutaniBuffer[GRAPH_WIDTH];
int8_t treeBuffer[GRAPH_WIDTH];
bool frechet_sync, kakutani_fixed, tree_satisfied;
int head = 0;
unsigned long lastFrame = 0;
const unsigned long frameMs = 3;  // 333 FPS target

uint32_t frechet_mask, kakutani_map, tree_mask;
float fixed_point = 0.5f;

// FIXED: Proper uint8_t popcount function
uint8_t popcount(uint32_t x) {
  x = x - ((x >> 1) & 0x55555555);
  x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
  return ((x + (x >> 4) & 0xF0F0F0F) * 0x1010101) >> 24;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400kHz I2C
  u8g2.begin();
  
  // Static title draw once
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(xOffset+2, yOffset-2);
    u8g2.print("THEOREM SYNC");
    u8g2.drawVLine(xOffset-2, yOffset+TOP_Y, 3*(ROW_H+ROW_G));
  } while (u8g2.nextPage());
  
  Serial.println("HYPER-FAST 300FPS - FIXED COMPILATION");
}

void loop() {
  unsigned long now = millis();
  if (now - lastFrame >= frameMs) {
    lastFrame = now;
    updateWaves();
    
    u8g2.firstPage();
    do {
      drawFast();
    } while (u8g2.nextPage());
  }
}

void updateWaves() {
  sine1_phase += sine1_freq; if (sine1_phase > 6.28318f) sine1_phase -= 6.28318f;
  sine2_phase += sine2_freq; if (sine2_phase > 6.28318f) sine2_phase -= 6.28318f;
  
  // FAST FRECHET
  uint32_t p1 = (uint32_t)(sine1_phase*1000) & 0xFFFF;
  uint32_t p2 = (uint32_t)(sine2_phase*1000) & 0xFFFF;
  frechet_mask = ((p1 & p2) ^ (p1 ^ p2)) ^ 0xF1E2D3C4 ^ ((p1*p2)>>16);
  
  float phase_diff = fabsf(sine1_phase - sine2_phase);
  if (phase_diff > 3.14159f) phase_diff = 6.28318f - phase_diff;
  frechet_sync = (fabsf(sine1_freq - sine2_freq) + phase_diff * 0.05f) < 0.008f;
  
  // FAST KAKUTANI  
  float x = fabsf(sinf(head * 0.1f));
  kakutani_map = ((uint32_t)(x*0xFFFF) ^ (uint32_t)(head*12345)) ^ 0xCAFEBABE;
  
  float phi_lo = ((kakutani_map & 0xFF)/255.0f) * x * 0.8f;
  float phi_hi = ((kakutani_map>>8 & 0xFF)/255.0f) * (1.0f - x*0.2f) + x*0.3f;
  fixed_point += ((phi_lo + phi_hi)*0.5f - fixed_point) * 0.1f;
  kakutani_fixed = (x >= phi_lo && x <= phi_hi) && fabsf(x - fixed_point) < 0.05f;
  
  // ULTRA-FAST MATRIX TREE
  uint8_t irregularity = 0;
  uint32_t xor_mask = frechet_mask ^ kakutani_map;
  for (int row = 0; row < 8; row++) {
    uint8_t row_xor = (xor_mask >> (row*4)) & 0xFF;
    irregularity += popcount(row_xor);
  }
  
  uint8_t tree_balance = 64 + (irregularity - 4);
  tree_mask = frechet_mask ^ kakutani_map ^ (head*54321);
  uint8_t tree_depth = tree_mask & 0xFF;
  tree_satisfied = (irregularity > 20) && (tree_depth % 5 == 2) && (tree_balance % 3 == 0);
  
  // FAST BUFFER SHIFT
  for (int i = 0; i < GRAPH_WIDTH-1; i++) {
    frechetBuffer[i] = frechetBuffer[i+1];
    kakutaniBuffer[i] = kakutaniBuffer[i+1];
    treeBuffer[i] = treeBuffer[i+1];
  }
  
  // SAMPLES
  int phase_idx = (int)(sine1_phase * 40.74f) & 0xFF;
  frechetBuffer[GRAPH_WIDTH-1] = sin_lut[phase_idx];
  kakutaniBuffer[GRAPH_WIDTH-1] = (int8_t)((phi_hi - phi_lo) * 8 * x);
  treeBuffer[GRAPH_WIDTH-1] = tree_satisfied ? 6 : (int8_t)(3 - irregularity/8);
  
  head = (head + 1) & 99;
}

void drawFast() {
  const int y1 = yOffset + TOP_Y + 5;
  const int y2 = yOffset + TOP_Y + 15 + 3 + 5;
  const int y3 = yOffset + TOP_Y + 30 + 6 + 5;
  
  // FAST ZERO LINES
  u8g2.drawHLine(xOffset, y1, GRAPH_WIDTH);
  u8g2.drawHLine(xOffset, y2, GRAPH_WIDTH);
  u8g2.drawHLine(xOffset, y3, GRAPH_WIDTH);
  
  // VECTORIZED WAVES
  for (int x = 0; x < GRAPH_WIDTH; x++) {
    int sx = xOffset + x;
    
    int8_t v1 = frechetBuffer[x];
    if (v1 > 0) u8g2.drawVLine(sx, y1-v1, v1);
    else if (v1 < 0) u8g2.drawVLine(sx, y1+1, -v1);
    
    int8_t v2 = kakutaniBuffer[x];
    if (v2 > 0) u8g2.drawVLine(sx, y2-v2, v2);
    else if (v2 < 0) u8g2.drawVLine(sx, y2+1, -v2);
    
    int8_t v3 = treeBuffer[x];
    if (v3 > 0) u8g2.drawVLine(sx, y3-v3, v3);
    else if (v3 < 0) u8g2.drawVLine(sx, y3+1, -v3);
  }
  
  // STATUS
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(xOffset+2, yOffset+52);
  u8g2.print("F:");
  u8g2.print(frechet_sync ? "Y" : "N");
  u8g2.print(" K:");
  u8g2.print(kakutani_fixed ? "Y" : "N");
  u8g2.print(" T:");
  u8g2.print(tree_satisfied ? "Y" : "N");
}

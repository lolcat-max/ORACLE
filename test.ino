// FIXED DISPLAY - LARGER WAVE + BOX

#include <Arduino.h>
#include <math.h>
#include <U8g2lib.h>

#define SDA_PIN 5
#define SCL_PIN 6

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN);

const int xOffset = 28;
const int yOffset = 12;
#define GRAPH_WIDTH 72
#define ROW_H 16  // Increased height
#define TOP_Y 5

int8_t kakutaniBuffer[GRAPH_WIDTH];
bool kakutani_fixed;
int head = 0;
unsigned long lastFrame = 0;
const unsigned long frameMs = 1;  // 100 FPS - visible

uint32_t kakutani_map;
float fixed_point = 0.5f;

void setup() {
  Serial.begin(115200);
  u8g2.begin();
  
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(xOffset+2, yOffset-2);
    u8g2.print("KAKUTANI");
    
    // Graph box for visibility
    u8g2.drawFrame(xOffset-1, yOffset+TOP_Y-1, GRAPH_WIDTH+2, ROW_H+2);
    u8g2.drawHLine(xOffset, yOffset+TOP_Y+8, GRAPH_WIDTH);  // Zero line
  } while (u8g2.nextPage());
  
  Serial.println("KAKUTANI DISPLAY FIXED");
}

void loop() {
 
    updateKakutani();
    
    u8g2.firstPage();
    do {
      drawGraph();
    } while (u8g2.nextPage());
  
}

void updateKakutani() {
  static float phase = 0.0f;
  phase += 0.12f;
  if (phase > 6.28318f) phase -= 6.28318f;
  
  float x = fabsf(sinf(phase));
  kakutani_map = ((uint32_t)(x*0xFFFF) ^ (uint32_t)(head*12345)) ^ 0xCAFEBABE;
  
  float phi_lo = ((kakutani_map & 0xFF)/255.0f) * x * 0.8f;
  float phi_hi = ((kakutani_map>>8 & 0xFF)/255.0f) * (1.0f - x*0.2f) + x*0.3f;
  
  fixed_point += ((phi_lo + phi_hi)*0.5f - fixed_point) * 0.15f;
  kakutani_fixed = (x >= phi_lo && x <= phi_hi) && fabsf(x - fixed_point) < 0.04f;
  
  // Scroll + BIGGER sample (-16 to +16)
  for (int i = 0; i < GRAPH_WIDTH-1; i++) {
    kakutaniBuffer[i] = kakutaniBuffer[i+1];
  }
  kakutaniBuffer[GRAPH_WIDTH-1] = (int8_t)((phi_hi - phi_lo - 0.5f) * 32.0f);  // ±16px
  if (kakutani_fixed) kakutaniBuffer[GRAPH_WIDTH-1] = 16;  // Peak on fixed
  
  head = (head + 1) & 127;
  
  // Debug
  Serial.print("x="); Serial.print(x,3);
  Serial.print(" phi="); Serial.print(phi_lo,2); Serial.print("-"); Serial.print(phi_hi,2);
  Serial.print(" fixed="); Serial.println(kakutani_fixed ? "Y" : "N");
}

void drawGraph() {
  const int zeroY = yOffset + TOP_Y + 8;
  
  // Wave - FULL HEIGHT
  for (int x = 0; x < GRAPH_WIDTH; x++) {
    int sx = xOffset + x;
    int8_t v = kakutaniBuffer[x];
    
    if (v > 0) {
      u8g2.drawVLine(sx, zeroY - v, v);
    } else if (v < 0) {
      u8g2.drawVLine(sx, zeroY + 1, -v);
    }
  }
  
  // Status bottom
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(xOffset+2, 58);
  u8g2.print("K:");
  u8g2.print(kakutani_fixed ? "Y" : "N");
  u8g2.print(" map:"); u8g2.print((kakutani_map>>8)&0xFF, HEX);
}

// FIXED HORIZONTAL SKIPPING - CLEAN RENDER
// Uses only drawPixel() - no VLines causing artifacts

#include <Arduino.h>
#include <math.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define SDA_PIN 5
#define SCL_PIN 6

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN);

const int xOffset = 30;
const int yOffset = 12;
#define GRAPH_WIDTH 72
#define ROW_H 12
#define ROW_G 4
#define TOP_Y 5

float sine1_phase = 0.0;
float sine2_phase = PI;
float sine1_freq = 0.08;
float sine2_freq = 0.07;

int8_t sineBuffer[2][GRAPH_WIDTH];
float frechet_dist = 1.0;
bool sync_status = false;
int head = 0;
unsigned long lastFrame = 0;
const unsigned long frameMs = 60;  // SLOWER = stable

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // SLOWER I2C = no artifacts
  u8g2.begin();
  
  for (int w = 0; w < 2; w++) {
    for (int x = 0; x < GRAPH_WIDTH; x++) {
      sineBuffer[w][x] = 0;
    }
  }
  Serial.println("Fixed Horizontal Skip");
}

void loop() {
  unsigned long now = millis();
  if (now - lastFrame >= frameMs) {
    lastFrame = now;
    
    updateSineSamples();
    
    // ATOMIC RENDER - NO SKIP
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setCursor(xOffset, yOffset - 2);
      u8g2.print("SINE SYNC");
      
      u8g2.drawVLine(xOffset - 2, yOffset + TOP_Y, 2 * (ROW_H + ROW_G));
      
      drawCleanSineWaves();  // PIXEL-ONLY
      
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(xOffset + 15, yOffset + 52);
      u8g2.print(sync_status ? "✓" : "-");
      u8g2.setCursor(xOffset + 35, yOffset + 52);
      char buf[6];
      sprintf(buf, "%.1f", frechet_dist * 100);
      u8g2.print(buf);
      
    } while (u8g2.nextPage());
  }
}

void updateSineSamples() {
  sine1_phase += sine1_freq;
  sine2_phase += sine2_freq;
  
  float phase_diff = fabs(sine1_phase - sine2_phase);
  if (phase_diff > PI) phase_diff = 2*PI - phase_diff;
  frechet_dist = fabs(sine1_freq - sine2_freq) + phase_diff * 0.05;
  sync_status = frechet_dist < 0.005;
  
  // Smooth scrolling buffer
  for (int x = 0; x < GRAPH_WIDTH - 1; x++) {
    sineBuffer[0][x] = sineBuffer[0][x + 1];
    sineBuffer[1][x] = sineBuffer[1][x + 1];
  }
  
  // New sample at end
  float t1 = head * 0.2 + sine1_phase;
  float t2 = head * 0.2 + sine2_phase;
  sineBuffer[0][GRAPH_WIDTH-1] = (int8_t)(sin(t1) * 5);
  sineBuffer[1][GRAPH_WIDTH-1] = (int8_t)(sin(t2) * 5);
  
  head = (head + 1) % 100;
}

void drawCleanSineWaves() {
  for (int wave = 0; wave < 2; wave++) {
    int centerY = yOffset + TOP_Y + wave * (ROW_H + ROW_G) + (ROW_H / 2);
    
    // Label
    u8g2.setFont(u8g2_font_4x6_tr);
    char label[4];
    sprintf(label, "S%d", wave + 1);
    u8g2.setCursor(2, centerY + 3);
    u8g2.print(label);
    
    // Zero line - HORIZONTAL PIXELS ONLY
    for (int x = 0; x < GRAPH_WIDTH; x++) {
      u8g2.drawPixel(xOffset + x, centerY);
    }
    
    // WAVE - PIXEL BY PIXEL (NO VLINES = NO SKIP)
    for (int x = 0; x < GRAPH_WIDTH; x++) {
      int8_t val = sineBuffer[wave][x];
      int screenX = xOffset + x;
      
      // Draw center + deflection points
      u8g2.drawPixel(screenX, centerY);           // Center
      if (val > 0) {
        for (int dy = 1; dy <= val; dy++) {
          u8g2.drawPixel(screenX, centerY - dy);  // Up pixels
        }
      } else if (val < 0) {
        for (int dy = 1; dy <= -val; dy++) {
          u8g2.drawPixel(screenX, centerY + dy);  // Down pixels
        }
      }
    }
  }
}

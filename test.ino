#include <Arduino.h>
#include <math.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define SDA_PIN 5
#define SCL_PIN 6

// CORRECTED DISPLAY SETUP: Using memory-optimized page buffer constructor
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL_PIN, /* data=*/ SDA_PIN);

// ADDED: Offsets for the 72x40 visible area
const int xOffset = 30;
const int yOffset = 12;

// Constants and parameters from your original code
#define NUM_CHANNELS 4
#define MATRIX_SIZE 4
#define HISTORY_SIZE 20
#define SMOOTHING_WINDOW 7
#define FRECHET_THRESHOLD 0.030
#define CHARGE_THRESHOLD 0.1
#define SIGNAL_AMPLIFICATION 100.0

#define GRAPH_WIDTH 72
#define GRAPH_HEIGHT 30 // This defines the logical area, not physical screen

// Oscillation parameters
#define OSCILLATION_PROBABILITY 0.15
#define OSCILLATION_MIN_DURATION 50
#define OSCILLATION_MAX_DURATION 150
#define OSCILLATION_AMPLITUDE 0.05

// Analog input pins
const int analogPins[NUM_CHANNELS] = { A0, A1, A2, A3 };

// Data buffers and variables
float channelHistory[NUM_CHANNELS][HISTORY_SIZE];
float smoothedData[NUM_CHANNELS][HISTORY_SIZE];
float baselineVoltages[NUM_CHANNELS];
float chargeAccumulation[NUM_CHANNELS];
float currentMatrix[MATRIX_SIZE][MATRIX_SIZE];
float metricMatrix[MATRIX_SIZE][MATRIX_SIZE];
float seminormVector[MATRIX_SIZE];
float cauchySequence[MATRIX_SIZE];

float frechetMetric = 0.0;
bool isLocallyConvex = false;
bool isMetrizable = false;

// Oscillation state
bool isOscillating = false;
int oscillationCounter = 0;
int oscillationDuration = 0;
float oscillationFrequencies[NUM_CHANNELS];
float oscillationPhases[NUM_CHANNELS];
unsigned long oscillationStartTime = 0;

int historyIndex = 0;
unsigned long lastSampleTime = 0;
const unsigned long sampleIntervalMs = 2; // 50Hz sample rate
int analysisCount = 0;

// Gantt chart globals
#define TASKS NUM_CHANNELS
#define ROW_H 6
#define ROW_G 1
#define TOP_Y 10 // Adjusted to fit nicely with offset

uint8_t gantt[TASKS][GRAPH_WIDTH];
int head = 0;
unsigned long lastFrame = 0;
const unsigned long frameMs = 50; // ~20 FPS

// Function declarations are unchanged
void establishBaseline();
void sampleAllChannels();
// ... and so on

void setup() {
  Serial.begin(115200);
  
  // CORRECTED: Initialize I2C bus first
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();

  // Initialize arrays (your original code)
  for (int i = 0; i < MATRIX_SIZE; i++) {
    cauchySequence[i] = 0.0;
    seminormVector[i] = 0.0;
    oscillationFrequencies[i] = 0.0;
    oscillationPhases[i] = 0.0;
  }
  for (int i = 0; i < TASKS; i++) {
    for (int j = 0; j < GRAPH_WIDTH; j++) {
      gantt[i][j] = 0;
    }
  }

  randomSeed(analogRead(A0) + analogRead(A1));
  establishBaseline();
  Serial.println("Setup complete - Fréchet space with corrected display");
}

// Your original logic for these functions is preserved
bool areChannelsOptimal() {
  if (analysisCount < MATRIX_SIZE) return false;
  bool converging = false;
  for (int i = 1; i < MATRIX_SIZE; i++) {
    if (fabs(cauchySequence[0] - cauchySequence[i]) > FRECHET_THRESHOLD) {
      converging = true;
      break;
    }
  }
  if (!converging) return false;
  for (int ch = 0; ch < 3; ch++) {
    if (seminormVector[ch] <= 100) return false;
  }
  return true;
}

void pushGanttSamples() {
  bool converging = false;
  if (analysisCount >= MATRIX_SIZE) {
    for (int i = 1; i < MATRIX_SIZE; i++) {
      if (fabs(cauchySequence[0] - cauchySequence[i]) > FRECHET_THRESHOLD) {
        converging = true;
        break;
      }
    }
  }
  for (int ch = 0; ch < TASKS; ch++) {
    bool active = converging && (seminormVector[ch] > 100);
    gantt[ch][head] = active ? 1 : 0;
  }
  head = (head + 1) % GRAPH_WIDTH;
}

// CORRECTED: drawGanttChart now uses offsets
void drawGanttChart() {
  // Draw each task row, applying offsets
  for (int ch = 0; ch < TASKS; ch++) {
    int y = yOffset + TOP_Y + ch * (ROW_H + ROW_G);
    int runStart = -1;
    for (int x = 0; x < GRAPH_WIDTH; x++) {
      int col = (head + x) % GRAPH_WIDTH;
      if (gantt[ch][col] && runStart < 0) {
        runStart = x;
      }
      if ((!gantt[ch][col] || x == GRAPH_WIDTH - 1) && runStart >= 0) {
        int runEnd = gantt[ch][col] ? x : x - 1;
        int w = runEnd - runStart + 1;
        if (w > 0) {
          u8g2.drawBox(xOffset + runStart, y, w, ROW_H);
        }
        runStart = -1;
      }
    }
  }
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleTime >= sampleIntervalMs) {
    lastSampleTime = now;
    if (areChannelsOptimal()) {
      Serial.println("YES");
    }
    
    // Your analysis pipeline is unchanged
    updateOscillationState();
    sampleAllChannels();
    applySmoothingFilter();
    updateCurrentMatrix();
    calculateFrechetMetrics();
    updateCauchySequence();
    pushGanttSamples();

    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    analysisCount++;
  }

  if (now - lastFrame >= frameMs) {
    lastFrame = now;

    // CORRECTED: Using page buffer loop for memory safety
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_ncenB08_tr);
      
      // Apply offset to cursor for text
      u8g2.setCursor(xOffset, yOffset + 8);
      u8g2.print("Unity fuse");

      // Draw chart (which now has offsets internally)
      drawGanttChart();

    } while (u8g2.nextPage());
  }
}

// --- All other functions (establishBaseline, sampleAllChannels, etc.) are exactly as you provided them ---

void establishBaseline() {
  Serial.println("Establishing baseline...");
  const int samples = 50;
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float sum = 0;
    for (int i = 0; i < samples; i++) {
      sum += (analogRead(analogPins[ch]) / 4095.0) * 3.3;
      delay(10);
    }
    baselineVoltages[ch] = sum / samples;
    Serial.print("Channel "); Serial.print(ch); Serial.print(" baseline: "); Serial.println(baselineVoltages[ch], 4);
  }
}

void sampleAllChannels() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float voltage = (analogRead(analogPins[ch]) / 4095.0) * 3.3;
    voltage += getOscillationValue(ch);
    channelHistory[ch][historyIndex] = voltage;
  }
}

void updateOscillationState() {
  if (!isOscillating) {
    if (random(1000) < (OSCILLATION_PROBABILITY * 1000)) {
      isOscillating = true;
      oscillationCounter = 0;
      oscillationDuration = random(OSCILLATION_MIN_DURATION, OSCILLATION_MAX_DURATION);
      oscillationStartTime = millis();
      for (int i = 0; i < NUM_CHANNELS; i++) {
        oscillationFrequencies[i] = random(20, 100) / 10.0;
        oscillationPhases[i] = random(0, 628) / 100.0;
      }
    }
  } else {
    oscillationCounter++;
    if (oscillationCounter >= oscillationDuration) {
      isOscillating = false;
    }
  }
}

float getOscillationValue(int channel) {
  if (!isOscillating) return 0.0;
  float timeInSeconds = (millis() - oscillationStartTime) / 1000.0;
  float omega = 2.0 * PI * oscillationFrequencies[channel];
  float fundamental = sin(omega * timeInSeconds + oscillationPhases[channel]);
  float harmonic = 0.3 * sin(2.0 * omega * timeInSeconds + oscillationPhases[channel] * 0.5);
  float progress = (float)oscillationCounter / (float)oscillationDuration;
  float envelope = 1.0;
  if (progress < 0.1) envelope = progress / 0.1;
  else if (progress > 0.9) envelope = (1.0 - progress) / 0.1;
  return (fundamental + harmonic) * OSCILLATION_AMPLITUDE * envelope;
}

void applySmoothingFilter() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float sum = 0.0;
    int count = 0;
    for (int i = 0; i < SMOOTHING_WINDOW; i++) {
      int idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
      sum += channelHistory[ch][idx];
      count++;
    }
    smoothedData[ch][historyIndex] = sum / count;
  }
}

void updateCurrentMatrix() {
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++) {
      if (i == j) {
        float diff = smoothedData[i][historyIndex] - baselineVoltages[i];
        currentMatrix[i][j] = diff * SIGNAL_AMPLIFICATION;
      } else {
        currentMatrix[i][j] = calculateCorrelation(i, j);
      }
    }
  }
}

float calculateCorrelation(int ch1, int ch2) {
  if (analysisCount < 3) return 0.0;
  float sum1 = 0, sum2 = 0, sum12 = 0, sumSq1 = 0, sumSq2 = 0;
  int samples = min(HISTORY_SIZE, analysisCount);
  for (int i = 0; i < samples; i++) {
    int idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
    float val1 = smoothedData[ch1][idx] - baselineVoltages[ch1];
    float val2 = smoothedData[ch2][idx] - baselineVoltages[ch2];
    sum1 += val1; sum2 += val2; sum12 += val1 * val2;
    sumSq1 += val1 * val1; sumSq2 += val2 * val2;
  }
  float mean1 = sum1 / samples;
  float mean2 = sum2 / samples;
  float covariance = (sum12 / samples) - mean1 * mean2;
  float std1 = sqrt(max(0.0f, (sumSq1 / samples) - mean1 * mean1));
  float std2 = sqrt(max(0.0f, (sumSq2 / samples) - mean2 * mean2));
  if (std1 > 0.0001 && std2 > 0.0001) {
    return (covariance / (std1 * std2)) * SIGNAL_AMPLIFICATION;
  }
  return 0.0;
}

void calculateFrechetMetrics() {
  for (int i = 0; i < MATRIX_SIZE; i++) {
    float norm = 0.0;
    for (int j = 0; j < MATRIX_SIZE; j++) {
      norm += currentMatrix[i][j] * currentMatrix[i][j];
    }
    seminormVector[i] = sqrt(norm);
  }
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++) {
      metricMatrix[i][j] = fabs(seminormVector[i] - seminormVector[j]);
    }
  }
  frechetMetric = 0.0;
  for (int i = 0; i < MATRIX_SIZE; i++) {
    frechetMetric += seminormVector[i];
  }
  frechetMetric /= MATRIX_SIZE;
}

void updateCauchySequence() {
  for (int i = MATRIX_SIZE - 1; i > 0; i--) {
    cauchySequence[i] = cauchySequence[i - 1];
  }
  cauchySequence[0] = frechetMetric;
}

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

// DISPLAY SETUP
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL_PIN, /* data=*/ SDA_PIN);

// VISUAL OFFSETS
const int xOffset = 30;
const int yOffset = 12;

// SYSTEM CONSTANTS
#define NUM_CHANNELS 4
#define MATRIX_SIZE 4
#define HISTORY_SIZE 20
#define SMOOTHING_WINDOW 7
#define FRECHET_THRESHOLD 0.0030
#define SIGNAL_AMPLIFICATION 100.0

// GRAPHICS CONSTANTS
#define GRAPH_WIDTH 72
#define GRAPH_HEIGHT 30
#define TASKS NUM_CHANNELS
#define ROW_H 8          // Increased height for polarization swing
#define ROW_G 2          // Gap between rows
#define TOP_Y 5
#define POLARIZATION_GAIN 30.0 // Adjust sensitivity of the wave display

// OSCILLATION CONSTANTS
#define OSCILLATION_PROBABILITY 0.15
#define OSCILLATION_MIN_DURATION 50
#define OSCILLATION_MAX_DURATION 150
#define OSCILLATION_AMPLITUDE 0.05

// PINS
const int analogPins[NUM_CHANNELS] = { A0, A1, A2, A3 };

// DATA BUFFERS
float channelHistory[NUM_CHANNELS][HISTORY_SIZE];
float smoothedData[NUM_CHANNELS][HISTORY_SIZE];
float baselineVoltages[NUM_CHANNELS];
float currentMatrix[MATRIX_SIZE][MATRIX_SIZE];
float seminormVector[MATRIX_SIZE];

// METRIC BUFFERS
float distanceMatrix[MATRIX_SIZE][MATRIX_SIZE]; 
float cauchySequence[MATRIX_SIZE];

// CHART BUFFER (Changed to int8_t for Signed Polarization)
int8_t gantt[TASKS][GRAPH_WIDTH]; 

// STATE
bool isOscillating = false;
int oscillationCounter = 0;
int oscillationDuration = 0;
float oscillationFrequencies[NUM_CHANNELS];
float oscillationPhases[NUM_CHANNELS];
unsigned long oscillationStartTime = 0;

int historyIndex = 0;
unsigned long lastSampleTime = 0;
const unsigned long sampleIntervalMs = 2;
int analysisCount = 0;

int head = 0;
unsigned long lastFrame = 0;
const unsigned long frameMs = 50;

// FUNCTION PROTOTYPES
void establishBaseline();
void sampleAllChannels();
void updateOscillationState();
float getOscillationValue(int channel);
void applySmoothingFilter();
void updateCurrentMatrix();
float calculateCorrelation(int ch1, int ch2);
void updateDistanceAndCauchyMetrics();
float calculateDistanceMetric(int ch1, int ch2);
bool areChannelsOptimal();

void setup() {
  Serial.begin(115200);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();

  // Initialize buffers
  for (int i = 0; i < MATRIX_SIZE; i++) {
    cauchySequence[i] = 0.0;
    seminormVector[i] = 0.0;
    for (int j = 0; j < MATRIX_SIZE; j++) {
      distanceMatrix[i][j] = 0.0;
    }
  }
  // Initialize Gantt with 0 (flatline)
  for (int i = 0; i < TASKS; i++) {
    for (int j = 0; j < GRAPH_WIDTH; j++) {
      gantt[i][j] = 0;
    }
  }

  randomSeed(analogRead(A0) + analogRead(A1));
  establishBaseline();
  Serial.println("Setup complete - Polarization Mode");
}

/**
 * @brief Captures the polarity (direction) and magnitude of the signal
 * instead of just a binary On/Off state.
 */
void pushGanttSamples() {
  for (int ch = 0; ch < TASKS; ch++) {
    // 1. Get signal relative to baseline
    float signalDelta = smoothedData[ch][historyIndex] - baselineVoltages[ch];
    
    // 2. Apply Gain and clamp to fit the row height (±3 pixels approx)
    // Positive values go UP, Negative values go DOWN
    int val = (int)(signalDelta * POLARIZATION_GAIN);
    
    // 3. Constrain to fit inside ROW_H/2 (e.g., -4 to +4)
    int maxDeflection = (ROW_H / 2) - 1;
    if (val > maxDeflection) val = maxDeflection;
    if (val < -maxDeflection) val = -maxDeflection;
    
    // 4. Store signed integer
    gantt[ch][head] = (int8_t)val;
  }
  head = (head + 1) % GRAPH_WIDTH;
}

/**
 * @brief Renders 2D Polarization (Vertical deflection vs Time)
 */
void drawGanttChart() {
  for (int ch = 0; ch < TASKS; ch++) {
    // Calculate the vertical center of this channel's row
    int centerY = yOffset + TOP_Y + ch * (ROW_H + ROW_G) + (ROW_H / 2);
    
    // Draw the zero-crossing baseline (faintly)
    // u8g2.drawPixel(xOffset, centerY); // Optional axis dot
    
    for (int x = 0; x < GRAPH_WIDTH; x++) {
      // Circular buffer access
      int col = (head + x) % GRAPH_WIDTH;
      int8_t val = gantt[ch][col];
      
      int screenX = xOffset + x;
      
      if (val == 0) {
        // Draw flat baseline
        u8g2.drawPixel(screenX, centerY);
      } else {
        // Draw Vertical Vector (Polarization)
        // if val is positive, we draw UP from center. 
        // if val is negative, we draw DOWN from center.
        
        // Note: In screen coords, Y increases downwards.
        // So "Up" is (centerY - val)
        int yTarget = centerY - val;
        
        // Draw line from Center to Target
        u8g2.drawVLine(screenX, min(centerY, yTarget), abs(val) + 1);
      }
    }
  }
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleTime >= sampleIntervalMs) {
    lastSampleTime = now;
    
    updateOscillationState();
    sampleAllChannels();
    applySmoothingFilter();
    updateCurrentMatrix(); 
    updateDistanceAndCauchyMetrics(); 
    
    // Update the polarization chart data
    pushGanttSamples();

    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    analysisCount++;
  }

  if (now - lastFrame >= frameMs) {
    lastFrame = now;
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setCursor(xOffset, yOffset - 2);
      u8g2.print("Polarization"); // Updated Title
      
      // Draw a small axis guide on the left
      u8g2.drawVLine(xOffset - 2, yOffset + TOP_Y, TASKS * (ROW_H + ROW_G));
      
      drawGanttChart();
    } while (u8g2.nextPage());
  }
}

// --- METRIC FUNCTIONS (Translation Invariant) ---

float calculateDistanceMetric(int ch1, int ch2) {
  float sumOfSquares = 0.0;
  int samples = min(HISTORY_SIZE, analysisCount);
  if (samples == 0) return 0.0;

  for (int i = 0; i < samples; i++) {
    int idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
    float diff = smoothedData[ch1][idx] - smoothedData[ch2][idx];
    sumOfSquares += diff * diff;
  }
  return sqrt(sumOfSquares / samples); 
}

void updateDistanceAndCauchyMetrics() {
  // Update seminorms
  for (int i = 0; i < MATRIX_SIZE; i++) {
    float norm = 0.0;
    for (int j = 0; j < MATRIX_SIZE; j++) {
      norm += currentMatrix[i][j] * currentMatrix[i][j];
    }
    seminormVector[i] = sqrt(norm);
  }

  // Update Distance Matrix
  float frobeniusNormSq = 0.0;
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = i; j < MATRIX_SIZE; j++) {
      if (i == j) {
        distanceMatrix[i][j] = 0.0;
      } else {
        float dist = calculateDistanceMetric(i, j);
        distanceMatrix[i][j] = dist;
        distanceMatrix[j][i] = dist; 
      }
      frobeniusNormSq += distanceMatrix[i][j] * distanceMatrix[i][j];
    }
  }

  float systemStateMetric = sqrt(frobeniusNormSq);

  for (int i = MATRIX_SIZE - 1; i > 0; i--) {
    cauchySequence[i] = cauchySequence[i - 1];
  }
  cauchySequence[0] = systemStateMetric;
}

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

// --- DATA ACQUISITION & UTILS ---

void establishBaseline() {
  Serial.println("Baseline calibration...");
  const int samples = 50;
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float sum = 0;
    for (int i = 0; i < samples; i++) {
      sum += (analogRead(analogPins[ch]) / 4095.0) * 3.3;
      delay(10);
    }
    baselineVoltages[ch] = sum / samples;
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
  return (sin(omega * timeInSeconds + oscillationPhases[channel]) * OSCILLATION_AMPLITUDE);
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

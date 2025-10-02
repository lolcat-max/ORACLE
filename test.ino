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

// OLED display setup
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Constants and parameters
#define NUM_CHANNELS 4
#define MATRIX_SIZE 4
#define HISTORY_SIZE 20
#define SMOOTHING_WINDOW 7
#define FRECHET_THRESHOLD 0.05
#define CHARGE_THRESHOLD 0.1
#define SIGNAL_AMPLIFICATION 100.0

#define GRAPH_WIDTH 72
#define GRAPH_HEIGHT 30

// Oscillation parameters
#define OSCILLATION_PROBABILITY 0.15  // 15% chance per cycle
#define OSCILLATION_MIN_DURATION 50   // Min samples (1 second at 50Hz)
#define OSCILLATION_MAX_DURATION 150  // Max samples (3 seconds)
#define OSCILLATION_AMPLITUDE 0.05    // Voltage amplitude

// Analog input pins (adjust as per hardware)
const int analogPins[NUM_CHANNELS] = {A0, A1, A2, A3};

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

float frechetHistory[GRAPH_WIDTH];
int frechetHistoryIndex = 0;

// Oscillation state
bool isOscillating = false;
int oscillationCounter = 0;
int oscillationDuration = 0;
float oscillationFrequencies[NUM_CHANNELS];
float oscillationPhases[NUM_CHANNELS];
unsigned long oscillationStartTime = 0;

int historyIndex = 0;
unsigned long lastSampleTime = 0;
const unsigned long sampleIntervalMs = 20; // 50Hz sample rate
int analysisCount = 0;

// Function declarations
void establishBaseline();
void sampleAllChannels();
void applySmoothingFilter();
void updateCurrentMatrix();
void calculateFrechetMetrics();
void updateCauchySequence();
float calculateFrechetProbability();
float calculateCorrelation(int ch1, int ch2);
void updateOscillationState();
float getOscillationValue(int channel);
void updateFrechetHistory(float value);
void drawGraph();

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();

  // Initialize arrays
  for (int i = 0; i < MATRIX_SIZE; i++) {
    cauchySequence[i] = 0.0;
    seminormVector[i] = 0.0;
    oscillationFrequencies[i] = 0.0;
    oscillationPhases[i] = 0.0;
  }
  randomSeed(analogRead(A0) + analogRead(A1));

  establishBaseline();
  Serial.println("Setup complete - oscillations enabled");
}

void loop() {
  unsigned long now = millis();
  if (now - lastSampleTime >= sampleIntervalMs) {
    lastSampleTime = now;

    updateOscillationState();
    sampleAllChannels();
    applySmoothingFilter();
    updateCurrentMatrix();
    calculateFrechetMetrics();
    updateCauchySequence();

    updateFrechetHistory(frechetMetric);

    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    analysisCount++;

    // Calculate entropy from seminorm variance
    float meanSeminorm = 0.0;
    for (int i = 0; i < MATRIX_SIZE; i++) {
      meanSeminorm += seminormVector[i];
    }
    meanSeminorm /= MATRIX_SIZE;

    float variance = 0.0;
    for (int i = 0; i < MATRIX_SIZE; i++) {
      float diff = seminormVector[i] - meanSeminorm;
      variance += diff * diff;
    }
    variance /= MATRIX_SIZE;
    bool lowEntropy = (variance < 10.0);

    u8g2.clearBuffer();
u8g2.setFont(u8g2_font_ncenB08_tr);

u8g2.setCursor(0, 10);
u8g2.print("F:");
u8g2.print(frechetMetric, 6);

// Draw graph inside display bounds
// Clear the graph area (start from y=15 for example)
u8g2.setDrawColor(0);
u8g2.drawBox(0, 15, GRAPH_WIDTH, 25);
u8g2.setDrawColor(1);

float maxVal = frechetHistory[0];
float minVal = frechetHistory[0];
for (int i = 1; i < GRAPH_WIDTH; i++) {
  if (frechetHistory[i] > maxVal) maxVal = frechetHistory[i];
  if (frechetHistory[i] < minVal) minVal = frechetHistory[i];
}
if (fabs(maxVal - minVal) < 0.0001) maxVal = minVal + 1.0;

for (int i = 0; i < GRAPH_WIDTH; i++) {
  int idx = (frechetHistoryIndex + i) % GRAPH_WIDTH;
  float normalized = (frechetHistory[idx] - minVal) / (maxVal - minVal);
  int barHeight = (int)(normalized * 25);
  u8g2.drawLine(i, 40, i, 40 - barHeight);
}

u8g2.sendBuffer();
  }
}

void establishBaseline() {
  Serial.println("Establishing baseline...");
  const int samples = 50;
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float sum = 0;
    for (int i = 0; i < samples; i++) {
      int rawValue = analogRead(analogPins[ch]);
      float voltage = (rawValue / 4095.0) * 3.3;
      sum += voltage;
      delay(10);
    }
    baselineVoltages[ch] = sum / samples;
    Serial.print("Channel ");
    Serial.print(ch);
    Serial.print(" baseline: ");
    Serial.println(baselineVoltages[ch], 4);
  }
}

void sampleAllChannels() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    int rawValue = analogRead(analogPins[ch]);
    float voltage = (rawValue / 4095.0) * 3.3;
    voltage += getOscillationValue(ch); // add oscillation
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
      Serial.println(">>> Oscillation started");
    }
  } else {
    oscillationCounter++;
    if (oscillationCounter >= oscillationDuration) {
      isOscillating = false;
      Serial.println("<<< Oscillation ended");
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

  float sum1 = 0, sum2 = 0, sum12 = 0;
  float sumSq1 = 0, sumSq2 = 0;
  int samples = min(HISTORY_SIZE, analysisCount);

  for (int i = 0; i < samples; i++) {
    int idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
    float val1 = smoothedData[ch1][idx] - baselineVoltages[ch1];
    float val2 = smoothedData[ch2][idx] - baselineVoltages[ch2];
    sum1 += val1;
    sum2 += val2;
    sum12 += val1 * val2;
    sumSq1 += val1 * val1;
    sumSq2 += val2 * val2;
  }

  float mean1 = sum1 / samples;
  float mean2 = sum2 / samples;
  float covariance = (sum12 / samples) - mean1 * mean2;

  float std1 = sqrt((sumSq1 / samples) - mean1 * mean1);
  float std2 = sqrt((sumSq2 / samples) - mean2 * mean2);

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

  // Use sum of seminorms (normalized)
  frechetMetric = 0.0;
  for (int i = 0; i < MATRIX_SIZE; i++) {
    frechetMetric += seminormVector[i];
  }
  frechetMetric /= MATRIX_SIZE;
}

float calculateFrechetProbability() {
  float normalized = frechetMetric / (1.0 + frechetMetric);
  float baseProbability = 1.0 / (1.0 + exp(-8.0 * (normalized - 0.5)));

  float meanSeminorm = 0.0;
  for (int i = 0; i < MATRIX_SIZE; i++) {
    meanSeminorm += seminormVector[i];
  }
  meanSeminorm /= MATRIX_SIZE;

  float variance = 0.0;
  for (int i = 0; i < MATRIX_SIZE; i++) {
    float diff = seminormVector[i] - meanSeminorm;
    variance += diff * diff;
  }
  variance /= MATRIX_SIZE;
  float diversityFactor = 1.0 - exp(-variance * 2.0);

  float oscillationBoost = isOscillating ? 0.25 : 0.0;

  float changeRate = 0.0;
  if (analysisCount >= 2) {
    changeRate = fabs(cauchySequence[0] - cauchySequence[1]) * 10.0;
    changeRate = min(changeRate, 0.3f);
  }

  float prob = baseProbability * 0.5 +
               diversityFactor * 0.2 +
               oscillationBoost +
               changeRate;

  float noise = (random(0, 100) - 50) / 1000.0;
  prob += noise;

  return constrain(prob, 0.0, 1.0);
}

void updateCauchySequence() {
  for (int i = MATRIX_SIZE - 1; i > 0; i--) {
    cauchySequence[i] = cauchySequence[i - 1];
  }
  cauchySequence[0] = frechetMetric;

  if (analysisCount < MATRIX_SIZE) {
    isLocallyConvex = false;
    isMetrizable = false;
    return;
  }

  bool converging = true;
  for (int i = 1; i < MATRIX_SIZE; i++) {
    if (fabs(cauchySequence[0] - cauchySequence[i]) > FRECHET_THRESHOLD) {
      converging = false;
    }
  }

  isLocallyConvex = converging && (frechetMetric > 0.1);
  isMetrizable = calculateFrechetProbability() > 0.5;
}

void updateFrechetHistory(float value) {
  frechetHistory[frechetHistoryIndex] = value;
  frechetHistoryIndex = (frechetHistoryIndex + 1) % GRAPH_WIDTH;
}

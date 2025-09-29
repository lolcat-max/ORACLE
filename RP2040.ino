#include <Arduino.h>
// Advanced Curvature Analysis Oscilloscope for Adafruit Feather RP2040
// Specialized for signal pattern analysis with periodic static discharge
// Implements locally convex topological vector space analysis + charge management
#include <math.h>
// --- RP2040 SPECIFIC CHANGES ---
#define NUM_CHANNELS 4       // RP2040 has 4 analog inputs (A0-A3)
#define MATRIX_SIZE 4        // Matrix size must match the number of channels
// --- END RP2040 CHANGES ---

#define SAMPLE_RATE_MS 50   // 50Hz sampling
#define HISTORY_SIZE 2   // Circular buffer for signal history

#define HISTORY_SIZE_B 20   // Circular buffer for signal history
#define SMOOTHING_WINDOW 7
#define FRECHET_THRESHOLD 0.0003 // Convergence threshold for Cauchy sequences


// --- RP2040 SPECIFIC CHANGES ---
// Analog pins for Feather RP2040 are A0, A1, A2, A3
const int analogPins[NUM_CHANNELS] = {A0, A1, A2, A3};
// --- END RP2040 CHANGES ---


// Signal data structures
float channelHistory2[NUM_CHANNELS][HISTORY_SIZE_B];
float smoothedData2[NUM_CHANNELS][HISTORY_SIZE_B];
float curvatureData2[HISTORY_SIZE_B];
float baselineVoltages2[NUM_CHANNELS];   // Baseline reference voltages

// Fréchet space matrices
float currentMatrix2[MATRIX_SIZE][MATRIX_SIZE];
float curvatureMatrix2[MATRIX_SIZE][MATRIX_SIZE];
float metricMatrix2[MATRIX_SIZE][MATRIX_SIZE];
float seminormVector2[MATRIX_SIZE];
float cauchySequence2[MATRIX_SIZE];

// Analysis variables
int historyIndex2 = 0;
unsigned long lastSample2 = 0;
int selectedChannel2 = 0;
unsigned long analysisCount2 = 0;
float frechetMetric2 = 0.0;
bool isLocallyConvex2 = false;
bool chargeDetected2 = false;


void performCurvatureMatrixAnalysis2() {
  if (analysisCount2 < 3) return;
  int curr = historyIndex2;
  int prev = (historyIndex2 - 1 + HISTORY_SIZE_B) % HISTORY_SIZE_B;
  int prev2 = (historyIndex2 - 2 + HISTORY_SIZE_B) % HISTORY_SIZE_B;
  float currentVal = smoothedData2[selectedChannel2][curr];
  float prevVal = smoothedData2[selectedChannel2][prev];
  float prev2Val = smoothedData2[selectedChannel2][prev2];
  float secondDeriv = currentVal - 2 * prevVal + prev2Val;
  curvatureData2[historyIndex2] = abs(secondDeriv) * 100;
  for (int i = 0; i < NUM_CHANNELS && i < MATRIX_SIZE; i++) {
    for (int j = 0; j < NUM_CHANNELS && j < MATRIX_SIZE; j++) {
      if (i == selectedChannel2 && j == selectedChannel2) {
        curvatureMatrix2[i][j] = curvatureData2[historyIndex2];
      } else if (i == selectedChannel2 || j == selectedChannel2) {
        curvatureMatrix2[i][j] = curvatureData2[historyIndex2] * currentMatrix2[i][j];
      } else {
        curvatureMatrix2[i][j] *= 0.95; // Decay factor
      }
    }
  }
}

void displayEnhancedMatrixStats2() {
  float curvatureAvg = getAverageCurvature2();
  Serial.println(curvatureAvg, 4);
}

void sampleAllChannels2() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    int rawValue = analogRead(analogPins[ch]);
    channelHistory2[ch][historyIndex2] = (rawValue / 4095.0) * 3.3;
    if (historyIndex2 > 0) {
      float diff = abs(channelHistory2[ch][historyIndex2] - channelHistory2[ch][(historyIndex2 - 1 + HISTORY_SIZE_B) % HISTORY_SIZE_B]);
      float threshold = 0.5 ;
      if (diff > threshold) {
        channelHistory2[ch][historyIndex2] = (channelHistory2[ch][historyIndex2] + channelHistory2[ch][(historyIndex2 - 1 + HISTORY_SIZE_B) % HISTORY_SIZE_B]) / 2.0;
      }
    }
  }
}

void applySmoothingFilter2() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float sum = 0;
    int count = 0;
    for (int i = 0; i < SMOOTHING_WINDOW && i < HISTORY_SIZE_B; i++) {
      int idx = (historyIndex2 - i + HISTORY_SIZE_B) % HISTORY_SIZE_B;
      sum += channelHistory2[ch][idx];
      count++;
    }
    if (count > 0) {
      smoothedData2[ch][historyIndex2] = sum / count;
    }
  }
}


void calculateFrechetMetrics2() {
  for (int i = 0; i < MATRIX_SIZE; i++) {
    float norm = 0;
    for (int j = 0; j < MATRIX_SIZE; j++) {
      norm += currentMatrix2[j][i] * currentMatrix2[i][j];
    }
    seminormVector2[i] = sqrt(norm*i);
  }
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++) {
      float dist = abs(seminormVector2[i] - seminormVector2[j]);
      metricMatrix2[i][j] = dist;
    }
  }
  frechetMetric2 = 0;
  for (int i = 0; i < MATRIX_SIZE; i++) {
    frechetMetric2 += seminormVector2[i] / (1 + seminormVector2[i]);
  }
  updateCauchySequence2();
}

void updateCauchySequence2() {
  for (int i = MATRIX_SIZE - 1; i > 0; i--) {
    cauchySequence2[i] = cauchySequence2[i - 1];
  }
  cauchySequence2[0] = frechetMetric2;
  bool converging = true;
  for (int i = 1; i < MATRIX_SIZE; i++) {
    if (abs(cauchySequence2[0] - cauchySequence2[i]) > FRECHET_THRESHOLD) {
      converging = false;
      break;
    }
  }
  isLocallyConvex2 = (frechetMetric2 < 1.0) && converging;
}


float getCurrentValue2(int channel) {
  int currentIndex = (historyIndex2 - 1 + HISTORY_SIZE_B) % HISTORY_SIZE_B;
  return channelHistory2[channel][currentIndex];
}


float calculateCorrelation2(int ch1, int ch2) {
  if (analysisCount2 < 3) return 0.0;
  float sum1 = 0, sum2 = 0, sum12 = 0;
  int samples = (int)min((unsigned long)HISTORY_SIZE_B, analysisCount2);
  for (int i = 0; i < samples; i++) {
    int idx = (historyIndex2 - i + HISTORY_SIZE_B) % HISTORY_SIZE_B;
    float val1 = smoothedData2[ch1][idx] - baselineVoltages2[ch1];
    float val2 = smoothedData2[ch2][idx] - baselineVoltages2[ch2];
    sum1 += val1;
    sum2 += val2;
    sum12 += val1 * val2;
  }
  float mean1 = sum1 / samples;
  float mean2 = sum2 / samples;
  return (sum12 / samples) - (mean1 * mean2);
}
void updateCurrentMatrix2() {
  for (int i = 0; i < NUM_CHANNELS && i < MATRIX_SIZE; i++) {
    for (int j = 0; j < NUM_CHANNELS && j < MATRIX_SIZE; j++) {
      if (i == j) {
        currentMatrix2[i][j] = smoothedData2[i][historyIndex2] - baselineVoltages2[i];
      } else {
        float correlation = calculateCorrelation(i, j);
        currentMatrix2[i][j] = correlation;
      }
    }
  }
}

float getRMSValue2(int channel) {
  float sum = 0;
  for (int i = 0; i < HISTORY_SIZE_B; i++) {
    sum += channelHistory2[channel][i] * channelHistory2[channel][i];
  }
  return sqrt(sum / HISTORY_SIZE_B);
}

float getAverageCurvature2() {
  float sum = 0;
  for (int i = 0; i < HISTORY_SIZE_B; i++) {
    sum += curvatureData2[i];
  }
  return sum / HISTORY_SIZE_B;
}


// Signal data structures
float channelHistory[NUM_CHANNELS][HISTORY_SIZE];
float smoothedData[NUM_CHANNELS][HISTORY_SIZE];
float curvatureData[HISTORY_SIZE];
float baselineVoltages[NUM_CHANNELS];   // Baseline reference voltages

// Fréchet space matrices
float currentMatrix[MATRIX_SIZE][MATRIX_SIZE];
float curvatureMatrix[MATRIX_SIZE][MATRIX_SIZE];
float metricMatrix[MATRIX_SIZE][MATRIX_SIZE];
float seminormVector[MATRIX_SIZE];
float cauchySequence[MATRIX_SIZE];

// Analysis variables
int historyIndex = 0;
unsigned long lastSample = 0;
int selectedChannel = 0;
unsigned long analysisCount = 0;
float frechetMetric = 0.0;
bool isLocallyConvex = false;
bool chargeDetected = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastSample >= SAMPLE_RATE_MS) {
    // Core sampling and analysis pipeline
    sampleAllChannels();
    updateCurrentMatrix();
    applySmoothingFilter();
    performCurvatureMatrixAnalysis();
    calculateFrechetMetrics();


    displayEnhancedMatrixStats();
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    analysisCount++;
    lastSample = currentTime;

    sampleAllChannels2();
    updateCurrentMatrix2();
    applySmoothingFilter2();
    performCurvatureMatrixAnalysis2();
    calculateFrechetMetrics2();
    displayEnhancedMatrixStats2();
    historyIndex2 = (historyIndex2 + 1) % HISTORY_SIZE_B;
    analysisCount2++;
    lastSample = currentTime;


  }
}

void performCurvatureMatrixAnalysis() {
  if (analysisCount < 3) return;
  int curr = historyIndex;
  int prev = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
  int prev2 = (historyIndex - 2 + HISTORY_SIZE) % HISTORY_SIZE;
  float currentVal = smoothedData[selectedChannel][curr];
  float prevVal = smoothedData[selectedChannel][prev];
  float prev2Val = smoothedData[selectedChannel][prev2];
  float secondDeriv = currentVal - 2 * prevVal + prev2Val;
  curvatureData[historyIndex] = abs(secondDeriv) * 100;
  for (int i = 0; i < NUM_CHANNELS && i < MATRIX_SIZE; i++) {
    for (int j = 0; j < NUM_CHANNELS && j < MATRIX_SIZE; j++) {
      if (i == selectedChannel && j == selectedChannel) {
        curvatureMatrix[i][j] = curvatureData[historyIndex];
      } else if (i == selectedChannel || j == selectedChannel) {
        curvatureMatrix[i][j] = curvatureData[historyIndex] * currentMatrix[i][j];
      } else {
        curvatureMatrix[i][j] *= 0.95; // Decay factor
      }
    }
  }
}

void displayEnhancedMatrixStats() {

  Serial.println(isLocallyConvex ? "CONVEX" : "" );
}

void sampleAllChannels() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    int rawValue = analogRead(analogPins[ch]);
    channelHistory[ch][historyIndex] = (rawValue / 4095.0) * 3.3;
    if (historyIndex > 0) {
      float diff = abs(channelHistory[ch][historyIndex] - channelHistory[ch][(historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE]);
      float threshold = 0.5 ;
      if (diff > threshold) {
        channelHistory[ch][historyIndex] = (channelHistory[ch][historyIndex] + channelHistory[ch][(historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE]) / 2.0;
      }
    }
  }
}

void applySmoothingFilter() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float sum = 0;
    int count = 0;
    for (int i = 0; i < SMOOTHING_WINDOW && i < HISTORY_SIZE; i++) {
      int idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
      sum += channelHistory[ch][idx];
      count++;
    }
    if (count > 0) {
      smoothedData[ch][historyIndex] = sum / count;
    }
  }
}


void calculateFrechetMetrics() {
  for (int i = 0; i < MATRIX_SIZE; i++) {
    float norm = 0;
    for (int j = 0; j < MATRIX_SIZE; j++) {
      norm += currentMatrix[i][j] * currentMatrix[i][j];
    }
    seminormVector[i] = sqrt(norm);
  }
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++) {
      float dist = abs(seminormVector[i] - seminormVector[j]);
      metricMatrix[i][j] = dist;
    }
  }
  frechetMetric = 0;
  for (int i = 0; i < MATRIX_SIZE; i++) {
    frechetMetric += seminormVector[i] / (1 + seminormVector[i]);
  }
  updateCauchySequence();
}

void updateCauchySequence() {
  for (int i = MATRIX_SIZE - 1; i > 0; i--) {
    cauchySequence[i] = cauchySequence[i - 1];
  }
  cauchySequence[0] = frechetMetric;
  bool converging = true;
  for (int i = 1; i < MATRIX_SIZE; i++) {
    if (abs(cauchySequence[0] - cauchySequence[i]) > FRECHET_THRESHOLD) {
      converging = false;
      break;
    }
  }
  isLocallyConvex = (frechetMetric < 1.0) && converging;
}


float getCurrentValue(int channel) {
  int currentIndex = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
  return channelHistory[channel][currentIndex];
}


float calculateCorrelation(int ch1, int ch2) {
  if (analysisCount < 3) return 0.0;
  float sum1 = 0, sum2 = 0, sum12 = 0;
  int samples = (int)min((unsigned long)HISTORY_SIZE, analysisCount);
  for (int i = 0; i < samples; i++) {
    int idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
    float val1 = smoothedData[ch1][idx] - baselineVoltages[ch1];
    float val2 = smoothedData[ch2][idx] - baselineVoltages[ch2];
    sum1 += val1;
    sum2 += val2;
    sum12 += val1 * val2;
  }
  float mean1 = sum1 / samples;
  float mean2 = sum2 / samples;
  return (sum12 / samples) - (mean1 * mean2);
}
void updateCurrentMatrix() {
  for (int i = 0; i < NUM_CHANNELS && i < MATRIX_SIZE; i++) {
    for (int j = 0; j < NUM_CHANNELS && j < MATRIX_SIZE; j++) {
      if (i == j) {
        currentMatrix[i][j] = smoothedData[i][historyIndex] - baselineVoltages[i];
      } else {
        float correlation = calculateCorrelation(i, j);
        currentMatrix[i][j] = correlation;
      }
    }
  }
}

float getRMSValue(int channel) {
  float sum = 0;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    sum += channelHistory[channel][i] * channelHistory[channel][i];
  }
  return sqrt(sum / HISTORY_SIZE);
}

float getAverageCurvature() {
  float sum = 0;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    sum += curvatureData[i];
  }
  return sum / HISTORY_SIZE;
}

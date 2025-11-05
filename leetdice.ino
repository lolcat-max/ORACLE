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

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL_PIN, /* data=*/ SDA_PIN);

const int xOffset = 28;
const int yOffset = 20;

#define NUM_CHANNELS 4
#define MATRIX_SIZE 4
#define HISTORY_SIZE 20
#define SMOOTHING_WINDOW 7
#define FRECHET_THRESHOLD 0.0030
#define CHARGE_THRESHOLD 0.1
#define SIGNAL_AMPLIFICATION 100.0
#define GRAPH_WIDTH 72
#define GRAPH_HEIGHT 30
#define OSCILLATION_PROBABILITY 0.15
#define OSCILLATION_MIN_DURATION 50
#define OSCILLATION_MAX_DURATION 150
#define OSCILLATION_AMPLITUDE 0.05

// Polynomial variance parameters
#define POLY_DEGREE 2
#define POLY_WINDOW_SIZE 10
#define VARIANCE_HISTORY 50

const int analogPins[NUM_CHANNELS] = { A0, A1, A2, A3 };

float channelHistory[NUM_CHANNELS][HISTORY_SIZE];
float smoothedData[NUM_CHANNELS][HISTORY_SIZE];
float baselineVoltages[NUM_CHANNELS];
float currentMatrix[MATRIX_SIZE][MATRIX_SIZE];
float seminormVector[MATRIX_SIZE];
float distanceMatrix[MATRIX_SIZE][MATRIX_SIZE]; 
float cauchySequence[MATRIX_SIZE];

// Polynomial variance tracking
float varianceHistory[NUM_CHANNELS][VARIANCE_HISTORY];
int varianceIndex = 0;
float currentVariance[NUM_CHANNELS];
float polyCoeffs[NUM_CHANNELS][POLY_DEGREE + 1];

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

#define TASKS NUM_CHANNELS
#define ROW_H 6
#define ROW_G 1
#define TOP_Y 10

// Dynamic chart state with polynomial variance
float ganttIntensity[TASKS];
unsigned long ganttLastUpdate[TASKS];
const unsigned long STATE_MIN_DURATION_MS = 100;

unsigned long lastFrame = 0;
const unsigned long frameMs = 50;

struct LogicState {
  bool t_sample;
  bool t_frame;
  bool p_converge;
  bool p_active_all3;
  bool p_optimal;
  bool p_oscStart;
  bool p_oscStop;
};

// Forward declarations
void establishBaseline();
void sampleAllChannels();
void applySmoothingFilter();
void updateCurrentMatrix();
void updateDistanceAndCauchyMetrics();
void calculatePolynomialVariance();
float calculateCorrelation(int ch1, int ch2);
float calculateDistanceMetric(int ch1, int ch2);
float getOscillationValue(int channel);
void drawVarianceChart();
bool solveLinearSystem(float A[][POLY_DEGREE + 1], float b[], float x[], int n);

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();

  for (int i = 0; i < MATRIX_SIZE; i++) {
    cauchySequence[i] = 0.0;
    seminormVector[i] = 0.0;
    oscillationFrequencies[i] = 0.0;
    oscillationPhases[i] = 0.0;
    currentVariance[i] = 0.0;
    ganttIntensity[i] = 0.0;
    for (int j = 0; j < MATRIX_SIZE; j++) {
      distanceMatrix[i][j] = 0.0;
    }
    for (int j = 0; j < VARIANCE_HISTORY; j++) {
      varianceHistory[i][j] = 0.0;
    }
  }
  
  for (int i = 0; i < TASKS; i++) {
    ganttLastUpdate[i] = 0;
  }

  randomSeed(analogRead(A0) + analogRead(A1));
  establishBaseline();
  Serial.println("Setup complete - Polynomial Time Variance Analysis");
}

LogicState evalLogic(unsigned long now) {
  LogicState s{};
  s.t_sample = (now - lastSampleTime) >= sampleIntervalMs;
  s.t_frame  = (now - lastFrame)      >= frameMs;

  bool haveHist = (analysisCount >= MATRIX_SIZE);
  bool anyFar = false;
  if (haveHist) {
    for (int i = 1; i < MATRIX_SIZE; i++) {
      if (fabs(cauchySequence[0] - cauchySequence[i]) > FRECHET_THRESHOLD) {
        anyFar = true; 
        break;
      }
    }
  }
  s.p_converge = haveHist && anyFar;

  bool a0 = seminormVector[0] > 100.0f;
  bool a1 = seminormVector[1] > 100.0f;
  bool a2 = seminormVector[2] > 100.0f;
  s.p_active_all3 = a0 && a1 && a2;
  s.p_optimal = s.p_converge && s.p_active_all3;

  s.p_oscStart = (!isOscillating) && (random(1000) < (OSCILLATION_PROBABILITY * 1000));
  s.p_oscStop  = (isOscillating)  && (oscillationCounter >= oscillationDuration);

  return s;
}

void loop() {
  unsigned long now = millis();
  LogicState s = evalLogic(now);

  if (s.t_sample) {
    lastSampleTime = now;

    if (s.p_oscStart) {
      isOscillating = true;
      oscillationCounter = 0;
      oscillationDuration = random(OSCILLATION_MIN_DURATION, OSCILLATION_MAX_DURATION);
      oscillationStartTime = now;
      for (int i = 0; i < NUM_CHANNELS; i++) {
        oscillationFrequencies[i] = random(20, 100) / 10.0;
        oscillationPhases[i]      = random(0, 628) / 100.0;
      }
    } else if (s.p_oscStop) {
      isOscillating = false;
    } else if (isOscillating) {
      oscillationCounter++;
    }

    sampleAllChannels();
    applySmoothingFilter();
    updateCurrentMatrix();
    updateDistanceAndCauchyMetrics();
    
    // Calculate polynomial variance for temporal analysis
    if (analysisCount >= POLY_WINDOW_SIZE) {
      calculatePolynomialVariance();
    }

    if (s.p_optimal) {
      Serial.println("YES");
    }

    // Update intensity based on polynomial variance
    for (int ch = 0; ch < TASKS; ch++) {
      bool activeCh = s.p_converge && (seminormVector[ch] > 100.0f);
      
      // Smooth transition using variance as weight
      float targetIntensity = activeCh ? (1.0 + currentVariance[ch] * 10.0) : 0.0;
      targetIntensity = constrain(targetIntensity, 0.0, 1.0);
      
      // Exponential smoothing
      ganttIntensity[ch] = ganttIntensity[ch] * 0.7 + targetIntensity * 0.3;
    }

    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    varianceIndex = (varianceIndex + 1) % VARIANCE_HISTORY;
    analysisCount++;
  }

  if (s.t_frame) {
    lastFrame = now;
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setCursor(xOffset, yOffset + 8);
      drawVarianceChart();
    } while (u8g2.nextPage());
  }
}

// ============ HELPER FUNCTIONS ============

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
    Serial.print("Channel "); 
    Serial.print(ch); 
    Serial.print(" baseline: "); 
    Serial.println(baselineVoltages[ch], 4);
  }
}

void sampleAllChannels() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    float voltage = (analogRead(analogPins[ch]) / 4095.0) * 3.3;
    voltage += getOscillationValue(ch);
    channelHistory[ch][historyIndex] = voltage;
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
  for (int i = 0; i < MATRIX_SIZE; i++) {
    float norm = 0.0;
    for (int j = 0; j < MATRIX_SIZE; j++) {
      norm += currentMatrix[i][j] * currentMatrix[i][j];
    }
    seminormVector[i] = sqrt(norm);
  }

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

// Gaussian elimination solver for polynomial fitting
bool solveLinearSystem(float A[][POLY_DEGREE + 1], float b[], float x[], int n) {
  float augmented[POLY_DEGREE + 1][POLY_DEGREE + 2];
  
  // Create augmented matrix
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      augmented[i][j] = A[i][j];
    }
    augmented[i][n] = b[i];
  }
  
  // Forward elimination
  for (int i = 0; i < n; i++) {
    // Find pivot
    int maxRow = i;
    for (int k = i + 1; k < n; k++) {
      if (fabs(augmented[k][i]) > fabs(augmented[maxRow][i])) {
        maxRow = k;
      }
    }
    
    // Swap rows
    for (int k = i; k <= n; k++) {
      float tmp = augmented[maxRow][k];
      augmented[maxRow][k] = augmented[i][k];
      augmented[i][k] = tmp;
    }
    
    // Check for singular matrix
    if (fabs(augmented[i][i]) < 1e-10) {
      return false;
    }
    
    // Eliminate column
    for (int k = i + 1; k < n; k++) {
      float factor = augmented[k][i] / augmented[i][i];
      for (int j = i; j <= n; j++) {
        augmented[k][j] -= factor * augmented[i][j];
      }
    }
  }
  
  // Back substitution
  for (int i = n - 1; i >= 0; i--) {
    x[i] = augmented[i][n];
    for (int j = i + 1; j < n; j++) {
      x[i] -= augmented[i][j] * x[j];
    }
    x[i] /= augmented[i][i];
  }
  
  return true;
}

void calculatePolynomialVariance() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    // Build design matrix for polynomial fit
    float A[POLY_DEGREE + 1][POLY_DEGREE + 1];
    float b[POLY_DEGREE + 1];
    
    // Initialize matrices
    for (int i = 0; i <= POLY_DEGREE; i++) {
      b[i] = 0.0;
      for (int j = 0; j <= POLY_DEGREE; j++) {
        A[i][j] = 0.0;
      }
    }
    
    // Collect data points from recent history
    for (int k = 0; k < POLY_WINDOW_SIZE; k++) {
      int idx = (historyIndex - k + HISTORY_SIZE) % HISTORY_SIZE;
      float t = (float)k / POLY_WINDOW_SIZE; // Normalized time
      float y = smoothedData[ch][idx] - baselineVoltages[ch];
      
      // Build normal equations: A^T * A * x = A^T * y
      for (int i = 0; i <= POLY_DEGREE; i++) {
        float ti = pow(t, i);
        b[i] += ti * y;
        for (int j = 0; j <= POLY_DEGREE; j++) {
          float tj = pow(t, j);
          A[i][j] += ti * tj;
        }
      }
    }
    
    // Solve for polynomial coefficients
    if (solveLinearSystem(A, b, polyCoeffs[ch], POLY_DEGREE + 1)) {
      // Calculate variance (mean squared error)
      float sumSquaredError = 0.0;
      for (int k = 0; k < POLY_WINDOW_SIZE; k++) {
        int idx = (historyIndex - k + HISTORY_SIZE) % HISTORY_SIZE;
        float t = (float)k / POLY_WINDOW_SIZE;
        float y = smoothedData[ch][idx] - baselineVoltages[ch];
        
        // Evaluate polynomial
        float yFit = 0.0;
        for (int i = 0; i <= POLY_DEGREE; i++) {
          yFit += polyCoeffs[ch][i] * pow(t, i);
        }
        
        float error = y - yFit;
        sumSquaredError += error * error;
      }
      
      currentVariance[ch] = sumSquaredError / POLY_WINDOW_SIZE;
      varianceHistory[ch][varianceIndex] = currentVariance[ch];
    }
  }
}

void drawVarianceChart() {
  // Draw bars with intensity based on polynomial variance
  for (int ch = 0; ch < TASKS; ch++) {
    int y = yOffset + TOP_Y + ch * (ROW_H + ROW_G);
    
    // Calculate bar width based on intensity (0.0 to 1.0)
    int barWidth = (int)(GRAPH_WIDTH * ganttIntensity[ch]);
    
    if (barWidth > 2) {
      // Draw filled bar proportional to variance intensity
      u8g2.drawBox(xOffset, y, barWidth, ROW_H);
      
      // Draw outline for full width
      u8g2.drawFrame(xOffset, y, GRAPH_WIDTH, ROW_H);
    } else {
      // Draw outline only when inactive
      u8g2.drawFrame(xOffset, y, GRAPH_WIDTH, ROW_H);
    }
    
    // Channel label
    u8g2.setFont(u8g2_font_micro_tr);
    char label[4];
    sprintf(label, "C%d", ch);
    u8g2.drawStr(xOffset - 12, y + ROW_H - 1, label);
    
    // Show variance value
    if (ganttIntensity[ch] > 0.1) {
      char varStr[6];
      int varInt = (int)(currentVariance[ch] * 1000);
      sprintf(varStr, "%d", varInt);
      u8g2.drawStr(xOffset + GRAPH_WIDTH + 2, y + ROW_H - 1, varStr);
    }
  }
}

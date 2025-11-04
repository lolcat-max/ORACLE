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

const int xOffset = 30;
const int yOffset = 12;

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

const int analogPins[NUM_CHANNELS] = { A0, A1, A2, A3 };

float channelHistory[NUM_CHANNELS][HISTORY_SIZE];
float smoothedData[NUM_CHANNELS][HISTORY_SIZE];
float baselineVoltages[NUM_CHANNELS];
float currentMatrix[MATRIX_SIZE][MATRIX_SIZE];
float seminormVector[MATRIX_SIZE];
float distanceMatrix[MATRIX_SIZE][MATRIX_SIZE]; 
float cauchySequence[MATRIX_SIZE];

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

uint8_t gantt[TASKS][GRAPH_WIDTH];
int head = 0;
unsigned long lastFrame = 0;
const unsigned long frameMs = 50;

// ============ STRUCT DEFINITION - ONLY ONE COPY ============
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
float calculateCorrelation(int ch1, int ch2);
float calculateDistanceMetric(int ch1, int ch2);
float getOscillationValue(int channel);
void drawGanttChart();

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();

  for (int i = 0; i < MATRIX_SIZE; i++) {
    cauchySequence[i] = 0.0;
    seminormVector[i] = 0.0;
    oscillationFrequencies[i] = 0.0;
    oscillationPhases[i] = 0.0;
    for (int j = 0; j < MATRIX_SIZE; j++) {
      distanceMatrix[i][j] = 0.0;
    }
  }
  for (int i = 0; i < TASKS; i++) {
    for (int j = 0; j < GRAPH_WIDTH; j++) {
      gantt[i][j] = 0;
    }
  }

  randomSeed(analogRead(A0) + analogRead(A1));
  establishBaseline();
  Serial.println("Setup complete - Using Translation-Invariant Complete Metric");
}

// ============ EVALLOGIC FUNCTION - ONLY ONE COPY ============
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

    if (s.p_optimal) {
      Serial.println("YES");
    }

    for (int ch = 0; ch < TASKS; ch++) {
      bool activeCh = s.p_converge && (seminormVector[ch] > 100.0f);
      gantt[ch][head] = activeCh ? 1 : 0;
    }
    head = (head + 1) % GRAPH_WIDTH;

    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
    analysisCount++;
  }

  if (s.t_frame) {
    lastFrame = now;
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.setCursor(xOffset, yOffset + 8);
      u8g2.print("Unity fuse");
      drawGanttChart();
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

void drawGanttChart() {
  for (int ch = 0; ch < TASKS; ch++) {
    int y = yOffset + TOP_Y + ch * (ROW_H + ROW_G);
    int runStart = -1;
    for (int x = 0; x < GRAPH_WIDTH; x++) {
      int col = (head + x) % GRAPH_WIDTH;
      if (gantt[ch][col] && runStart < 0) runStart = x;
      if ((!gantt[ch][col] || x == GRAPH_WIDTH - 1) && runStart >= 0) {
        int runEnd = gantt[ch][col] ? x : x - 1;
        int w = runEnd - runStart + 1;
        if (w > 0) u8g2.drawBox(xOffset + runStart, y, w, ROW_H);
        runStart = -1;
      }
    }
  }
}

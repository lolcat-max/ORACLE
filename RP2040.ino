#include <Arduino.h>
#include <math.h>

// Fréchet Space Analysis Applied to Arduino Analog Inputs
// Simplified version for RP2040 Feather A0-A3 analog pins [web:109][web:112]

// Mathematical constants
#define PI 3.14159265359

// RP2040 Feather analog configuration [web:109]
#define NUM_CHANNELS 4       // A0, A1, A2, A3 analog pins
#define MATRIX_SIZE 4        // 4x4 matrices for analysis
#define ADC_RESOLUTION 4095  // 12-bit ADC on RP2040 [web:115]
#define VREF 3.3            // 3.3V reference voltage [web:109]

// Fréchet space parameters [memory:98]
#define SAMPLE_RATE_MS 50   // 50Hz for curvature analysis
#define HISTORY_SIZE 20     // Time series buffer
#define SEMINORM_COUNT 6    // Family of seminorms
#define FRECHET_THRESHOLD 0.0003
#define SMOOTHING_WINDOW 5

// Analog pin definitions [web:112]
const int analogPins[NUM_CHANNELS] = {A0, A1, A2, A3};
const char* channelNames[NUM_CHANNELS] = {"A0", "A1", "A2", "A3"};

// Fréchet space mathematical structures
typedef struct {
    // Locally convex topology
    float convex_hull[NUM_CHANNELS][NUM_CHANNELS];
    float balanced_sets[NUM_CHANNELS];
    float absorbing_sets[NUM_CHANNELS];
    
    // Seminorm family for metric
    float seminorms[SEMINORM_COUNT][NUM_CHANNELS];
    float seminorm_weights[SEMINORM_COUNT];
    
    // Cauchy sequence analysis
    float cauchy_sequences[NUM_CHANNELS][HISTORY_SIZE];
    bool cauchy_convergent[NUM_CHANNELS];
    
    // Topological properties
    bool is_locally_convex;
    bool is_complete;
    bool is_metrizable;
    float metric_distance;
    
    // Dual space functionals
    float dual_functionals[NUM_CHANNELS][NUM_CHANNELS];
    float dual_norms[NUM_CHANNELS];
    
    // Convergence analysis
    float convergence_rates[NUM_CHANNELS];
    float stability_bounds[NUM_CHANNELS];
} frechet_space_t;

frechet_space_t frechet = {0};

// Analog signal data structures [memory:96]
float channelHistory[NUM_CHANNELS][HISTORY_SIZE];
float smoothedData[NUM_CHANNELS][HISTORY_SIZE];
float curvatureData[HISTORY_SIZE];
float baselineVoltages[NUM_CHANNELS];
float signalVariance[NUM_CHANNELS];

// Analysis matrices
float currentMatrix[MATRIX_SIZE][MATRIX_SIZE];
float curvatureMatrix[MATRIX_SIZE][MATRIX_SIZE];
float metricMatrix[MATRIX_SIZE][MATRIX_SIZE];

// State variables
int historyIndex = 0;
unsigned long lastSample = 0;
int selectedChannel = 0;
unsigned long analysisCount = 0;
float globalFrechetMetric = 0.0;
bool baselineEstablished = false;

// Function declarations
void setup();
void loop();
void establishAnalogBaseline();
void sampleAnalogChannels();
void applySmoothingFilter();
void performLocallyConvexAnalysis();
void calculateSeminormFamily();
void analyzeCauchySequences();
void checkTopologicalProperties();
void computeDualSpaceAnalysis();
void performConvergenceAnalysis();
void calculateFrechetMetric();
void performCurvatureAnalysis();
void detectSignalFlips();
void displayFrechetAnalysis();
void displayAnalogOscilloscope();
float calculateSeminorm(int seminorm_idx, float* vector);
void updateCauchySequence(int channel, float new_value);
bool testCauchyConvergence(int channel);
void handleSerialCommands();
void displayTopologyStatus();
void displayAnalogStatus();
void resetAnalysis();
float readAnalogVoltage(int pin);

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("╔════════════════════════════════════════════════════╗");
    Serial.println("║       FRÉCHET SPACE ANALOG SIGNAL ANALYZER         ║");
    Serial.println("║         RP2040 Feather A0-A3 Mathematical          ║");
    Serial.println("║            Topology Analysis System                ║");
    Serial.println("╠════════════════════════════════════════════════════╣");
    Serial.println("║ Connect signals to A0, A1, A2, A3 (0-3.3V)        ║");
    Serial.println("║ Commands:                                          ║");
    Serial.println("║   f - Full Fréchet Analysis                        ║");
    Serial.println("║   t - Topology Properties                          ║");
    Serial.println("║   c - Curvature Analysis                           ║");
    Serial.println("║   s - Signal Status                                ║");
    Serial.println("║   o - Oscilloscope Display                         ║");
    Serial.println("║   r - Reset Analysis                               ║");
    Serial.println("║   0-3 - Select Channel                             ║");
    Serial.println("╚════════════════════════════════════════════════════╝");
    
    // Initialize ADC [web:115]
    analogReadResolution(12);  // 12-bit resolution for RP2040
    
    initializeFrechetSpace();
    establishAnalogBaseline();
    
    Serial.println("■ Fréchet Space Analog Analysis Ready!");
}

void initializeFrechetSpace() {
    Serial.println("■ Initializing Fréchet Space for Analog Signals...");
    
    // Initialize locally convex topology
    for (int i = 0; i < NUM_CHANNELS; i++) {
        for (int j = 0; j < NUM_CHANNELS; j++) {
            frechet.convex_hull[i][j] = (i == j) ? 1.0f : 0.0f;
            frechet.dual_functionals[i][j] = 0.0f;
        }
        frechet.balanced_sets[i] = 0.0f;
        frechet.absorbing_sets[i] = 1.0f;
        frechet.dual_norms[i] = 0.0f;
        frechet.convergence_rates[i] = 0.0f;
        frechet.stability_bounds[i] = 1.0f;
        frechet.cauchy_convergent[i] = false;
        signalVariance[i] = 0.0f;
    }
    
    // Initialize seminorm family
    for (int i = 0; i < SEMINORM_COUNT; i++) {
        frechet.seminorm_weights[i] = 1.0f / (float)(i + 1);
        for (int j = 0; j < NUM_CHANNELS; j++) {
            frechet.seminorms[i][j] = 0.0f;
        }
    }
    
    // Initialize Cauchy sequences
    for (int i = 0; i < NUM_CHANNELS; i++) {
        for (int j = 0; j < HISTORY_SIZE; j++) {
            frechet.cauchy_sequences[i][j] = 0.0f;
            channelHistory[i][j] = 0.0f;
            smoothedData[i][j] = 0.0f;
        }
    }
    
    // Initialize topological properties
    frechet.is_locally_convex = false;
    frechet.is_complete = false;
    frechet.is_metrizable = true;
    frechet.metric_distance = 0.0f;
    
    Serial.println("■ Mathematical Framework Initialized");
}

void establishAnalogBaseline() {
    Serial.println("■ Establishing Analog Signal Baseline...");
    Serial.println("  Sampling all channels for 3 seconds...");
    
    // Sample baseline for 3 seconds
    const int baseline_samples = 60;  // 3 seconds at 50ms intervals
    float baseline_sum[NUM_CHANNELS] = {0};
    
    for (int sample = 0; sample < baseline_samples; sample++) {
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            float voltage = readAnalogVoltage(analogPins[ch]);
            baseline_sum[ch] += voltage;
        }
        
        // Progress indicator
        if (sample % 10 == 0) {
            Serial.printf("  Progress: %d%% ", (sample * 100) / baseline_samples);
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                Serial.printf("%s:%.3fV ", channelNames[ch], 
                             readAnalogVoltage(analogPins[ch]));
            }
            Serial.println();
        }
        
        delay(50);
    }
    
    // Calculate baseline averages
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        baselineVoltages[ch] = baseline_sum[ch] / baseline_samples;
    }
    
    baselineEstablished = true;
    
    Serial.println("■ Analog Baseline Established:");
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        Serial.printf("  %s baseline: %.4fV\n", 
                     channelNames[ch], baselineVoltages[ch]);
    }
}

float readAnalogVoltage(int pin) {
    // Read analog value and convert to voltage [web:109][web:115]
    int rawValue = analogRead(pin);
    return (rawValue / (float)ADC_RESOLUTION) * VREF;
}

void loop() {
    unsigned long currentTime = millis();
    
    handleSerialCommands();

        // Core analysis pipeline
        sampleAnalogChannels();
        applySmoothingFilter();
        performLocallyConvexAnalysis();
        calculateSeminormFamily();
        analyzeCauchySequences();
        checkTopologicalProperties();
        computeDualSpaceAnalysis();
        performConvergenceAnalysis();
        calculateFrechetMetric();
        performCurvatureAnalysis();
        
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
        analysisCount++;
        lastSample = currentTime;
        
        // Display periodic analysis
        if (analysisCount % 40 == 0) {  // Every 2 seconds
            displayAnalogOscilloscope();
        }
   
    
    delay(10);
}

void sampleAnalogChannels() {
    // Sample all analog channels simultaneously [web:105]
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        float voltage = readAnalogVoltage(analogPins[ch]);
        float deviation = voltage - baselineVoltages[ch];
        
        channelHistory[ch][historyIndex] = deviation;
        updateCauchySequence(ch, deviation);
        
        // Update signal variance for motion detection [memory:98]
        signalVariance[ch] = 0.9f * signalVariance[ch] + 
                           0.1f * (deviation * deviation);
    }
}

void applySmoothingFilter() {
    // Apply smoothing filter to reduce noise [web:106]
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        float sum = 0.0f;
        int count = 0;
        
        for (int i = 0; i < SMOOTHING_WINDOW && i < HISTORY_SIZE; i++) {
            int idx = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
            sum += channelHistory[ch][idx];
            count++;
        }
        
        if (count > 0) {
            smoothedData[ch][historyIndex] = sum / (float)count;
        }
    }
}

void performLocallyConvexAnalysis() {
    // Test local convexity property of the signal space
    float current_vector[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        current_vector[i] = smoothedData[i][historyIndex];
    }
    
    bool in_convex_hull = true;
    float convex_combination = 0.0f;
    
    for (int i = 0; i < NUM_CHANNELS; i++) {
        float local_convexity = 0.0f;
        int window_size = min(5, (int)analysisCount);
        
        if (window_size > 0) {
            for (int j = 0; j < window_size; j++) {
                int idx = (historyIndex - j + HISTORY_SIZE) % HISTORY_SIZE;
                local_convexity += smoothedData[i][idx] / (float)window_size;
            }
            
            float deviation = fabs(current_vector[i] - local_convexity);
            float denominator = 1.0f + fabs(local_convexity);
            frechet.convex_hull[i][i] = 1.0f - (deviation / denominator);
            
            if (deviation > 0.1f) in_convex_hull = false;  // Threshold for analog signals
            convex_combination += frechet.convex_hull[i][i];
        }
    }
    
    frechet.is_locally_convex = in_convex_hull && (convex_combination > 2.0f);
}

void calculateSeminormFamily() {
    // Calculate family of seminorms on the analog signal vector space
    float current_vector[NUM_CHANNELS];
    for (int i = 0; i < NUM_CHANNELS; i++) {
        current_vector[i] = smoothedData[i][historyIndex];
    }
    
    for (int p = 0; p < SEMINORM_COUNT; p++) {
        float seminorm_value = calculateSeminorm(p, current_vector);
        for (int i = 0; i < NUM_CHANNELS; i++) {
            frechet.seminorms[p][i] = seminorm_value * frechet.seminorm_weights[p];
        }
    }
}

float calculateSeminorm(int seminorm_idx, float* vector) {
    float result = 0.0f;
    
    switch (seminorm_idx) {
        case 0: // L1 seminorm
            for (int i = 0; i < NUM_CHANNELS; i++) {
                result += fabs(vector[i]);
            }
            break;
        case 1: // L2 seminorm
            for (int i = 0; i < NUM_CHANNELS; i++) {
                result += vector[i] * vector[i];
            }
            result = sqrt(result);
            break;
        case 2: // L∞ seminorm
            for (int i = 0; i < NUM_CHANNELS; i++) {
                float abs_val = fabs(vector[i]);
                if (abs_val > result) result = abs_val;
            }
            break;
        case 3: // Weighted seminorm
            for (int i = 0; i < NUM_CHANNELS; i++) {
                result += fabs(vector[i]) * (float)(i + 1);
            }
            break;
        case 4: // RMS-based seminorm [memory:98]
            for (int i = 0; i < NUM_CHANNELS; i++) {
                result += signalVariance[i];
            }
            result = sqrt(result);
            break;
        case 5: // Peak-to-peak seminorm
            float min_val = vector[0], max_val = vector[0];
            for (int i = 1; i < NUM_CHANNELS; i++) {
                if (vector[i] < min_val) min_val = vector[i];
                if (vector[i] > max_val) max_val = vector[i];
            }
            result = max_val - min_val;
            break;
    }
    
    return result;
}

void updateCauchySequence(int channel, float new_value) {
    // Update Cauchy sequence for convergence analysis
    for (int i = HISTORY_SIZE - 1; i > 0; i--) {
        frechet.cauchy_sequences[channel][i] = frechet.cauchy_sequences[channel][i-1];
    }
    frechet.cauchy_sequences[channel][0] = new_value;
}

void analyzeCauchySequences() {
    // Analyze Cauchy sequence convergence for completeness
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        frechet.cauchy_convergent[ch] = testCauchyConvergence(ch);
        
        // Calculate convergence rate
        if (analysisCount > 10) {
            float recent_variance = 0.0f;
            float older_variance = 0.0f;
            
            for (int i = 0; i < 5; i++) {
                float recent_val = frechet.cauchy_sequences[ch][i];
                recent_variance += recent_val * recent_val;
                
                if (i + 5 < HISTORY_SIZE) {
                    float older_val = frechet.cauchy_sequences[ch][i + 5];
                    older_variance += older_val * older_val;
                }
            }
            
            recent_variance /= 5.0f;
            older_variance /= 5.0f;
            
            if (older_variance > 0.0f) {
                frechet.convergence_rates[ch] = recent_variance / older_variance;
            }
        }
    }
}

bool testCauchyConvergence(int channel) {
    if (analysisCount < 15) return false;
    
    float max_distance = 0.0f;
    int test_points = min(8, HISTORY_SIZE / 2);
    
    for (int i = 0; i < test_points; i++) {
        for (int j = i + 1; j < test_points; j++) {
            float distance = fabs(frechet.cauchy_sequences[channel][i] - 
                                frechet.cauchy_sequences[channel][j]);
            if (distance > max_distance) max_distance = distance;
        }
    }
    
    return max_distance < FRECHET_THRESHOLD;
}

void checkTopologicalProperties() {
    // Check completeness and other topological properties
    int convergent_count = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (frechet.cauchy_convergent[i]) convergent_count++;
    }
    frechet.is_complete = (convergent_count >= NUM_CHANNELS - 1);
    
    // Calculate Fréchet metric distance
    float total_seminorm = 0.0f;
    for (int i = 0; i < SEMINORM_COUNT; i++) {
        for (int j = 0; j < NUM_CHANNELS; j++) {
            total_seminorm += frechet.seminorms[i][j];
        }
    }
    frechet.metric_distance = total_seminorm / (float)(SEMINORM_COUNT * NUM_CHANNELS);
}

void computeDualSpaceAnalysis() {
    // Analyze the dual space of linear functionals
    for (int i = 0; i < NUM_CHANNELS; i++) {
        float dual_norm = 0.0f;
        
        for (int j = 0; j < NUM_CHANNELS; j++) {
            float functional_value = 0.0f;
            
            for (int k = 0; k < NUM_CHANNELS; k++) {
                functional_value += frechet.convex_hull[j][k] * 
                                  smoothedData[k][historyIndex];
            }
            
            frechet.dual_functionals[i][j] = functional_value;
            dual_norm += functional_value * functional_value;
        }
        
        frechet.dual_norms[i] = sqrt(dual_norm);
    }
}

void performConvergenceAnalysis() {
    // Analyze convergence properties and stability bounds
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        if (analysisCount > 3) {
            float current = frechet.cauchy_sequences[ch][0];
            float previous = frechet.cauchy_sequences[ch][1];
            float stability = fabs(current - previous);
            
            frechet.stability_bounds[ch] = 0.9f * frechet.stability_bounds[ch] + 
                                         0.1f * stability;
        }
    }
}

void calculateFrechetMetric() {
    // Calculate the global Fréchet metric
    globalFrechetMetric = 0.0f;
    
    for (int p = 0; p < SEMINORM_COUNT; p++) {
        float weighted_seminorm = 0.0f;
        
        for (int i = 0; i < NUM_CHANNELS; i++) {
            weighted_seminorm += frechet.seminorms[p][i];
        }
        
        float denominator = 1.0f + weighted_seminorm;
        float metric_component = frechet.seminorm_weights[p] * 
                               weighted_seminorm / denominator;
        globalFrechetMetric += metric_component;
    }
}

void performCurvatureAnalysis() {
    // Analyze signal curvature using second derivatives [memory:98]
    if (analysisCount < 3) return;
    
    int curr = historyIndex;
    int prev = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    int prev2 = (historyIndex - 2 + HISTORY_SIZE) % HISTORY_SIZE;
    
    float currentVal = smoothedData[selectedChannel][curr];
    float prevVal = smoothedData[selectedChannel][prev];
    float prev2Val = smoothedData[selectedChannel][prev2];
    
    float secondDeriv = currentVal - 2.0f * prevVal + prev2Val;
    curvatureData[historyIndex] = fabs(secondDeriv) * 1000.0f;  // Scale for display
}

void displayAnalogOscilloscope() {
    // Display oscilloscope-style output [memory:101]
    Serial.println("\n╔════════════════════════════════════════════════════╗");
    Serial.println("║            ANALOG SIGNAL OSCILLOSCOPE              ║");
    Serial.println("╠════════════════════════════════════════════════════╣");
    
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        float current = readAnalogVoltage(analogPins[ch]);
        float deviation = current - baselineVoltages[ch];
        float rms = sqrt(signalVariance[ch]);
        
        Serial.printf(" %s: %.3fV │ Dev: %+.3fV │ RMS: %.3fV │ Conv: %-5s \n",
                     channelNames[ch], current, deviation, rms,
                     frechet.cauchy_convergent[ch] ? "YES" : "NO");
    }
    
    Serial.println("╠════════════════════════════════════════════════════╣");
    Serial.printf("║ Fréchet Metric: %.6f │ Locally Convex: %-5s   ║\n",
                  globalFrechetMetric, frechet.is_locally_convex ? "TRUE" : "FALSE");
    Serial.printf("  Complete: %-5s │ Metric Distance: %.6f\n",
                  frechet.is_complete ? "TRUE" : "FALSE", frechet.metric_distance);
    Serial.println("╚════════════════════════════════════════════════════╝");
}

void handleSerialCommands() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        switch(cmd) {
            case 'f':
                displayFrechetAnalysis();
                break;
            case 't':
                displayTopologyStatus();
                break;
            case 's':
                displayAnalogStatus();
                break;
            case 'o':
                displayAnalogOscilloscope();
                break;
            case 'r':
                resetAnalysis();
                break;
            case '0': case '1': case '2': case '3':
                selectedChannel = cmd - '0';
                Serial.printf("■ Selected Channel: %s\n", channelNames[selectedChannel]);
                break;
            case 'h':
                Serial.println("■ Commands: f=Fréchet, t=Topology, s=Status, o=Oscilloscope, r=Reset, 0-3=Channel");
                break;
        }
    }
}

void displayFrechetAnalysis() {
    Serial.println("\n╔════════════════════════════════════════════════════╗");
    Serial.println("║         FRÉCHET SPACE ANALYSIS RESULTS             ║");
    Serial.println("╠════════════════════════════════════════════════════╣");
    Serial.printf("║ Global Fréchet Metric:     %.6f              ║\n", globalFrechetMetric);
    Serial.printf("║ Locally Convex:            %-5s                ║\n", 
                  frechet.is_locally_convex ? "TRUE" : "FALSE");
    Serial.printf("║ Complete:                   %-5s                ║\n", 
                  frechet.is_complete ? "TRUE" : "FALSE");
    Serial.printf("║ Metric Distance:            %.6f              ║\n", frechet.metric_distance);
    Serial.printf("║ Analysis Count:             %-8lu           ║\n", analysisCount);
    Serial.println("╠════════════════════════════════════════════════════╣");
    Serial.println("║ Channel Convergence Analysis:                     ║");
    for (int i = 0; i < NUM_CHANNELS; i++) {
        Serial.printf("║ %s: Conv=%-5s Rate=%.4f Bound=%.4f Var=%.4f ║\n", 
                     channelNames[i],
                     frechet.cauchy_convergent[i] ? "YES" : "NO",
                     frechet.convergence_rates[i], 
                     frechet.stability_bounds[i],
                     signalVariance[i]);
    }
    Serial.println("╚════════════════════════════════════════════════════╝");
}

void displayTopologyStatus() {
    Serial.println("\n╔════════════════════════════════════════════════════╗");
    Serial.println("║            TOPOLOGICAL PROPERTIES                  ║");
    Serial.println("╠════════════════════════════════════════════════════╣");
    Serial.printf("║ Vector Space Dimension:    4 (A0-A3)              ║\n");
    Serial.printf("║ Locally Convex:            %-5s                ║\n", 
                  frechet.is_locally_convex ? "TRUE" : "FALSE");
    Serial.printf("║ Complete:                   %-5s                ║\n", 
                  frechet.is_complete ? "TRUE" : "FALSE");
    Serial.printf("║ Metrizable:                 %-5s                ║\n", 
                  frechet.is_metrizable ? "TRUE" : "FALSE");
    Serial.printf("║ Selected Channel:           %s                   ║\n", 
                  channelNames[selectedChannel]);
    Serial.println("╠════════════════════════════════════════════════════╣");
    Serial.println("║ Seminorm Values (Current):                        ║");
    for (int i = 0; i < min(4, SEMINORM_COUNT); i++) {
        Serial.printf("║ Seminorm %d: [", i);
        for (int j = 0; j < NUM_CHANNELS; j++) {
            Serial.printf("%.3f", frechet.seminorms[i][j]);
            if (j < NUM_CHANNELS-1) Serial.print(" ");
        }
        Serial.println("]                        ║");
    }
    Serial.println("╚════════════════════════════════════════════════════╝");
}

void displayAnalogStatus() {
    Serial.println("\n╔════════════════════════════════════════════════════╗");
    Serial.println("║              ANALOG INPUT STATUS                   ║");
    Serial.println("╠════════════════════════════════════════════════════╣");
    Serial.printf("║ ADC Resolution:             12-bit (%d levels)    ║\n", ADC_RESOLUTION + 1);
    Serial.printf("║ Reference Voltage:          %.1fV                 ║\n", VREF);
    Serial.printf("║ Sample Rate:                %d ms (%.1f Hz)       ║\n", 
                  SAMPLE_RATE_MS, 1000.0f / SAMPLE_RATE_MS);
    Serial.println("╠════════════════════════════════════════════════════╣");
    Serial.println("║ Current Readings:                                  ║");
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        float voltage = readAnalogVoltage(analogPins[ch]);
        int raw = analogRead(analogPins[ch]);
        Serial.printf("║ %s: %.4fV (Raw: %4d) Baseline: %.4fV       ║\n",
                     channelNames[ch], voltage, raw, baselineVoltages[ch]);
    }
    Serial.println("╚════════════════════════════════════════════════════╝");
}

void resetAnalysis() {
    Serial.println("■ Resetting Fréchet Space Analysis...");
    
    analysisCount = 0;
    historyIndex = 0;
    globalFrechetMetric = 0.0f;
    
    initializeFrechetSpace();
    establishAnalogBaseline();
    
    Serial.println("■ Analysis Reset Complete - Ready for New Data");
}

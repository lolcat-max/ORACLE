// Flip/event counter on Arduino Uno using Analog A0
// Works well with a diode+RC envelope detector feeding A0.

const int PIN = A0;

// Tuning knobs
const int SAMPLE_US = 200;        // 200 us -> ~5 kHz sampling
const float BASE_ALPHA = 0.002;   // baseline tracking speed (smaller = slower)
const int THRESH_UP = 25;         // rising threshold above baseline (ADC counts)
const int THRESH_DOWN = 10;       // falling threshold above baseline (ADC counts)

// State
unsigned long flipCount = 0;
bool armed = true;                // armed to detect next rising event
float baseline = 0;               // slow-moving baseline
int lastVal = 0;

void setup() {
  Serial.begin(115200);
  // Initialize baseline with a few readings
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    sum += analogRead(PIN);
    delay(2);
  }
  baseline = sum / 200.0;
  lastVal = (int)baseline;

  Serial.println("t_ms,adc,baseline,flipCount");
}

void loop() {
  int v = analogRead(PIN);

  // Update baseline slowly (so drift/noise is handled)
  baseline = (1.0 - BASE_ALPHA) * baseline + BASE_ALPHA * v;

  // Signal relative to baseline
  int dv = v - (int)baseline;

  // Hysteresis event detection
  if (armed) {
    if (dv > THRESH_UP) {
      flipCount++;
      armed = false; // wait until it falls back down
    }
  } else {
    if (dv < THRESH_DOWN) {
      armed = true;
    }
  }

  // Stream data (you can comment this out if you only want counts)
  Serial.print(millis());
  Serial.print(",");
  Serial.print(v);//signal
  Serial.print(",");
  Serial.print((int)baseline);
  Serial.print(",");
  Serial.println(flipCount);

  delayMicroseconds(SAMPLE_US);
}

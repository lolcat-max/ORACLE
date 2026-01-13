/*
  Fréchet space convergence demo (Arduino Serial Plotter)

  We model a Fréchet topology using a countable family of seminorms {p_k}.
  We only plot a few (p1, p2, p3) but compute a Fréchet-style metric from them:

    d(x,y) = Σ_{k=1..K} 2^{-k} * min(1, p_k(x-y))

  Sequence (converges to v when v is steady):
    x_{n+1} = (1-alpha)x_n + alpha v

  v is the "target vector" from analogRead(A0), analogRead(A1), normalized to [-1,1].

  Channels:
    X Y p1err p2err p3err dFrechet
*/

const unsigned long BAUD = 115200;

const int PIN_X = A0;
const int PIN_Y = A1;

const float alpha = 0.08f;   // contraction step (0<alpha<1), bigger = faster convergence

// Current iterate x_n in R^2
float x = 0.0f, y = 0.0f;

static inline float clamp01(float v) { return (v < 0.0f) ? 0.0f : (v > 1.0f ? 1.0f : v); }
static inline float absf(float a) { return (a < 0.0f) ? -a : a; }

void setup() {
  Serial.begin(BAUD);
  delay(200);
  Serial.println("X Y p1err p2err p3err dFrechet");
}

void loop() {
  // Read target v from analog and map to [-1, 1] so the origin is meaningful
  float vx = (analogRead(PIN_X) / 1023.0f) * 2.0f - 1.0f;
  float vy = (analogRead(PIN_Y) / 1023.0f) * 2.0f - 1.0f;

  // Contraction iteration toward v (gives convergence when v is stable)
  x = (1.0f - alpha) * x + alpha * vx;
  y = (1.0f - alpha) * y + alpha * vy;

  // Error e = x - v
  float ex = x - vx;
  float ey = y - vy;

  // A small family of seminorms p_k(e).
  // These are all seminorms on R^2 (nonnegative, subadditive, absolutely homogeneous).
  // p1: L1-type
  float p1 = absf(ex) + absf(ey);

  // p2: weighted L1 (different weights => different seminorm)
  float p2 = absf(ex) + 2.0f * absf(ey);

  // p3: max-type (L_infty)
  float p3 = (absf(ex) > absf(ey)) ? absf(ex) : absf(ey);

  // Fréchet-style metric using K=3 seminorms (finite truncation for Arduino)
  // d = sum_{k=1..3} 2^{-k} * min(1, p_k)
  float d = 0.0f;
  d += 0.5f  * clamp01(p1);  // 2^-1
  d += 0.25f * clamp01(p2);  // 2^-2
  d += 0.125f* clamp01(p3);  // 2^-3

  // Print channels (space-separated)
  Serial.print(x, 5);   Serial.print(' ');
  Serial.print(y, 5);   Serial.print(' ');
  Serial.print(p1, 5);  Serial.print(' ');
  Serial.print(p2, 5);  Serial.print(' ');
  Serial.print(p3, 5);  Serial.print(' ');
  Serial.println(d, 6);

  delay(25);
}

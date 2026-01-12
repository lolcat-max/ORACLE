/*
 * Knowing vs Walking - Quantum Automorphism Plotter
 * 
 * Visualizes the fundamental duality:
 * - Blue line: KNOWING (direct distance in Banach space)
 * - Red line: WALKING (actual path length through state space)
 * - Green line: SYMMETRY (how aligned they are)
 * 
 * Open Serial Plotter: Tools > Serial Plotter
 */

const int NUM_SENSORS = 6;
const int PATH_MEMORY = 30;

struct QuantumState {
  float pos[NUM_SENSORS];
  unsigned long time;
};

QuantumState path[PATH_MEMORY];
int pathIdx = 0;

QuantumState t1_know;    // The knowledge
QuantumState t2_walk;    // The walking
bool initialized = false;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Read sensors into quantum state
  readQuantumState(&t2_walk);
  
  // Initialize knowledge point
  if (!initialized) {
    t1_know = t2_walk;
    initialized = true;
  }
  
  // Store path
  path[pathIdx] = t2_walk;
  pathIdx = (pathIdx + 1) % PATH_MEMORY;
  
  // Compute metrics
  float knowing = computeKnowing();      // Direct distance t1→t2
  float walking = computeWalking();      // Path integral
  float symmetry = knowing / (walking + 0.001);  // Automorphism
  if (symmetry > 1.0) symmetry = 1.0;
  
  // Compute Banach L2 norm
  float norm = computeBanachNorm(t2_walk);
  
  // Plot: Knowing vs Walking vs Symmetry
  Serial.print("Knowing:");
  Serial.print(knowing * 2);  // Scale for visibility
  Serial.print(",Walking:");
  Serial.print(walking * 2);
  Serial.print(",Symmetry:");
  Serial.print(symmetry);
  Serial.print(",BanachNorm:");
  Serial.print(norm);
  
  // Add quantum phase
  float phase = computePhase();
  Serial.print(",Phase:");
  Serial.print(phase);
  
  Serial.println();
  
  // Reset knowledge every 5 seconds
  if (millis() % 5000 < 100) {
    t1_know = t2_walk;
  }
  
  delay(50);
}

void readQuantumState(QuantumState* state) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    state->pos[i] = (analogRead(A0 + i) - 511.5) / 511.5;
  }
  state->time = millis();
}

float computeKnowing() {
  // Knowing: straight-line Banach space distance
  float dist = 0.0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    float diff = t2_walk.pos[i] - t1_know.pos[i];
    dist += diff * diff;
  }
  return sqrt(dist);
}

float computeWalking() {
  // Walking: actual path length
  float totalDist = 0.0;
  
  for (int i = 1; i < PATH_MEMORY; i++) {
    int curr = (pathIdx - i + PATH_MEMORY) % PATH_MEMORY;
    int prev = (pathIdx - i - 1 + PATH_MEMORY) % PATH_MEMORY;
    
    if (path[prev].time > 0) {
      float segDist = 0.0;
      for (int j = 0; j < NUM_SENSORS; j++) {
        float diff = path[curr].pos[j] - path[prev].pos[j];
        segDist += diff * diff;
      }
      totalDist += sqrt(segDist);
    }
  }
  
  return totalDist;
}

float computeBanachNorm(QuantumState state) {
  // L2 norm in Banach space
  float sum = 0.0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sum += state.pos[i] * state.pos[i];
  }
  return sqrt(sum);
}

float computePhase() {
  // Berry phase - geometric memory of path
  static float phase = 0.0;
  
  int prev = (pathIdx - 1 + PATH_MEMORY) % PATH_MEMORY;
  
  if (path[prev].time > 0) {
    float connection = 0.0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      float dPos = t2_walk.pos[i] - path[prev].pos[i];
      connection += t2_walk.pos[i] * dPos;
    }
    phase += connection;
    
    while (phase > PI) phase -= 2*PI;
    while (phase < -PI) phase += 2*PI;
  }
  
  return phase;
}

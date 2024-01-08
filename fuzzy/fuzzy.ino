#include <math.h>
#include <algorithm>

#define FEEDBACK_COEFFICIENT_L 2.35
#define FEEDBACK_COEFFICIENT_R 2.35
#define MAX_MOTOR_SPEED 255

class Car {
  const int LMotorSpeedPin = 5;             //Left Motor Speed pin (ENA)
  const int LMotorForward = A0;            //Motor-L forward (IN1).
  const int LMotorBackward = A1;            //Motor-L backward (IN2)
  
  const int RMotorSpeedPin = 6;             //Right Motor Speed pin (ENB)
  const int RMotorForward = A3;             //Motor-R forward (IN4)
  const int RMotorBackward = A2;

  const int triggerPin = 11;
  const int echoPin = 12;

  static int ltick;
  static int rtick;

  double value = 0;
  int r_speed;
  int l_speed;

  static void incrementRight(){
    rtick++;
  }

  static void incrementLeft(){
    ltick++;
  }

  public: 
  void ultrasonicSetup() {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  void motorSetup() {
    pinMode(A0, OUTPUT);
    pinMode(LMotorBackward, OUTPUT);
    pinMode(RMotorForward, OUTPUT);
    pinMode(RMotorBackward, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(2), incrementLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), incrementRight, CHANGE);

    digitalWrite(LMotorForward, HIGH);  					
    digitalWrite(LMotorBackward, LOW);
    digitalWrite(RMotorForward, HIGH);   				  
    digitalWrite(RMotorBackward, LOW);
  }

  int readUltrasonicDistance() {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;
    return distance; // Unit: cm
  }

  void forward(int base_speed) {
    if (base_speed == 0) {
      r_speed = 0;
      l_speed = 0;
    } else {
      r_speed = base_speed + FEEDBACK_COEFFICIENT_R * (ltick - rtick);
      l_speed = base_speed - FEEDBACK_COEFFICIENT_L * (ltick - rtick);
    }

    analogWrite(LMotorSpeedPin, l_speed);                
    analogWrite(RMotorSpeedPin, r_speed);         
  }
};

int MIN_ALLOWED_DISTANCE = 70; // The robot has to stop 50 cm from the wall
// parametryczne opóźnienie symulujące różne warunki podłoża
int DELAY = 0; // Losowe opóźnienie od 0 do 1000 ms
double LITTLE_DELAY;
double MEDIUM_DELAY;
double LARGE_DELAY;

Car CAR;
int Car::ltick = 0;
int Car::rtick = 0;

void setup() {
  Serial.begin(9600);
  CAR.ultrasonicSetup();
  CAR.motorSetup();

  // Rozmycie odczytu opóźnienia
  getNormalizedDelays(DELAY, LITTLE_DELAY, MEDIUM_DELAY, LARGE_DELAY);
}

void loop() {
  // Odczytanie odległości od ściany
  int distance = CAR.readUltrasonicDistance(); // Unit = cm

  // Sprawdzenie, czy odległość nie przekracza maksymalnej dozwolonej wartości
  if (distance <= MIN_ALLOWED_DISTANCE) {
    Serial.println("Dystans zbyt mały, zatrzymanie robota.");
    CAR.forward(0); // Zatrzymaj robota
    return; // Zakończ pętlę loop
  }

  // Rozmycie odczytu odległości
  float close_distance = 0;
  float medium_distance = 0;
  float far_distance = 0;
  getNormalizedDistances(distance, close_distance, medium_distance, far_distance)

  // Obliczanie prędkości silnika z użyciem reguł rozmytych i metody środka ciężkości
  int motorSpeed = calculateMotorSpeed(close_distance, medium_distance, far_distance, little_delay, medium_distance, large_delay);

  // Ustawienie obliczonej prędkości silnika
  CAR.forward(motorSpeed);

  // Logowanie odległości i prędkości
  Serial.print("Dystans: ");
  Serial.print(distance);
  Serial.print(" cm, Prędkość silnika: ");
  Serial.println(motorSpeed);

  // parametryczne opóźnienie symulujące różne warunki podłoża
  delay(DELAY);
}
// Funkcje przynależności dla różnych poziomów opóźnienia
// Define the delay thresholds

// Define your thresholds
#define LITTLE_DELAY_THRESHOLD 150
#define LARGE_DELAY_THRESHOLD 850

// Define standard deviations
#define STD_DEV_SMALL 50
#define STD_DEV_MEDIUM 100
#define STD_DEV_LARGE 50

// Define means for Gaussian functions
#define MEAN_SMALL 100
#define MEAN_MEDIUM 500
#define MEAN_LARGE 900

// Helper function for Gaussian calculation
double gaussian(double x, double mean, double std_dev) {
    return exp(-pow(x - mean, 2) / (2 * pow(std_dev, 2)));
}

// Function for fuzzyLittleDelay (unnormalized)
double fuzzyLittleDelay(int delay) {
    if (delay <= LITTLE_DELAY_THRESHOLD) {
        return 1.0;
    } else {
        return gaussian(delay, MEAN_SMALL, STD_DEV_SMALL);
    }
}

// Function for fuzzyMediumDelay (unnormalized)
double fuzzyMediumDelay(int delay) {
    return gaussian(delay, MEAN_MEDIUM, STD_DEV_MEDIUM);
}

// Function for fuzzyLargeDelay (unnormalized)
double fuzzyLargeDelay(int delay) {
    if (delay >= LARGE_DELAY_THRESHOLD) {
        return 1.0;
    } else {
        return gaussian(delay, MEAN_LARGE, STD_DEV_LARGE);
    }
}

// Normalize and get the fuzzy values
void getNormalizedDelays(int delay, double* fuzzyLittle, double* fuzzyMedium, double* fuzzyLarge) {
    double sum;
    double little = fuzzyLittleDelay(delay);
    double medium = fuzzyMediumDelay(delay);
    double large = fuzzyLargeDelay(delay);

    sum = little + medium + large;

    // Avoid division by zero
    if (sum == 0) {
        *fuzzyLittle = 0;
        *fuzzyMedium = 0;
        *fuzzyLarge = 0;
    } else {
        *fuzzyLittle = little / sum;
        *fuzzyMedium = medium / sum;
        *fuzzyLarge = large / sum;
    }
}

// Funkcje przynależności dla różnych poziomów odległości
#define CLOSE_DISTANCE_THRESHOLD 80
#define MEDIUM_LOWER_DISTANCE_THRESHOLD 200
#define MEDIUM_UPPER_DISTANCE_THRESHOLD 500
#define FAR_DISTANCE_THRESHOLD 700

#define CLOSE_DISTANCE_COEFF -0.025 // a
#define FAR_DISTANCE_COEFF -0.005 // a
#define CLOSE_DISTANCE_CONST 1.25 // b
#define FAR_DISTANCE_CONST 3.5 // b


double fuzzyCloseDistance(int distance) {
  if (distance <= CLOSE_DISTANCE_THRESHOLD) return 1; // Pełna przynależność, gdy robot jest bardzo blisko ściany
  if (distance > CLOSE_DISTANCE_THRESHOLD && distance < MEDIUM_LOWER_DISTANCE_THRESHOLD) return static_cast<double>(distance) * (CLOSE_DISTANCE_COEFF) + 1.25; // Linear interpolation
  return 0;
}

double fuzzyMediumDistance(int distance) {
  max(min((distance - CLOSE_DISTANCE_THRESHOLD)/(MEDIUM_LOWER_DISTANCE_THRESHOLD-CLOSE_DISTANCE_THRESHOLD),1,(FAR_DISTANCE_THRESHOLD-distance)/(FAR_DISTANCE_THRESHOLD-MEDIUM_UPPER_DISTANCE_THRESHOLD),0)
  return 0;
}

double fuzzyFarDistance(int distance) {
  if (distance >= MEDIUM_UPPER_DISTANCE_THRESHOLD && distance < FAR_DISTANCE_THRESHOLD) return static_cast<double>(distance) * (FAR_DISTANCE_COEFF) +  FAR_DISTANCE_CONST// Linear interpolation from "medium" to "far"
  if (distance >= FAR_DISTANCE_THRESHOLD) return 1; // Pełna przynależność, gdy robot jest daleko
  return 0;
}

void getNormalizedDistances(int distance, double* fuzzyClose, double* fuzzyMedium, double* fuzzyFar) {
    double close = fuzzyCloseDistance(distance);
    double medium = fuzzyMediumDistance(distance);
    double far = fuzzyFarDistance(distance);

    double sum = close + medium + far;

    // Avoid division by zero
    if (sum == 0) {
        *fuzzyClose = 0;
        *fuzzyMedium = 0;
        *fuzzyFar = 0;
    } else {
        *fuzzyClose = close / sum;
        *fuzzyMedium = medium / sum;
        *fuzzyFar = far / sum;
    }
}

// Funkcje przynależności dla różnych poziomów prędkości
double fuzzyVerySlowArea(double x) {
  if (x < 0.25) return (x * x) / (0.25 * 2);
  else return 0.25 * 0.5;
}

double fuzzySlowArea(double x) {
  if (x <= 0) return 0;
  else if (x < 0.25) return (x * x) / (0.25 * 2);
  else if (x < 0.5) return ((0.25 * 0.5) + (x - 0.25) * (x + 0.25) / (0.5 * 2));
  else return (0.25 * 0.5) + (0.25 * 0.25 / 2);
}

double fuzzyMediumArea(double x) {
  if (x <= 0.25) return 0;
  else if (x < 0.5) return (x - 0.25) * (x + 0.25) / (0.5 * 2);
  else if (x < 0.75) return (0.25 * 0.25 / 2) + ((x - 0.5) * (x + 0.5) / (0.5 * 2));
  else return (0.25 * 0.25 / 2) + (0.25 * 0.25 / 2);
}

double fuzzyFastArea(double x) {
  if (x <= 0.5) return 0;
  else if (x < 0.75) return (x - 0.5) * (x + 0.5) / (0.5 * 2);
  else if (x < 1) return (0.25 * 0.25 / 2) + ((0.75 - x) * (1.75 - x) / (0.5 * 2));
  else return 0.25 * 0.25 / 2;
}

double fuzzyVeryFastArea(double x) {
  if (x <= 0.75) return 0;
  else if (x < 1) return ((0.75 - x) * (1.75 - x) / (0.5 * 2));
  else return 1 * 0.25 / 2; // Full area if x is at maximum
}

// Helper function to calculate the centroid of a trapezoid given the heights and widths
double calculateCentroid(double baseStart, double baseEnd, double height) {
    // Centroid formula for a trapezoid (average of the bases)
    return (baseStart + baseEnd) / 2.0 * height;
}

// Function to calculate the weighted centroid (center of gravity)
double calculateWeightedCentroid(double areaVerySlow, double areaSlow, double areaMedium, double areaFast, double areaVeryFast) {
    // Calculate the centroids for each trapezoidal section assuming linear distribution
    double centroidVerySlow = calculateCentroid(0.0, 0.25, areaVerySlow);
    double centroidSlow = calculateCentroid(0.25, 0.5, areaSlow);
    double centroidMedium = calculateCentroid(0.5, 0.75, areaMedium);
    double centroidFast = calculateCentroid(0.75, 1.0, areaFast);
    double centroidVeryFast = calculateCentroid(0.8, 1.0, areaVeryFast);

    // Compute the weighted sum of the centroids
    double numerator = (centroidVerySlow * areaVerySlow) + (centroidSlow * areaSlow) + (centroidMedium * areaMedium) + (centroidFast * areaFast) + (centroidVeryFast * areaVeryFast);
    double denominator = areaVerySlow + areaSlow + areaMedium + areaFast + areaVeryFast;

    // Ensure denominator is not zero to avoid division by zero
    if (denominator == 0) return 0;

    // Calculate the weighted centroid
    double weightedCentroid = numerator / denominator;

    return weightedCentroid;
}

// Funkcja obliczająca prędkość silnika na podstawie reguł rozmytych i metody środka ciężkości
int calculateMotorSpeed(double close_distance, double medium_distance, double far_distance, double little_delay, double medium_delay, double large_delay) {
  if (close_distance == 1) return 0;

  // Jeśli odległość jest "bliska" i opóźnienie "duże", robot ma zatrzymać się lub poruszać się bardzo wolno.  
  if (close_distance > 0 && large_delay > 0) {
    double verySlow = fuzzyVerySlowArea(min(close_distance, large_delay)); // wynikanie mandamiego
  }
  // Jeśli odległość jest "bliska" i opóźnienie "małe", robot może poruszać się wolno, ponieważ odległość jest priorytetem
  if (close_distance > 0 && little_delay > 0) {
    double slow = fuzzySlowArea(min(close_distance, little_delay));
  }
  // Jeśli opóźnienie jest "duże", robot powinien poruszać się z umiarkowaną prędkością
  if (large_delay > 0) {
    double medium = fuzzyMedium(large_delay);
  }
  // Jeśli odległość jest "średnia" i opóźnienie "małe", robot może poruszać się szybko
  if (medium_distance > 0 && little_delay > 0) {
    double fast = fuzzyFastArea(min(medium_distance, little_delay));
  }
  // Jeśli odległość jest "daleka" i opóźnienie "małe", robot może poruszać się bardzo szybko
  if (large_distance && little_delay) {
    double veryFast = fuzzyVeryFastArea(min(large_distance, little_delay));
  }

  // Calculate the weighted centroid for the combined areas
  double weightedCentroid = calculateWeightedCentroid(verySlow, slow, medium, fast, veryFast);
  // Convert the centroid to a motor speed, scaling up to the motor's speed range
  int motorSpeed = static_cast<int>(weightedCentroid * MAX_MOTOR_SPEED);

  const int speedVerySlow = 50;
  const int speedSlow = 100; // Niska prędkość dla bliskiej odległości
  const int speedMedium = 180; // Średnia prędkość dla średniej odległości
  const int speedFast = 217;
  const int speedFast = 255; // Maksymalna prędkość dla dalekiej odległości


  // Obliczanie środka ciężkości
  double numerator = (verySlow * speedVerySlow) + (slow * speedSlow) + (medium * speedMedium) + (fast * speedFast) + (veryFast * speedVeryFast);
  double denominator = verySlow + slow + medium + fast + veryFast;

  // Zabezpieczenie przed dzieleniem przez zero
  if (denominator == 0) return 0;

  // Obliczenie wynikowej prędkości
  int resultSpeed = static_cast<int>(numerator / denominator);

  return resultSpeed;
}

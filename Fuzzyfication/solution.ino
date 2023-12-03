#define FEEDBACK_COEFFICIENT_L 3
#define FEEDBACK_COEFFICIENT_R 3

class Car {
  const int LMotorSpeedPin = 5;             //Left Motor Speed pin (ENA)
  const int LMotorForward = A0;             //Motor-L forward (IN1).
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
    pinMode(LMotorForward, OUTPUT);
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

int MIN_ALLOWED_DISTANCE = 50; // The robot has to stop 50 cm from the wall
// parametryczne opóźnienie symulujące różne warunki podłoża
int DELAY = random(0, 1000); // Losowe opóźnienie od 0 do 1000 ms

Car CAR;
int Car::ltick = 0;
int Car::rtick = 0;

void setup() {
  Serial.begin(9600);
  CAR.ultrasonicSetup();
  CAR.motorSetup();
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
  float close = fuzzyClose(distance);
  float medium = fuzzyMedium(distance);
  float far = fuzzyFar(distance);

  // Obliczanie prędkości silnika z użyciem reguł rozmytych i metody środka ciężkości
  int motorSpeed = calculateMotorSpeed(close, medium, far);

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

// Funkcje przynależności dla różnych poziomów odległości
float fuzzyClose(int distance) {
  if (distance <= 80) return 1; // Pełna przynależność, gdy robot jest bardzo blisko ściany
  if (distance > 80 && distance < 200) return (200 - distance) / 120.0; // Linear interpolation
  return 0;
}

// Funkcje przynależności dla różnych poziomów odległości
float fuzzyMedium(int distance) {
  if (distance > 80 && distance <= 200) return (distance - 80) / 120.0; // Linear interpolation from "close" to "medium"
  if (distance > 200 && distance < 500) return 1; // Pełna przynależność w średnim zakresie
  if (distance >= 500 && distance < 700) return (700 - distance) / 200.0; // Linear interpolation from "medium" to "far"
  return 0;
}

float fuzzyFar(int distance) {
  if (distance >= 500 && distance < 700) return (distance - 500) / 200.0; // Linear interpolation from "medium" to "far"
  if (distance >= 700) return 1; // Pełna przynależność, gdy robot jest daleko
  return 0;
}

// Funkcja obliczająca prędkość silnika na podstawie reguł rozmytych i metody środka ciężkości
int calculateMotorSpeed(float close, float medium, float far) {
  // Zakresy prędkości dla każdego poziomu odległości
  const int speedClose = 100; // Niska prędkość dla bliskiej odległości
  const int speedMedium = 180; // Średnia prędkość dla średniej odległości
  const int speedFar = 255; // Maksymalna prędkość dla dalekiej odległości

  // Obliczanie środka ciężkości
  float numerator = (close * speedClose) + (medium * speedMedium) + (far * speedFar);
  float denominator = close + medium + far;

  // Zabezpieczenie przed dzieleniem przez zero
  if (denominator == 0) return 0;

  // Obliczenie wynikowej prędkości
  int resultSpeed = static_cast<int>(numerator / denominator);

  return resultSpeed;
}

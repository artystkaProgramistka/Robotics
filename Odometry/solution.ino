#define FEEDBACK_COEFFICIENT_L 3
#define FEEDBACK_COEFFICIENT_R 3
#define BASE_SPEED 150

class Car {
  const double DIAMETER = 0.065 * PI;

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

  int r_speed;
  int l_speed;

  static void incrementRight(){
    rtick++;
  }

  static void incrementLeft(){
    ltick++;
  }

  void motorForwardSetup() {
    digitalWrite(LMotorForward, HIGH);
    digitalWrite(RMotorForward, HIGH);
    digitalWrite(LMotorBackward, LOW);
    digitalWrite(RMotorBackward, LOW);
  }

  void motorBackwardSetup() {
    digitalWrite(LMotorForward, LOW);
    digitalWrite(RMotorForward, LOW);
    digitalWrite(LMotorBackward, HIGH);
    digitalWrite(RMotorBackward, HIGH);
  }

  void motorRightSetup() {
    digitalWrite(LMotorForward, HIGH);
    digitalWrite(RMotorForward, LOW);
    digitalWrite(LMotorBackward, LOW);
    digitalWrite(RMotorBackward, LOW);
  }

  void motorLeftSetup() {
    digitalWrite(LMotorForward, LOW);
    digitalWrite(RMotorForward, HIGH);
    digitalWrite(LMotorBackward, LOW);
    digitalWrite(RMotorBackward, LOW);
  }

  void stop() {
    digitalWrite(LMotorForward, LOW);
    digitalWrite(LMotorBackward, LOW);
    digitalWrite(RMotorBackward, LOW);
    digitalWrite(RMotorForward, LOW);
    rtick = 0;
    ltick = 0;
  }

  public: 
  void motorSetup() {
    pinMode(LMotorForward, OUTPUT);
    pinMode(RMotorForward, OUTPUT);
    pinMode(LMotorBackward, OUTPUT);
    pinMode(RMotorBackward, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(2), incrementLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), incrementRight, CHANGE);
  }

  void forward(double distance) {
    this->motorForwardSetup();
    while(((distance / DIAMETER * 42) >= rtick) || ((distance / DIAMETER * 42) >= ltick)){
      r_speed = BASE_SPEED + FEEDBACK_COEFFICIENT_R*(ltick - rtick);
      l_speed = BASE_SPEED - FEEDBACK_COEFFICIENT_L*(ltick - rtick);

      // Ensure speed values are within PWM range
      r_speed = constrain(r_speed, 100, 255);
      l_speed = constrain(l_speed, 100, 255);

      analogWrite(LMotorSpeedPin, l_speed);
      analogWrite(RMotorSpeedPin, l_speed);
    }
    this->stop();        
  }

  void backward(double distance){
    this->motorBackwardSetup();
    while(((distance / DIAMETER * 42) >= rtick) || ((distance / DIAMETER * 42) >= ltick)){
      r_speed = BASE_SPEED + 3*(ltick - rtick);
      l_speed = BASE_SPEED - 3*(ltick - rtick);
      if (r_speed < 100) {
        r_speed = 100;
      }
      if (r_speed > 255){
        r_speed = 255;
      }
      if (l_speed < 100) {
        l_speed = 100;
      }
      if (l_speed > 255){
        l_speed = 255;
      }
      analogWrite(LMotorSpeedPin, l_speed);
      analogWrite(RMotorSpeedPin, r_speed);
    }
    this->stop();
  }

  void right(double angle){
    this->motorRightSetup();
    while((angle / 360 * 0.26 * PI / DIAMETER * 42) >= ltick){
      analogWrite(LMotorSpeedPin, 130);
    }
    this->stop();
  }

  void left(double angle){
    this->motorLeftSetup();
    while((angle / 360 * 0.26 * PI / DIAMETER * 42) >= rtick){
      analogWrite(RMotorSpeedPin, 130);
    }
    this->stop();
  }
};

Car CAR;
int Car::ltick = 0;
int Car::rtick = 0;

enum CommandType { FORWARD, BACKWARD, RIGHT, LEFT, INVALID };

struct Command {
  CommandType type;
  double value;
};

CommandType parseCommandType(const String& cmd) {
  if (cmd == "F") return FORWARD;
  if (cmd == "B") return BACKWARD;
  if (cmd == "R") return RIGHT;
  if (cmd == "L") return LEFT;
  return INVALID;
}

Command parseCommand(const String& rawCommand) {
  Command command;
  command.type = parseCommandType(rawCommand.substring(0, 1));
  command.value = rawCommand.substring(1).toDouble();
  return command;
}

void executeCommand(const Command& command) {
  switch (command.type) {
    case FORWARD:
      CAR.forward(command.value);
      Serial.print("forward ");
      break;
    case BACKWARD:
      CAR.backward(command.value);
      Serial.print("backward ");
      break;
    case RIGHT:
      CAR.right(command.value);
      Serial.print("right ");
      break;
    case LEFT:
      CAR.left(command.value);
      Serial.print("left ");
      break;
    default:
      Serial.print("Invalid command ");
  }
  Serial.println(command.value);
}

void setup() {
  CAR.motorSetup();
  Serial.begin(9600);
}

void loop() {
  Serial.println("Enter command:");
  while (Serial.available() == 0) {}

  String commandInput = Serial.readString();
  commandInput.trim();
  
  int commandIndex = 0;
  String commands[7];

  while (commandInput.length() > 0) {
    int spaceIndex = commandInput.indexOf(' ');
    if (spaceIndex == -1) {
      commands[commandIndex++] = commandInput;
      break;
    } else {
      commands[commandIndex++] = commandInput.substring(0, spaceIndex);
      commandInput = commandInput.substring(spaceIndex + 1);
    }
  }

  for (int i = 0; i < commandIndex; i++) {
    Command command = parseCommand(commands[i]);
    executeCommand(command);
  }
}

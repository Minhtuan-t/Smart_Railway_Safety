#define DO_PIN 2  
// Arduino's pin connected to DO pin of the flame sensor

void setup() {
  Serial.begin(9600);
  pinMode(DO_PIN, INPUT);
}

void loop() {
  int flame_state = digitalRead(DO_PIN);

  if (flame_state == HIGH)
    Serial.println("The flame is NOT present => The fire is NOT detected");
  else
    Serial.println("The flame is present => The fire is detected");
}

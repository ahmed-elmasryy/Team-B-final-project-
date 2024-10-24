#include <WiFi.h>
#include <PubSubClient.h> 
//Connects to WiFi and sets up the MQTT client, subscribing to the "robot/move" topic for receiving movement commands

// WiFi credentials
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";
const char* mqttServer = "mqtt_broker_address";
const int mqttPort = 1883;
const char* mqttUser = "your_mqtt_username";
const char* mqttPassword = "your_mqtt_password";

// Motor pins
const int motor1Pin1 = 15; // Motor 1 direction pin 1
const int motor1Pin2 = 16; // Motor 1 direction pin 2
const int motorSpeed1 = 19; // Motor 1 speed pin (PWM)
const int motor2Pin1 = 17; // Motor 2 direction pin 1
const int motor2Pin2 = 18; // Motor 2 direction pin 2
const int motorSpeed2 = 20; // Motor 2 speed pin (PWM)

// Encoder pins
const int encoder1PinA = 34; // Encoder 1 A pin
const int encoder1PinB = 35; // Encoder 1 B pin
const int encoder2PinA = 32; // Encoder 2 A pin
const int encoder2PinB = 33; // Encoder 2 B pin

// MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// Variables for motor speed and encoder counts
int currentSpeed1 = 0;
int currentSpeed2 = 0;
volatile int encoder1Count = 0;
volatile int encoder2Count = 0;
const int maxSpeed = 255; // Maximum PWM value

// Setup
void setup() {
    Serial.begin(115200);
    
    // Motor pins
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motorSpeed1, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(motorSpeed2, OUTPUT);
    
    // Encoder pins
    pinMode(encoder1PinA, INPUT_PULLUP);
    pinMode(encoder1PinB, INPUT_PULLUP);
    pinMode(encoder2PinA, INPUT_PULLUP);
    pinMode(encoder2PinB, INPUT_PULLUP);
    
    attachInterrupt(encoder1PinA, updateEncoder1, CHANGE);  //Interrupts are attached to encoder A pins to detect changes and update the encoder counts
    attachInterrupt(encoder2PinA, updateEncoder2, CHANGE);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Setup MQTT
    client.setServer(mqttServer, mqttPort);
    client.setCallback(mqttCallback);
    reconnect();
}

void loop() {  
  //checks the MQTT connection and publishes the motor speeds to the "robot/speeds" topic based on the encoder readings
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Publish encoder data as speed
    publishMotorSpeeds();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0'; // Null-terminate the string
    String command = String((char*)payload);

    if (command == "FORWARD") {
        moveForward();
    } else if (command == "BACKWARD") {
        moveBackward();
    } else if (command == "LEFT") {
        turnLeft();
    } else if (command == "RIGHT") {
        turnRight();
    } else if (command == "STOP") {
        stopMotors();
    }
}

// Gradually increase or decrease motor speed
void setMotorSpeed(int speed1, int speed2) {
    if (speed1 < 0) {
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
    } else {
        digitalWrite(motor1Pin1, HIGH);
        digitalWrite(motor1Pin2, LOW);
    }

    if (speed2 < 0) {
        digitalWrite(motor2Pin1, LOW);
        digitalWrite(motor2Pin2, HIGH);
    } else {
        digitalWrite(motor2Pin1, HIGH);
        digitalWrite(motor2Pin2, LOW);
    }

    // Gradually adjust speed
    for (int i = 0; i <= maxSpeed; i++) {
        analogWrite(motorSpeed1, abs(speed1) > i ? i : 0);
        analogWrite(motorSpeed2, abs(speed2) > i ? i : 0);
        delay(10); // Adjust this delay for faster/slower acceleration
    }

    // Update current speeds
    currentSpeed1 = speed1;
    currentSpeed2 = speed2;
}

// Movement functions
void moveForward() {
    setMotorSpeed(maxSpeed, maxSpeed);
}

void moveBackward() {
    setMotorSpeed(-maxSpeed, -maxSpeed);
}

void turnLeft() {
    setMotorSpeed(-maxSpeed, maxSpeed); // Motor 1 backward, Motor 2 forward
}

void turnRight() {
    setMotorSpeed(maxSpeed, -maxSpeed); // Motor 1 forward, Motor 2 backward
}

void stopMotors() {
    setMotorSpeed(0, 0);
}

// Publish current motor speeds
void publishMotorSpeeds() {
    String speedData = String(currentSpeed1) + "," + String(currentSpeed2);
    client.publish("robot/speeds", speedData.c_str());
}

// Encoder update functions
void updateEncoder1() {
    if (digitalRead(encoder1PinB) == HIGH) {
        encoder1Count++;
    } else {
        encoder1Count--;
    }
}

void updateEncoder2() {
    if (digitalRead(encoder2PinB) == HIGH) {
        encoder2Count++;
    } else {
        encoder2Count--;
    }
}

// Reconnect to MQTT broker
void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
            Serial.println("connected");
            client.subscribe("robot/move"); // Subscribe to the movement topic "robot/move"
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

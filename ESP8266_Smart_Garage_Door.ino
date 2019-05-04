#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <TFMini.h>

#define WiFiSSID "**********" //Enter your own WiFi SSID.
#define WiFiPasswd "********" //Enter password for the SSID above.
#define mqtt_server "10.0.0.138" //MQTT server IP address.
#define calibrationButton 4
#define graceDistance 12
#define ledPin 0
#define openClosePin 16

uint16_t openDistance = 0;
uint16_t closedDistance = 0;
String previousState = "CLOSED";
bool isDisabled = false;

WiFiClient espClient;
PubSubClient client(espClient);
SoftwareSerial ss(2,14);
TFMini distanceSensor;

//This is the ISR that's triggered by the timer.
void inline ledISR(void);

//Function that gets the distance reading from the distance sensor.
uint16_t inline getReading();

void setup() {
  Serial.begin(115200);
  Serial.println();

  ss.begin(TFMINI_BAUDRATE);
  distanceSensor.begin(&ss);

  pinMode(calibrationButton, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(openClosePin, OUTPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(openClosePin, LOW);

  //Initialize timer that triggers the ISR
  timer1_isr_init();
  timer1_attachInterrupt(ledISR);
  timer1_write(1000000);
    
  WiFi.mode(WIFI_STA);
  WiFi.begin(WiFiSSID, WiFiPasswd);
  
  Serial.print(F("Connecting"));

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print(F("\nConnected, IP address: "));
  Serial.println(WiFi.localIP());

  //Wait for calibration button
  Serial.println(F("\nWaiting for calibration button...."));
  delay(2000);
  if (digitalRead(calibrationButton) == LOW) {
    calibrate();
  }

  //Load calibration values from EEPROM
  EEPROM.begin(8);
  EEPROM.get(0,openDistance);
  EEPROM.get(4,closedDistance);
  EEPROM.end();

  Serial.print("\nCalibration values: ");
  Serial.print(openDistance);
  Serial.print(", ");
  Serial.println(closedDistance);
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  client.subscribe("GarageRequest");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("\nMessage arrived ["));
  Serial.print(topic);
  Serial.print(F("] "));

  String dataString = "";

  for (unsigned int i = 0; i < length; i++) {
    dataString += (char)payload[i];
  }
  
  Serial.println(dataString);

  if (dataString == "state") {
    if (isDisabled) {
      Serial.println(F("\nSTATUS UPDATES DISABLED!"));
      client.publish("GarageState", previousState.c_str());
      return;
    }
    Serial.println(F("Received DATA"));
    String clientId = "MainGarageDoor-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      /* Get current distance reading
       * Dividing by 25.4 both averages the 10 readings
       * and converts the reading to inches simultaneously.
       */
      float reading = (float)(getReading()) / 25.4;
      
      Serial.print(reading);

      //Evaluate garage door state
      if (reading >= (float)(closedDistance - graceDistance)) {
        client.publish("GarageState", "CLOSED");
        previousState = "CLOSED";
      }
      else if (reading <= (float)(openDistance + graceDistance)) {
        client.publish("GarageState", "OPEN");
        previousState = "OPEN";
      }
      else {
        client.publish("GarageState", previousState.c_str());
      }
    }
  }
  else if (dataString == "open") {
    String clientId = "MainGarageDoor-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      /* Get current distance reading
       * Dividing by 25.4 both averages the 10 readings
       * and converts the reading to inches simultaneously.
       */      
      float reading = (float)(getReading()) / 25.4;
  
      if (reading <= (float)(openDistance + graceDistance)) {
        //Door is already open, so just publish "OPEN"
        client.publish("GarageState", "OPEN");
        previousState = "OPEN";
      }
      else {
        /* Door should be closed.
         * Emulate pressing the button to open door. 
         */
        digitalWrite(openClosePin, HIGH);
        delay(200);
        digitalWrite(openClosePin, LOW);
      
        /* Dividing by 25.4 both averages the 10 readings
         * and converts the reading to inches simultaneously.
         */ 
        float firstReading = (float)(getReading()) / 25.4;
    
        delay(2000);
    
        /* Dividing by 25.4 both averages the 10 readings
         * and converts the reading to inches simultaneously.
         */        
        float secondReading = (float)(getReading()) / 25.4;
    
        if (abs(firstReading - secondReading) <= 1) {
          /* Emulate pressing button again because the door
           * probably didn't move. 
           */
          digitalWrite(openClosePin, HIGH);
          delay(200);
          digitalWrite(openClosePin, LOW);
        } 
      } 
    }
  }
  else if (dataString == "close") {
    String clientId = "MainGarageDoor-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      /* Get current distance reading
       * Dividing by 25.4 both averages the 10 readings
       * and converts the reading to inches simultaneously.
       */
      float reading = (float)(getReading()) / 25.4;
  
      if (reading >= (float)(closedDistance - graceDistance)) {
        //Door is already closed, so just publish "CLOSED"
        client.publish("GarageState", "CLOSED");
        previousState = "CLOSED";
      }
      else {
        /* Door should be open
         * Emulate pressing the button to close door. 
         */
        digitalWrite(openClosePin, HIGH);
        delay(200);
        digitalWrite(openClosePin, LOW);
    
        /* Dividing by 25.4 both averages the 10 readings
         * and converts the reading to inches simultaneously.
         */        
        float firstReading = (float)(getReading()) / 25.4;
    
        delay(2000);
    
        /* Dividing by 25.4 both averages the 10 readings
         * and converts the reading to inches simultaneously.
         */        
        float secondReading = (float)(getReading()) / 25.4;
    
        if (abs(firstReading - secondReading) <= 1) {
          /* Emulate pressing button again because the door
           * probably didn't move. 
           */
          digitalWrite(openClosePin, HIGH);
          delay(200);
          digitalWrite(openClosePin, LOW);
        }
      } 
    }
  }
  else if (dataString == "power") {
    //Toggle reporting state on/off
    isDisabled = !isDisabled;
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print(F("Attempting MQTT connection..."));
    String clientId = "MainGarageDoor-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println(F("Connected"));
      client.publish("GarageState", "Hello World");
      client.subscribe("GarageRequest");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void calibrate() {
  bool calibrationMode = true;
  openDistance = 0;
  closedDistance = 0;

  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  interrupts();

  Serial.println(F("\nEntered calibration mode..."));
  
  while (calibrationMode) {
    /* Wait for user to initially release the calibration button
     * So we don't get a false first reading 
     */    
    while(digitalRead(calibrationButton) == LOW) {
      /* I made this a PULLUP so the input is not left floating
       * and possibly giving false presses during calibration. 
       */
      delay(20);
    }

    //Now we wait for user to press button to get open distance.
    while (digitalRead(calibrationButton) == HIGH) {
      /* I made this a PULLUP so the input is not left floating
       * and possibly giving false presses during calibration. 
       */
      delay(20);
    }

    Serial.println(F("\nGetting door open reading..."));    
    /* Get current distance reading
     * Dividing by 25.4 both averages the 10 readings
     * and converts the reading to inches simultaneously.
     */
    openDistance = (float)(getReading()) / 25.4;

    Serial.print(F("\nDoor open value: "));
    Serial.println(openDistance);
      
    /* Wait for user to release calibration button
     * before moving to the closed door calibration 
     */
    while(digitalRead(calibrationButton) == LOW) {
      delay(20);
    }
    
    /* Wait until user presses the calibrate button to dial
     * in the "closed" distance. Also set the timer counter
     * to a lower value to make the LED flash faster. It 
     * provides an indication to the user which distance it's 
     * currently calibrating. 
     */
    timer1_write(1000000);
    
    while (digitalRead(calibrationButton) == HIGH) {
      /* I made this a PULLUP so the input is not left floating
       * and possibly giving false presses during calibration. 
       */
      delay(20);
    }

    Serial.println(F("\nGetting door closed value..."));
    /* Get current distance reading
     * Dividing by 25.4 both averages the 10 readings
     * and converts the reading to inches simultaneously.
     */
    closedDistance = (float)(getReading()) / 25.4;

    Serial.print(F("\nDoor closed value: "));
    Serial.println(closedDistance);

    Serial.println(F("\nSaving to EEPROM..."));
    
    //Save calibration values to EEPROM
    EEPROM.begin(8);
    EEPROM.put(0,openDistance);
    EEPROM.put(4,closedDistance);
    EEPROM.commit();
    EEPROM.end();

    Serial.println(F("\nLeaving calibration mode..."));
    //Exit calibration mode
    calibrationMode = false;
  }

  //Reset timerCounter back to it's original value and disable the timer.
  timer1_write(3000000);
  timer1_disable();
  digitalWrite(ledPin, LOW);
}

uint16_t getReading() {
  uint16_t reading = 0;
  for (int i = 0; i < 10; i++) {
    uint16_t dist = distanceSensor.getDistance();
    reading += dist;
    delay(80);
  }
  return reading;
}

void inline ledISR(void) {
  digitalWrite(ledPin, !digitalRead(ledPin));
}

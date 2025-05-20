#define RELAY_PIN_1 5
#define RELAY_PIN_2 6
char recv_buf[5] = {0, };

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);  
  digitalWrite(RELAY_PIN_1, LOW);
  digitalWrite(RELAY_PIN_2, LOW);
}

void loop() {
  if (Serial.readBytesUntil('\n', recv_buf, sizeof(recv_buf)) != 0) {
    if (recv_buf[0] == 'D') {  
      if (recv_buf[1] == 'O') {  
        digitalWrite(RELAY_PIN_1, HIGH);  
        digitalWrite(RELAY_PIN_2, HIGH); 
        Serial.println("RELAY ON"); // 상태 피드백
      }
      else if (recv_buf[1] == 'F') {  
        digitalWrite(RELAY_PIN_1, LOW);   
        digitalWrite(RELAY_PIN_2, LOW);   
        Serial.println("RELAY OFF"); // 상태 피드백
      }
    }
  }
}

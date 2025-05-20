//A0(S1): SR A1(S2): SL A2(S3):RL A3(S4): RR
char recv_buf[5] = {0, }; 
 
void setup() {
  Serial.begin(115200);
}
 
void loop() {
  if (Serial.readBytesUntil('\n', recv_buf, 5) != 0)
  {
    if (recv_buf[0] == 'S' && recv_buf[1] == 'E') 
    {
      int sensorValue1 = analogRead(A0);
      int sensorValue2 = analogRead(A1);
      int sensorValue3 = analogRead(A2);
      int sensorValue4 = analogRead(A3);
 
      String data = String(sensorValue1) + "," + String(sensorValue2) + "," + String(sensorValue3) + "," + String(sensorValue4);
      Serial.write(data.c_str());
      Serial.write("\n");
    }
  }
}

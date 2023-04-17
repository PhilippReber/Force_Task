int N_SAMPLES_TO_AVERAGE = 300;

void setup() {
  Serial.begin(9600);
}

double val = 0;
void loop() {
  while (!Serial.available());
  Serial.readStringUntil('\n');

  for (int i = 0; i < N_SAMPLES_TO_AVERAGE; i++) {  
    val += analogRead(A0);
  }

  val /= N_SAMPLES_TO_AVERAGE;
  Serial.println(val);
  delay(1);
}

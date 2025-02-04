// Definer onboard LED
const int ledPin = LED_BUILTIN;

// Variabler til at holde styr på tiden
unsigned long previousMillis = 0;
const long interval = 2000; // 2 sekunder mellem hver cyklus

// Variabel til at sende data
char dataToSend;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600); // Start Serial kommunikation
  randomSeed(analogRead(0)); // Initialiser random seed
}

void loop() {
  unsigned long currentMillis = millis();

  // Håndter dataafsendelse hver 2. sekund
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Generer random værdi mellem '0' og '9'
    dataToSend = random('0', '9' + 1);

    // Send data til Serial
    Serial.write(dataToSend);
  }

  // Check for indgående data
  if (Serial.available() > 0) {
    char receivedChar = Serial.read();

    // Blink LED det antal gange som modtaget værdi
    if (receivedChar >= '0' && receivedChar <= '9') {
      int blinkCount = receivedChar - '0'; // Konverter char til tal

      for (int i = 0; i < blinkCount; i++) {
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
        delay(100);
      }
    }
  }
}

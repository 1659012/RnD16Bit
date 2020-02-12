#include <NewPing.h>
#define Trigger_Pin 8
#define Echo_Pin 7
#define Max_Distance 200
NewPing sonar(Trigger_Pin, Echo_Pin,Max_Distance);
void setup()
{
 Serial.begin(9600);
}

void loop()
{
  delay(50);
  unsigned int uS= sonar.ping();
  pinMode(Echo_Pin, OUTPUT);
  digitalWrite(Echo_Pin, LOW);
  pinMode(Echo_Pin,INPUT);
  Serial.print("Ping: ");
  Serial.print(uS/US_ROUNDTRIP_CM);
  Serial.println("cm");
}

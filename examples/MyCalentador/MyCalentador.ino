#include <oven.h>

#define THERMOCOUPLEPIN    A2
#define ZEROCROSSPIN        2
#define PWMOUTPUTPIN       11

OvenControl calentador = OvenControl(THERMOCOUPLEPIN, ZEROCROSSPIN, PWMOUTPUTPIN);

long setPoint=100;

void setup() {
  // put your setup code here, to run once:
  calentador.setAccuracy(2);
  calentador.setPWMSkip(10);
  calentador.myPID->SetTunings(50, 0.5, 0.01);
  calentador.moveTo(setPoint);

  Serial.begin(9600);
}

long temp;
unsigned long nextCheck=millis();

void loop()
{
  // put your main code here, to run repeatedly:
  if (millis() > nextCheck){
    nextCheck += 100;
    temp= calentador.getActualTemperature();
    Serial.println(temp);
    
    calentador.run();
    
    if(!calentador.finished()) {
      calentador.moveTo(setPoint);
    }
    else{
      calentador.stop();
    }
  }
}

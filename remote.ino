#include <SPI.h>
#include "RF24.h"

#define MAX_TILT 20


RF24 radio(9,10);

int a = 0;
uint8_t data[6];
int controll_number = 159;
int safe_lock = 0;
int x_offset, y_offset;

const uint64_t pipe = 0xE8E8F0F0E1LL;

void setup(void){
  
  Serial.begin(57600);

 pinMode(4, OUTPUT);
 pinMode(3, INPUT);
 pinMode(A0, INPUT);
 pinMode(A1, INPUT);
 pinMode(A2, INPUT);
 pinMode(A3, INPUT);
 digitalWrite(3, HIGH);
 digitalWrite(4, HIGH);

  radio.begin();

radio.setDataRate(RF24_250KBPS);
radio.setPALevel(RF24_PA_MAX);

    radio.openWritingPipe(pipe);
  radio.printDetails();

}

void loop(void)
{
  if(!digitalRead(3)){
    Serial.print("LOW 1");
    delay(1000);
    if(!digitalRead(3)){
      Serial.print("LOW 2");
    delay(1000);
    if(!digitalRead(3)){
      Serial.print("LOW 3");
      if(safe_lock == 0){
        safe_lock = 1;
      }else{
        safe_lock = 0;
      }
    
  }
  }
  }


int power = map(analogRead(A2), 0, 1023, 0, 255);
int x = map(analogRead(A1), 0, 1023, 0, 255);
int y = map(analogRead(A0), 0, 1023, 0, 255);
int rotation = map(analogRead(A3), 0, 1023, 0, 255);






if(x > 150){
  x = map(x, 150, 255, 0, MAX_TILT);
}else if(x < 105){
  x = map(x, 105, 0, 0, -MAX_TILT);
}else{
  x = 0;
}

if(y > 150){
  y = map(y, 150, 255, 0, MAX_TILT);
}else if(y < 105){
  y = map(y, 105, 0, 0, -MAX_TILT);
}else{
  y = 0;
}


if(rotation > 150){
  rotation = map(rotation, 150, 255, 0, MAX_TILT);
}else if(rotation < 105){
  rotation = map(rotation, 105, 0, 0, -MAX_TILT);
}else{
  rotation = 0;
}

  data[0] = controll_number;
  data[1] = power;

  // + 10 because you can't send negative number
  data[2] = x + MAX_TILT;
  data[3] = y + MAX_TILT;
  data[4] = rotation + MAX_TILT;
  data[5] = safe_lock;




radio.write( data, sizeof(data) );

 delay(8);
}

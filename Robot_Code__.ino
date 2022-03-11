#include <Adafruit_DotStar.h>
#define NUMPIXELS 144
#define DATAPIN 11
#define CLOCKPIN 13

Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BGR);

void setup() {
  strip.begin();
  strip.setBrightness(40);
  strip.clear();
  strip.show();
  pinMode(A2, OUTPUT);
}
int larp = 1;
int curFrame = 0;
int curFrame2 = 143;
int curFrame3 = 1;
long int prevTime = 0;
int red;
int blue;

void loop() {
  
  long int curTime = millis();
  bool continueP;
if(analogRead(A2) == 0) {
  strip.clear();
  strip.show();
}
else if(analogRead(A2) >= 1) {
  if(analogRead(A2) > 1000) {
    blue = 255;
    red = 0;
  }
  else if((analogRead(A2) <= 999) && (analogRead(A2) >= 1)) {
    blue = 0;
    red = 255;
  }
  
  if((curTime - prevTime >= 10) && (larp == 1)) {   
    continueP = stripeUp(curFrame);
    strip.show();
    if(continueP == true) {
      curFrame++;
    } 
    else if(continueP == false) {
      curFrame = 0;
      larp = 2;
    }
    prevTime = curTime;
  } // end timing if statement

  if((curTime - prevTime >= 10) && (larp == 2)) {   
    continueP = stripeDown(curFrame2);
    strip.show();
    if(continueP == true) {
      curFrame--;
    } 
    else if(continueP == false) {
      curFrame = 0;
      larp = 3;
    }
    prevTime = curTime;
  }

  if((curTime - prevTime >= 10) && (larp == 3)) {
    continueP = lightOverlap(curFrame, curFrame2);
    strip.show();
    if(continueP == true) {
      curFrame++;
      curFrame2--;
    }
    else if(continueP == false) {
      curFrame = 0;
      curFrame2 = 143;
      larp = 4;
    }
    prevTime = curTime;
  }

  if((curTime - prevTime >= 20) && (larp == 4)) {
    continueP = oneSkipLame(curFrame, curFrame3);
    strip.show();
    if(continueP == true) {
      curFrame += 2;
      curFrame3 += 2;
    }
    else if(continueP == false) {
      curFrame = 0;
      curFrame3 = 1;
      larp = 5;
    }
    prevTime = curTime;
  }

  if((curTime - prevTime >= 20) && (larp == 5)) {
    continueP = oneSkipCool(curFrame, curFrame2);
    strip.show();
    if(continueP == true) {
      curFrame += 2;
      curFrame2 -= 2;
    }
    else if(continueP == false) {
      curFrame = 0;
      curFrame2 = 143;
      larp = 1;
    }
   prevTime = curTime;
  }
}
}

bool stripeUp(int frameNumber) { //...........................................................................................................................................................................................................................................
  if(frameNumber == 0) {
    strip.clear();
    return true;
  }
  else if(frameNumber < 143) {
    strip.setPixelColor(frameNumber, strip.Color(red, 0, blue));
    return true;
  }
  else if(frameNumber >= 144) {
    strip.setPixelColor(frameNumber, strip.Color(red, 0, blue));
    return false;
  }
}

bool stripeDown(int frameNumber) { //...........................................................................................................................................................................................................................................
  if(frameNumber == 143) {
    strip.clear();
    return true;
  }
  else if(frameNumber > 1) {
    strip.setPixelColor(frameNumber, strip.Color(red, 0, blue));
    return true;
  }
  else if(frameNumber >= 0) {
    strip.setPixelColor(frameNumber, strip.Color(red, 0, blue));
    return false;
  }
}

bool lightOverlap(int frameNumber, int frameNumber2) { //.....................................................................................................................................................................................................................
  if((frameNumber <= 143) && (frameNumber2 >= 0)) {
    strip.setPixelColor(frameNumber, strip.Color(red, 0, blue));
    strip.setPixelColor(frameNumber2, strip.Color(255, 255, 255));
    return true;
  }
  else if(frameNumber >= 144){
    strip.setPixelColor(frameNumber, strip.Color(red, 0, blue));
    strip.setPixelColor(frameNumber2, strip.Color(255, 255, 255));
    return false;
  }
}

bool oneSkipLame(int frameNumber, int frameNumber2) { //......................................................................................................................................................................................................................
  if((frameNumber <= 143) && (frameNumber2 <= 144)) {
    strip.setPixelColor(frameNumber, strip.Color(red, 0, blue));
    strip.setPixelColor(frameNumber2, strip.Color(255, 255, 255));
    return true;
  }
  else if(frameNumber >= 144) {
    strip.setPixelColor(frameNumber, strip.Color(red, 0, blue));
    strip.setPixelColor(frameNumber2, strip.Color(255, 255, 255));
    return false;
  }
}

bool oneSkipCool(int frameNumber, int frameNumber2) { //.....................................................................................................................................................................................................................
  if((frameNumber <= 143) && (frameNumber2 >= 0)) {
    strip.setPixelColor(frameNumber, strip.Color(255, 255, 255));
    strip.setPixelColor(frameNumber2, strip.Color(red, 0, blue));
    return true;
  }
  else if(frameNumber >= 144) {
    strip.setPixelColor(frameNumber, strip.Color(255, 255, 255));
    strip.setPixelColor(frameNumber2, strip.Color(red , 0, blue));
    return false;
  }
}

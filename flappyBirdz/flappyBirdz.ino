//-------------------------------------------------------------------------------
//  TinyCircuits TinyScreen FlappyBirdz Game Example
//  Last Updated 14 March 2016
//
//  This example shows how to create simple sprites with a memory structure ts_sprite
//  and add them to a list (spriteList) allowing for relatively simple game mechanics
//  in the main loop by modifying sprite x/y coordinates, checking for pixel collisions
//  using testPixelCollision, and displaying everything through drawBuffer.
//
//  Written by Ben Rose for TinyCircuits, http://TinyCircuits.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//-------------------------------------------------------------------------------

//ALPHA is just a value not equal to another color we're using- then we can test for ALPHA when needed.
const uint16_t ALPHA = 0x1111;

#include <TinyScreen.h>
#include "flappyBirdSprites.h"

TinyScreen display = TinyScreen(TinyScreenPlus);


typedef struct
{
  int x;
  int y;
  int width;
  int height;
  int bitmapNum;
  const unsigned int * bitmap;

} ts_sprite;

typedef struct
{
  int x;
  int y;
  int height;
  char stringChars[40];

} ts_string;

bool testBitmapCollision(ts_sprite *s1, ts_sprite *s2);
bool testPixelCollision(ts_sprite *s1, ts_sprite *s2);

int collisionDetected = 0;
int wingChangeCount = 3;
int wingPos = 1;
int pipeOffsetX = 5;
int pipeSpacingX = 34;
int movePipe = 4;
int movePipeMod = 1;
int pipeSpacingY = 32;
int frame = 0;
int cloudSpacingX = 30;
int cloudOffsetX = 0;
int speedUpBoxActive = 0;
int speedUpBoxHit = 0;
int speedUp = 0;
int slowBoxActive = 0;
int slowBoxHit = 0;
int slowUp = 0;
int closeBoxActive = 0;
int closeBoxHit = 0;
int doCloseBox = 0;
int darkBoxActive = 0;
int darkBoxHit = 0;
int doDarkBox = 0;
int heartBoxActive = 0;
int heartBoxHit = 0;

int defaultBrightness = 8;
int currentBrightness = 8;
int startScreen = 1;


ts_sprite flappyBird = { -25, 22, 17, 12, 0, flappyBirdBitmap};
ts_sprite wing = { -25, 23, 7, 8, 0, wingBitmap};
ts_sprite pipeUp0 = {0, 40, 12, 40, 0, greenPipeUpBitmap};
ts_sprite pipeUp1 = {0, 40, 12, 40, 0, greenPipeUpBitmap};
ts_sprite pipeUp2 = {0, 40, 12, 40, 0, greenPipeUpBitmap};
ts_sprite pipeUp3 = {0, 40, 12, 40, 0, greenPipeUpBitmap};
ts_sprite pipeDown0 = {0, -pipeSpacingY, 12, 40, 0, greenPipeDownBitmap};
ts_sprite pipeDown1 = {0, -pipeSpacingY, 12, 40, 0, greenPipeDownBitmap};
ts_sprite pipeDown2 = {0, -pipeSpacingY, 12, 40, 0, greenPipeDownBitmap};
ts_sprite pipeDown3 = {0, -pipeSpacingY, 12, 40, 0, greenPipeDownBitmap};
ts_sprite cloud0 = {55, 0, 15, 10, 0, cloudBitmap};
ts_sprite cloud1 = {55, 15, 15, 10, 0, cloudBitmap};
ts_sprite cloud2 = {55, 30, 15, 10, 0, cloudBitmap};
ts_sprite cloud3 = {55, 30, 15, 10, 0, cloudBitmap};
ts_sprite ground = {0, 52, 105, 12, 0, groundBitmap};
ts_sprite speedBox = { -10, -10, 10, 10, 0, speedUpBoxBitmap};
ts_sprite closeBox = { -10, -10, 10, 10, 0, closeBoxBitmap};
ts_sprite darkBox = { -10, -10, 10, 10, 0, darkBoxBitmap};
ts_sprite slowBox = { -10, -10, 10, 10, 0, slowBoxBitmap};
ts_sprite heartBox = { -10, -10, 10, 10, 0, heartBoxBitmap};
ts_sprite title = { 0, -60, 96, 56, 0, flappyTitle};
ts_sprite hearts = { -35, 2, 70, 5, 0, heartBitmap};

int lives = 0;
ts_sprite * spriteList[22] = {&cloud0, &cloud1, &cloud2, &cloud3, &pipeUp0, &pipeDown0, &pipeUp1, &pipeDown1, &pipeUp2, &pipeDown2, &pipeUp3, &pipeDown3, &ground, &hearts, &closeBox, &speedBox, &darkBox, &heartBox, &slowBox, &flappyBird, &wing, &title};

int amtSprites = 22;

int highScore = 0;
int currentScore = 0;
int showScore = 0;
ts_string score = {0, -20, 0, "0"};

void setup() {
  pinMode(44, INPUT_PULLUP);
  pinMode(45, INPUT_PULLUP);
  display.begin();
  display.setBitDepth(TSBitDepth16);
  display.setBrightness(defaultBrightness);
  display.setFlip(false);
}

int getInput() {
  //if (display.getButtons())
  //  return 1;
  if (!digitalRead(44) || !digitalRead(45))
    return 1;
  if (abs((int)analogRead(42) - 512) > 300)
    return 1;
  if (abs((int)analogRead(1) - 512) > 300)
    return 1;
  return 0;
}

void loop() {
  frame++;

  ground.x--;
  if (ground.x < -4)ground.x = 0;

  if (pipeUp0.x + pipeUp0.width < 0) {
    pipeUp0.x = pipeUp1.x;
    pipeUp0.y = pipeUp1.y;
    pipeDown0.x = pipeDown1.x;
    pipeDown0.y = pipeDown1.y;
    pipeUp1.x = pipeUp2.x;
    pipeUp1.y = pipeUp2.y;
    pipeDown1.x = pipeDown2.x;
    pipeDown1.y = pipeDown2.y;
    pipeUp2.x = pipeUp3.x;
    pipeUp2.y = pipeUp3.y;
    pipeDown2.x = pipeDown3.x;
    pipeDown2.y = pipeDown3.y;
    pipeUp3.x = pipeUp2.x + pipeSpacingX - doCloseBox;
    pipeDown3.x = pipeDown2.x + pipeSpacingX - doCloseBox;
    pipeUp3.y = 33 + micros() % 18;
    pipeDown3.y = pipeUp3.y - pipeSpacingY - 40;
    doCloseBox = 0;
  }
  pipeUp0.x--;
  pipeDown0.x--;
  pipeUp1.x--;
  pipeDown1.x--;
  pipeUp2.x--;
  pipeDown2.x--;
  pipeUp3.x--;
  pipeDown3.x--;

  if (pipeUp3.y == 38 && !speedUpBoxActive) {
    speedUpBoxActive = 1;
    speedBox.x = pipeUp3.x + 2;
    speedBox.y = pipeUp3.y - (pipeSpacingY / 2) - (speedBox.height / 2);
    speedBox.bitmapNum = 0;
  } else if (pipeUp3.y == 37 && !closeBoxActive) {
    closeBoxActive = 1;
    closeBox.x = pipeUp3.x + 2;
    closeBox.y = pipeUp3.y - (pipeSpacingY / 2) - (closeBox.height / 2);
    closeBox.bitmapNum = 0;
  } else if (pipeUp3.y == 36 && !darkBoxActive) {
    darkBoxActive = 1;
    darkBox.x = pipeUp3.x + 2;
    darkBox.y = pipeUp3.y - (pipeSpacingY / 2) - (darkBox.height / 2);
    darkBox.bitmapNum = 0;
  } else if (pipeUp3.y == 35 && !slowBoxActive) {
    slowBoxActive = 1;
    slowBox.x = pipeUp3.x + 2;
    slowBox.y = pipeUp3.y - (pipeSpacingY / 2) - (slowBox.height / 2);
    slowBox.bitmapNum = 0;
  } else if (pipeUp3.y == 34 && !heartBoxActive) {
    heartBoxActive = 1;
    heartBox.x = pipeUp3.x + 2;
    heartBox.y = pipeUp3.y - (pipeSpacingY / 2) - (heartBox.height / 2);
    heartBox.bitmapNum = 0;
  }

  if (frame & 1)
    cloudOffsetX--;
  if (cloudOffsetX <= -14) {
    cloudOffsetX += cloudSpacingX;
    cloud0.y = cloud1.y;
    cloud1.y = cloud2.y;
    cloud2.y = cloud3.y;
    cloud3.y = -5 + micros() % 40;
  }
  cloud0.x = cloudOffsetX + cloudSpacingX * 0;
  cloud1.x = cloudOffsetX + cloudSpacingX * 1;
  cloud2.x = cloudOffsetX + cloudSpacingX * 2;
  cloud3.x = cloudOffsetX + cloudSpacingX * 3;

  if (speedUpBoxActive) {
    speedBox.x--;
    if (speedUpBoxHit) speedBox.y--;
    if (speedBox.x < -speedBox.width || speedBox.y < -speedBox.height) {
      speedUpBoxHit = 0;
      speedUpBoxActive = 0;
    }
    if (!(frame % 3)) {
      speedBox.bitmapNum ^= 1;
    }
    if (testPixelCollision(&flappyBird, &speedBox)) {
      speedUpBoxHit = 1;
      speedUp = 10000;
    }
  }

  if (slowBoxActive) {
    slowBox.x--;
    if (slowBoxHit) slowBox.y--;
    if (slowBox.x < -slowBox.width || slowBox.y < -slowBox.height) {
      slowBoxHit = 0;
      slowBoxActive = 0;
    }
    if (!(frame % 3)) {
      slowBox.bitmapNum ^= 1;
    }
    if (testPixelCollision(&flappyBird, &slowBox)) {
      slowBoxHit = 1;
      speedUp = -10000;
    }
  }

  if (closeBoxActive) {
    closeBox.x--;
    if (closeBoxHit) closeBox.y--;
    if (closeBox.x < -closeBox.width || closeBox.y < -closeBox.height) {
      closeBoxHit = 0;
      closeBoxActive = 0;
    }
    if (frame & 1) {
      closeBox.bitmapNum ^= 1;
    }
    if (testPixelCollision(&flappyBird, &closeBox)) {
      closeBoxHit = 1;
      doCloseBox = 10;
    }
  }

  if (darkBoxActive) {
    darkBox.x--;
    if (darkBoxHit) darkBox.y--;
    if (darkBox.x < -darkBox.width || darkBox.y < -darkBox.height) {
      darkBoxHit = 0;
      darkBoxActive = 0;
    }
    if (frame & 1) {
      darkBox.bitmapNum ^= 1;
    }
    if (testPixelCollision(&flappyBird, &darkBox)) {
      darkBoxHit = 1;
      doDarkBox = 20;
    }
  }

  if (heartBoxActive) {
    heartBox.x--;
    if (heartBoxHit) heartBox.y--;
    if (heartBox.x < -heartBox.width || heartBox.y < -heartBox.height) {
      heartBoxHit = 0;
      heartBoxActive = 0;
    }
    if (frame & 1) {
      heartBox.bitmapNum ^= 1;
    }
    if (!heartBoxHit && testPixelCollision(&flappyBird, &heartBox)) {
      heartBoxHit = 1;
      lives++;
    }
  }

  if (getInput()) {
    wingChangeCount = 2;
    if (flappyBird.y > 0) {
      flappyBird.y -= 2;
      wing.y = flappyBird.y + 1;
    }
  } else {
    wingChangeCount = 6;
    if (flappyBird.y < ground.y - flappyBird.height) {
      flappyBird.y += 1;
      wing.y = flappyBird.y + 1;
    }
  }

  if (!(frame % wingChangeCount)) {
    wing.bitmapNum++;
    if (wing.bitmapNum > 2)wing.bitmapNum = 0;
  }

  pipeUp1.bitmap = greenPipeUpBitmap;
  pipeDown1.bitmap = greenPipeDownBitmap;
  pipeUp0.bitmap = greenPipeUpBitmap;
  pipeDown0.bitmap = greenPipeDownBitmap;

  if (testPixelCollision(&flappyBird, &pipeUp0)) {
    pipeUp0.bitmap = redPipeUpBitmap;
    if (!collisionDetected) {
      collisionDetected = 1;
      lives--;
    }
  } else if (testPixelCollision(&flappyBird, &pipeUp1)) {
    pipeUp1.bitmap = redPipeUpBitmap;
    if (!collisionDetected) {
      collisionDetected = 1;
      lives--;
    }
  } else if (testPixelCollision(&flappyBird, &pipeDown0)) {
    pipeDown0.bitmap = redPipeDownBitmap;
    if (!collisionDetected) {
      collisionDetected = 1;
      lives--;
    }
  } else if (testPixelCollision(&flappyBird, &pipeDown1)) {
    pipeDown1.bitmap = redPipeDownBitmap;
    if (!collisionDetected) {
      collisionDetected = 1;
      lives--;
    }
  }

  if (pipeUp0.x + pipeUp0.width == flappyBird.x) {
    if (!collisionDetected) {
      currentScore++;
      if (!(currentScore % 10)) {
        lives++;
      }
      sprintf(score.stringChars, "%d", currentScore);
      score.x = 50;
      score.y = 20;
      showScore = 1;
    }
    collisionDetected = 0;
  }

  if (hearts.x > (lives * 7) - hearts.width + 2) {
    hearts.x--;
  }
  if (hearts.x < (lives * 7) - hearts.width + 2) {
    hearts.x++;
  }

  if (!lives) {
    if (currentScore > highScore) {
      highScore = currentScore;
    }
    startScreen = 1;
    currentScore = 0;
  }

  if (showScore) {
    if (score.y > -20)
      score.y -= 2;
    else
      showScore = 0;
  }

  if (startScreen) {
    if (flappyBird.x > -25) {
      flappyBird.x--;
      wing.x--;
    }
    if (title.y < 5)
      title.y += 2;
    else if (getInput()) {
      startScreen = 0;
      title.y = -60;
      flappyBird.x = 25;
      wing.x = 25;
      flappyBird.y = (pipeUp0.y + pipeUp1.y) / 2 - (pipeSpacingY / 2) - (flappyBird.height / 3);
      wing.y = flappyBird.y + 1;
      lives = 3;
    }
  }




  unsigned long timer = micros();
  drawBuffer();
  timer = micros() - timer;
  //while(!SerialUSB);
  //SerialUSB.println(timer);
  //delay(100);
  int delayTime = 14000 - speedUp - (currentScore / 10) * 1000;
  if (delayTime < 0)delayTime = 0;
  delayMicroseconds(delayTime);
  if (speedUp > 0)speedUp -= 50;
  if (speedUp < 0)speedUp += 100;

  if (!(frame % 4)) {
    if (doDarkBox) {
      doDarkBox--;
      currentBrightness--;
      if (currentBrightness < 0)currentBrightness = 0;
      display.setBrightness(currentBrightness);
    } else if (currentBrightness < defaultBrightness) {
      currentBrightness++;
      display.setBrightness(currentBrightness);
    }
  }
}

#define zmax(a,b) ((a)>(b)?(a):(b))
#define zmin(a,b) ((a)<(b)?(a):(b))

bool testBitmapCollision(ts_sprite *s1, ts_sprite *s2) {
  if (s1->x < s2->x + s2->width && s1->x + s1->width > s2->x)
    if (s2->y < s1->y + s1->height && s2->y + s2->height > s1->y)
      return true;
  return false;
}

bool testPixelCollision(ts_sprite *s1, ts_sprite *s2) {
  if (!testBitmapCollision(s1, s2))return false;
  int startX = zmax(s1->x, s2->x);
  int endX = zmin(s1->x + s1->width, s2->x + s2->width);
  int startY = zmax(s1->y, s2->y);
  int endY = zmin(s1->y + s1->height, s2->y + s2->height);
  for (int y = startY; y < endY; y++) {
    for (int x = startX; x < endX; x++) {
      if (s1->bitmap[(y - s1->y)*s1->width + (x - s1->x)] != ALPHA && s2->bitmap[(y - s2->y)*s2->width + (x - s2->x)] != ALPHA)
        return true;
    }
  }
  return false;
}

int writeCount = 0;

void drawBuffer() {
  uint8_t lineBuffer[96 * 64 * 2];
  display.startData();
  for (int y = 0; y < 64; y++) {
    for (int b = 0; b < 96; b++) {
      lineBuffer[b * 2] = TS_16b_Blue >> 8;
      lineBuffer[b * 2 + 1] = TS_16b_Blue;
    }
    for (int spriteIndex = 0; spriteIndex < amtSprites; spriteIndex++) {
      ts_sprite *cs = spriteList[spriteIndex];
      if (y >= cs->y && y < cs->y + cs->height) {
        int endX = cs->x + cs->width;
        if (cs->x < 96 && endX > 0) {
          int bitmapNumOffset = cs->bitmapNum * cs->width * cs->height;
          int xBitmapOffset = 0;
          int xStart = 0;
          if (cs->x < 0)xBitmapOffset -= cs->x;
          if (cs->x > 0)xStart = cs->x;
          int yBitmapOffset = (y - cs->y) * cs->width;
          for (int x = xStart; x < endX; x++) {
            unsigned int color = cs->bitmap[bitmapNumOffset + xBitmapOffset + yBitmapOffset++];
            if (color != ALPHA) {
              lineBuffer[(x) * 2] = color >> 8;
              lineBuffer[(x) * 2 + 1] = color;
            }
          }
        }
      }
    }
    putString(y, score.x, score.y, score.stringChars, lineBuffer, liberationSans_16ptFontInfo);
    if (startScreen) {
      putString(y, 1, 38, score.stringChars, lineBuffer, liberationSans_10ptFontInfo);
      char hs[10];
      sprintf(hs, "%d", highScore);
      putString(y, 74, 38, hs, lineBuffer, liberationSans_10ptFontInfo);

    }
    display.writeBuffer(lineBuffer, 96 * 2);
    
  }


  display.endTransfer();
}


void putString(int y, int fontX, int fontY, char * string, uint8_t * buff, const FONT_INFO& fontInfo) {
  const FONT_CHAR_INFO* fontDescriptor = fontInfo.charDesc;
  int fontHeight = fontInfo.height;
  if (y >= fontY && y < fontY + fontHeight) {
    const unsigned char* fontBitmap = fontInfo.bitmap;
    int fontFirstCh = fontInfo.startCh;
    int fontLastCh = fontInfo.endCh;
    //if(!_fontFirstCh)return 1;
    //if(ch<_fontFirstCh || ch>_fontLastCh)return 1;
    //if(_cursorX>xMax || _cursorY>yMax)return 1;
    int stringChar = 0;
    int ch = string[stringChar++];
    while (ch) {
      uint8_t chWidth = pgm_read_byte(&fontDescriptor[ch - fontFirstCh].width);
      int bytesPerRow = chWidth / 8;
      if (chWidth > bytesPerRow * 8)
        bytesPerRow++;
      unsigned int offset = pgm_read_word(&fontDescriptor[ch - fontFirstCh].offset) + (bytesPerRow * fontHeight) - 1;

      for (uint8_t byte = 0; byte < bytesPerRow; byte++) {
        uint8_t data = pgm_read_byte(fontBitmap + offset - (y - fontY) - ((bytesPerRow - byte - 1) * fontHeight));
        uint8_t bits = byte * 8;
        for (int i = 0; i < 8 && (bits + i) < chWidth; i++) {
          if (data & (0x80 >> i)) {
            buff[(fontX) * 2] = TS_16b_Yellow >> 8;
            buff[(fontX) * 2 + 1] = TS_16b_Yellow;
            // setPixelInBuff(y,16+fontX,0);
            //lineBuffer[16+fontX]=0;
          } else {
            //SPDR=_fontBGcolor;
          }
          fontX++;
        }
      }
      fontX += 2;
      ch = string[stringChar++];
    }
  }
}

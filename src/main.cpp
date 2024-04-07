
#include <Arduino.h>
/// #include <TinyUSB_Mouse_and_Keyboard.h>
// #include <HID.h>
#include <Mouse.h>
#include <Keyboard.h>
#include <OneButton.h>
#include <Tlv493d.h>
#include <SimpleKalmanFilter.h>

Tlv493d mag = Tlv493d();
SimpleKalmanFilter xFilter(1, 1, 0.2), yFilter(1, 1, 0.2), zFilter(1, 1, 0.2);

// Setup buttons
OneButton button1(10, true);
OneButton button2(11, true);


float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;

int calSamples = 300;
int sensivity = 16;    //     PHH   from 8 to 16
int magRange = 4;         // PHH increased magrange from 3 to 4
int outRange = 127;       // Max allowed in HID report
float xyThreshold = 0.35; // Center threshold

int inRange = magRange * sensivity;
float zThreshold = xyThreshold * 5;
bool isOrbit = false;

void goHome();
void fitToScreen();
int mapf(float x, float in_min, float in_max, float out_min, float out_max);

int mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void setup()
{

  button1.attachClick(goHome);
  button1.attachLongPressStop(goHome);

  button2.attachClick(fitToScreen);
  button2.attachLongPressStop(fitToScreen);

  // mouse and keyboard init
  Mouse.begin();
  Keyboard.begin();

  Serial.begin(9600);
  Wire.begin();

  // mag sensor init
  mag.begin(Wire);
  mag.setAccessMode(mag.MASTERCONTROLLEDMODE);
  mag.disableTemp();

  // crude offset calibration on first boot
  for (int i = 1; i <= calSamples; i++)
  {

    delay(mag.getMeasurementDelay());
    mag.updateData();

    xOffset += mag.getX();
    yOffset += mag.getY();
    zOffset += mag.getZ();

    Serial.print(".");
  }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;

  Serial.println();
  Serial.println(xOffset);
  Serial.println(yOffset);
  Serial.println(zOffset);
}

void loop()
{

  // keep watching the push buttons
  button1.tick();
  button2.tick();

  // get the mag data
  delay(mag.getMeasurementDelay());
  mag.updateData();

  // update the filters
  xCurrent = xFilter.updateEstimate(mag.getX() - xOffset);
  yCurrent = yFilter.updateEstimate(mag.getY() - yOffset);
  zCurrent = zFilter.updateEstimate(mag.getZ() - zOffset);

  // check the center threshold
  if ((xCurrent*xCurrent+yCurrent*yCurrent) > xyThreshold*xyThreshold)
  //if (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold)
  {

    int xMove = 0;
    int yMove = 0;

    // map the magnetometer xy to the allowed 127 range in HID repports
    // PHH inversé x et y,
    xMove = mapf(xCurrent, -inRange, inRange, -outRange, outRange);
    yMove = mapf(-yCurrent, -inRange, inRange, -outRange, outRange);

    // press shift to orbit in Fusion 360 if the pan threshold is not corssed (zAxis)
    if (abs(zCurrent) < zThreshold && !isOrbit)
    {
      Keyboard.press(KEY_LEFT_SHIFT);
      isOrbit = true;
    }
#ifdef DEBUG_ORBIT
    if (isOrbit)
      Serial.print("o");
    else
      Serial.print(".");
#endif

    // pan or orbit by holding the middle mouse button and moving propotionaly to the xy axis
    Mouse.press(MOUSE_MIDDLE);
    Mouse.move(xMove, yMove, 0);

#ifdef DEBUG_XY
    char text[128];
    sprintf(text, "%2d,%2d  %1.2f %1.2f %1.2f inRange=%2d outRange=%2d %s", xMove, yMove, xCurrent, yCurrent, zCurrent,inRange,outRange,isOrbit?"orbit":"pan");
    Serial.println(text);
#endif

  }
  else
  {

    // release the mouse and keyboard if within the center threshold
    Mouse.release(MOUSE_MIDDLE);
    Keyboard.releaseAll();
    isOrbit = false;
  }

  /*
  Serial.print(xCurrent);
  Serial.print(",");
  Serial.print(yCurrent);
  Serial.print(",");
  Serial.print(zCurrent);
  Serial.println();
  */
}

// go to home view in Fusion 360 by pressing  (CMD + SHIFT + H) shortcut assigned to the custom Add-in command
void goHome()
{
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.write('h');

  delay(10);
  Keyboard.releaseAll();
  Serial.println("pressed home");
}

// fit to view by pressing the middle mouse button twice
void fitToScreen()
{
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);

  Serial.println("pressed fit");
}
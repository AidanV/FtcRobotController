package com.qualcomm.robotcore.hardware;

/**
 * Implementation of the CRServo interface.
 */
public class ColorSensorImpl implements ColorSensor {

  int red = 255;
  int green = 0;
  int blue = 0;

  @Override
  public int red() {
    return red;
  }

  @Override
  public int green() {
    return green;
  }

  @Override
  public int blue() {
    return blue;
  }

  public void update(int r, int g, int b){
    red = r;
    green = g;
    blue = b;
  }
}

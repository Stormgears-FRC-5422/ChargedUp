package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.lights.AddressableLEDBufferRGBW;

public class NeoPixel extends SubsystemBase {
  private final AddressableLED lightStrip = new AddressableLED(9);
  private final AddressableLEDBufferRGBW buffer = new AddressableLEDBufferRGBW(19);
  public static final Color8Bit fullColor = new Color8Bit(211, 0, 211);//purple
  public static final Color8Bit blankColor = new Color8Bit(0, 0, 0);
  public static final Color8Bit fullColorY = new Color8Bit(200, 200, 0);//yellow

  public static final Color8Bit fullColorR = new Color8Bit(200, 0, 0);//yellow

  public static final Color8Bit fullColorG = new Color8Bit(0, 200, 0);//yellow
  private int rainbowHue = 0;
  public int countLED = 0;
  public int ring = buffer.getEffectiveLength() / 4;
  public int segment = buffer.getEffectiveLength() / 2;
  public int total = buffer.getEffectiveLength();

  public NeoPixel() {
    lightStrip.setLength(buffer.getEffectiveLength());
    lightStrip.setData(buffer);
    lightStrip.start();
  }

  public void setAll(Color8Bit color) {
//    System.out.println("SETALL RUNNING");
    lightStrip.setData(buffer);
    if (color != null) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, color);
      }
    }
  }

  @Override
  public void periodic() {
//setThirdRing(fullColorY);

//        setFirstHalf(fullColor);
//        setSecondHalf(fullColorY);
  }

  //    public Color8Bit getFullColor() {
//        return fullColor;
//    }
  private void blink() {
    countLED++;
    if (countLED % 50 > 25) {
      setAll(blankColor);
    }
    if (countLED % 50 < 25) {
      setAll(fullColorY);
    }
    if (countLED == 50) {
      countLED = 0;
    }
  }



  public void setSecondHalf(Color8Bit color) {
    for (int i = 0; i < segment; i++) {
      buffer.setLED(i + segment, color);
    }
    lightStrip.setData(buffer);
  }

  public void setFirstHalf(Color8Bit color) {
    for (int i = 0; i < segment; i++) {
      buffer.setLED(i, color);
    }
    lightStrip.setData(buffer);
  }

  //for light tower
  public void setFirstRing(Color8Bit color) {
    for (int i = 0; i < ring; i++) {
      buffer.setLED(i, color);
    }
    lightStrip.setData(buffer);
  }

  public void setSecondRing(Color8Bit color) {
    for (int i = 0; i < ring; i++) {
      buffer.setLED(i + ring, color);
    }
    lightStrip.setData(buffer);
  }

  public void setThirdRing(Color8Bit color) {
    for (int i = 0; i < ring; i++) {
      buffer.setLED(i + (ring * 2), color);
    }
    lightStrip.setData(buffer);
  }

  public void setFourthRing(Color8Bit color) {
    for (int i = 0; i < ring; i++) {
      buffer.setLED(i + (2 * ring), color);
    }
    lightStrip.setData(buffer);
  }

  private void rainbow() {
    for (int i = 0; i < buffer.getLength(); i++) {
      final int hue = (rainbowHue + (i * 180 / buffer.getLength())) % 180;
      buffer.setHSV(i, hue, 255, 128);
    }
    rainbowHue += 3;
    rainbowHue %= 180;
  }

}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.lights.LEDLightStrip;
import frc.utils.lights.LightType;

import java.lang.reflect.Array;
import java.util.Collection;

public class NeoPixel extends SubsystemBase {

  public static final Color8Bit PURPLE_COLOR = new Color8Bit(211, 0, 211);//purple
  public static final Color8Bit BLACK_COLOR = new Color8Bit(0, 0, 0);
  public static final Color8Bit YELLOW_COLOR = new Color8Bit(200, 200, 0);//yellow
  public static final Color8Bit RED_COLOR = new Color8Bit(200, 0, 0);//yellow
  public static final Color8Bit GREEN_COLOR = new Color8Bit(0, 200, 0);//yellow


  LEDLightStrip ledLightStrip;

  boolean ledColorRequested;

  public NeoPixel() {
    ledLightStrip = new LEDLightStrip();
    ledLightStrip.addSegment(12, LightType.RGB);
    ledLightStrip.addSegment(12, LightType.RGB);
    ledLightStrip.addSegment(12, LightType.RGB);
    ledLightStrip.addSegment(12, LightType.RGB);
    ledLightStrip.addSegment(19, LightType.RGBW);
    ledLightStrip.addSegment(8, LightType.RGBW);
    ledLightStrip.setUp(9);
  }

  public void setColor(int segmentNumber, Color8Bit color) {
//    System.out.println("setting color " + color.toString() + " for segment:" + segmentNumber);
    ledLightStrip.setLEDColor(segmentNumber, color);
    ledColorRequested = true;
  }

  public void setAllColor(Color8Bit color) {
    for (int segment = 0; segment < ledLightStrip.numOfSegments(); segment++) {
      setColor(segment, color);
    }
  }

  public void setSpecificSegmentColor(int[] segments, Color8Bit color) {
    for (int segment : segments) {
      setColor(segment, color);
    }
  }


}

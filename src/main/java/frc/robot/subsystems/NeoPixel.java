package frc.robot.subsystems;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.commands.LEDcommand;
import frc.utils.lights.LEDLightStrip;
import frc.utils.lights.LightType;
import frc.utils.subsystemUtils.StormSubsystemBase;

public class NeoPixel extends StormSubsystemBase {

    public static final Color8Bit PURPLE_COLOR = new Color8Bit(211, 0, 211);//purple
    public static final Color8Bit NO_COLOR = new Color8Bit(0, 0, 0);//off
    public static final Color8Bit YELLOW_COLOR = new Color8Bit(256, 120, 0);//yellow

    public static final Color8Bit RED_COLOR = new Color8Bit(200, 0, 0);//yellow
    public static final Color8Bit GREEN_COLOR = new Color8Bit(0, 200, 0);//yellow

    public static final Color8Bit BLUE_COLOR = new Color8Bit(0, 0, 200);//blue

//  private final Addre

    LEDLightStrip ledLightStrip;

    boolean ledColorRequested;

    public int countLED = 0;


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

    public void setBlinkColor(int segmentNumber, Color8Bit color, double distance) {
        double limit = 0.5;
        ledLightStrip.setLEDColor(segmentNumber, color);
        ledColorRequested = true;
        double error = limit - distance;
        int blinks = 0;
        if (error > 0.2) {
            blinks = 25;
        } else if (error > 0.15) {
            blinks = 20;
        } else if (error > 0.1) {
            blinks = 15;
        } else if (error > 0.05) {
            blinks = 10;
        } else if (error > 0) {
            blinks = 5;
        }
        countLED++;
        if (countLED % 50 > blinks) {
            setColor(segmentNumber, NO_COLOR);
        }
        if (countLED % 50 < blinks) {
            setColor(segmentNumber, color);
        }
        if (countLED == 50) {
            countLED = 0;
        }

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

    public void setAllColor(Color8Bit color, double distance) {
        for (int segment = 0; segment < ledLightStrip.numOfSegments(); segment++) {
            setBlinkColor(segment, color, distance);
        }
    }

    public void setSpecificSegmentColor(int[] segments, Color8Bit color, double distance) {
        for (int segment : segments) {
            setBlinkColor(segment, color, distance);
        }
    }

    @Override
    public void stormPeriodic() {
        if (ledColorRequested) {
            ledLightStrip.setLEDData();
        }
    }


}

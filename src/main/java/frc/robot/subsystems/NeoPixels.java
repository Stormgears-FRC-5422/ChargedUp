package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.lights.LEDLightStrip;
import frc.utils.lights.LightColors;
import frc.utils.lights.LightType;

public class NeoPixels extends SubsystemBase {
    LEDLightStrip ledLightStrip;
    public NeoPixels() {

        ledLightStrip = new LEDLightStrip();
        ledLightStrip.addSegment(37, LightType.RGBW);
        ledLightStrip.addSegment(40, LightType.RGBW);
        ledLightStrip.setUp(7);
    }

    @Override
    public void periodic() {

        ledLightStrip.setLEDColor(0, LightColors.BLUE_COLOR);
        ledLightStrip.setLEDColor(1,LightColors.ORANGE_COLOR);

        ledLightStrip.setLEDData();
    }

    public void setColor(String color) {

        switch (color) {
            case "red":
                ledLightStrip.setLEDColor(0,LightColors.RED_COLOR);
                ledLightStrip.setLEDColor(1,LightColors.RED_COLOR);
                break;
            case "orange":
                ledLightStrip.setLEDColor(0,LightColors.ORANGE_COLOR);
                ledLightStrip.setLEDColor(1,LightColors.ORANGE_COLOR);
                break;
            case "yellow":
                ledLightStrip.setLEDColor(0,LightColors.YELLOW_COLOR);
                ledLightStrip.setLEDColor(1,LightColors.YELLOW_COLOR);
                break;
            case "green":
                ledLightStrip.setLEDColor(0,LightColors.GREEN_COLOR);
                ledLightStrip.setLEDColor(1,LightColors.GREEN_COLOR);
                break;
            case "blue":
                ledLightStrip.setLEDColor(0,LightColors.BLUE_COLOR);
                ledLightStrip.setLEDColor(1,LightColors.BLUE_COLOR);
                break;
            case "indigo":
                ledLightStrip.setLEDColor(0,LightColors.INDIGO_COLOR);
                ledLightStrip.setLEDColor(1,LightColors.INDIGO_COLOR);
                break;
            case "violet":
                ledLightStrip.setLEDColor(0,LightColors.PURPLE_COLOR);
                ledLightStrip.setLEDColor(1,LightColors.PURPLE_COLOR);
                break;
        }

    }

    public void setNullColor() {
        ledLightStrip.setLEDColor(0, LightColors.NO_COLOR);
        ledLightStrip.setLEDColor(1, LightColors.NO_COLOR);
        ledLightStrip.setLEDColor(2, LightColors.NO_COLOR);
    }
}

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
        ledLightStrip.addSegment(40, LightType.RGBW);
        ledLightStrip.setUp(7);
    }



    @Override
    public void periodic() {



        ledLightStrip.setLEDData();

    }

    public void setColor(Color8Bit color1, Color8Bit color2) {
        ledLightStrip.setLEDColor(0,color1);
        ledLightStrip.setLEDColor(1,color2);
        ledLightStrip.setLEDColor(2,color2);
    }

    public void setNullColor() {
        ledLightStrip.setLEDColor(0, LightColors.NO_COLOR);
        ledLightStrip.setLEDColor(1, LightColors.NO_COLOR);
        ledLightStrip.setLEDColor(2, LightColors.NO_COLOR);
    }
}

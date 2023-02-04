package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.lights.AddressableLEDBufferRGBW;

import static frc.robot.Constants.*;
public class NeoPixel extends SubsystemBase {
    private final AddressableLED lightStrip = new AddressableLED(0);
    private final AddressableLEDBufferRGBW buffer = new AddressableLEDBufferRGBW(14);
    private Color8Bit fullColor;
    private int rainbowHue = 0;

    private int countLED = 0;


    public NeoPixel() {
        lightStrip.setLength(buffer.getEffectiveLength());
        lightStrip.setData(buffer);
        lightStrip.start();
        fullColor = new Color8Bit(153, 51, 255);
//        fullColor = new Color8Bit(255, 213, 0); yellow
    }

    @Override
    public void periodic() {
        lightStrip.setData(buffer);
    //    rainbow();
        countLED++;
        System.out.println(countLED);
        if (countLED % 2 == 0) {
            setAll(fullColor);
            countLED = 0;
        } else{
            setAll(null);
        }


    }

    private void setAll(Color8Bit color) {
        if (color != null) {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, color);

            }
        }
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
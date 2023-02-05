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
    private final Color8Bit fullColor;
    private final Color8Bit blankColor;
    private int rainbowHue = 0;
    public int countLED = 0;

    public NeoPixel() {
        lightStrip.setLength(buffer.getEffectiveLength());
        lightStrip.setData(buffer);
        lightStrip.start();
        fullColor = new Color8Bit(153, 51, 255); //purple
        blankColor = new Color8Bit(0, 0, 0); //blank or off

    }
    public void blink(){
        countLED++;
        if (countLED % 50 > 25) {
            setAll(blankColor);
            System.out.println("should be off");
        };
        if (countLED % 50 < 25){
            setAll(fullColor);
        };
        if (countLED == 50) {
            countLED = 0;
        };
    }

    @Override
    public void periodic() {
        lightStrip.setData(buffer);
        //rainbow(); - rainbow light called
        blink(); //toggles between colors, currently full and blank
        System.out.println(countLED);
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
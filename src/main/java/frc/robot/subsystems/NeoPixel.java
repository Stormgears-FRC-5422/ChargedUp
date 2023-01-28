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
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(12);
    private Color8Bit fullColor;

    public NeoPixel() {
        lightStrip.setLength(buffer.getLength());
        lightStrip.setData(buffer);
        lightStrip.start();

        fullColor = new Color8Bit(64, 64, 0);
    }

    @Override
    public void periodic() {
        setAll(fullColor);
        lightStrip.setData(buffer);

    }

    private void setAll(Color8Bit color) {
        if (color != null) {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, color);
            }
        }
    }
}
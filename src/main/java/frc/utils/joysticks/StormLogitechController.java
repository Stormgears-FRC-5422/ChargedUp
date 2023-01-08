package frc.utils.joysticks;

import edu.wpi.first.wpilibj.Joystick;
import static frc.robot.Constants.kStickNullSize;

public class StormLogitechController extends Joystick {

    public static final int XAxis = 0;
    public static final int YAxis = 1;
    public static final int ZAxis = 2;
    public static final int sliderAxis = 3;

    public StormLogitechController(int port) {
        super(port);
    }

    private double applyNullZone(double value) {
        if (Math.abs(value) < kStickNullSize)
            return 0;

        return ((value - Math.signum(value) * kStickNullSize) / (1.0 - kStickNullSize));
    }

    public double getXAxis() {
        return applyNullZone(getRawAxis(XAxis));
    }
    public double getYAxis() {
        return -applyNullZone(getRawAxis(YAxis));
    }
    public double getZAxis() {
        return applyNullZone(getRawAxis(ZAxis));
    }
    public double getSliderAxis() {
        return (-applyNullZone(getRawAxis(sliderAxis)) + 1) / 2;
    }
}

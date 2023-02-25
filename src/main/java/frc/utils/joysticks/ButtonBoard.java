package frc.utils.joysticks;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonBoard extends Joystick {


    /**
     * Construct an instance of a joystick.
     *
     * @param port The port index on the Driver Station that the joystick is plugged into.
     */
    public ButtonBoard(int port) {
        super(port);
    }

    public boolean getLowerLeftCone() {
        return getRawButtonPressed(1);
    }

    public boolean getLowerCenterCube() {
        return getRawButtonPressed(2);
    }

    public boolean getLowerRightCone() {
        return getRawButtonPressed(3);
    }

    public boolean getMiddleLeftCone() {
        return getRawButtonPressed(4);
    }

    public boolean getMiddleCenterCube() {
        return getRawButtonPressed(5);
    }

    public boolean getMiddleRightCone() {
        return getRawButtonPressed(6);
    }

    public boolean getUpperLeftCone() {
        return getRawButtonPressed(7);
    }

    public boolean getLUpperCenterCube() {
        return getRawButtonPressed(8);
    }

    public boolean getUpperRightCone() {
        return getRawButtonPressed(9);
    }

}
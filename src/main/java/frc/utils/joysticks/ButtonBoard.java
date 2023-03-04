package frc.utils.joysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.NeoPixel;

import static java.util.Arrays.setAll;

public class ButtonBoard extends Joystick {
//     NeoPixel neoPixel = new NeoPixel();
//     Color8Bit fullColor = neoPixel.getFullColor();

    /**
     * Construct an instance of a joystick.
     *
     * @param port The port index on the Driver Station that the joystick is plugged into.
     */
    public ButtonBoard(int port) {
        super(port);
    }

    public boolean jumper() {
        return getRawButton(1);
    }

    public boolean button2() {
        return getRawButton(2);
    }

    public boolean button3() {
        return getRawButton(3);
    }

//    public boolean getLowerLeftCone() {
//        return getRawButtonPressed(1);
//    }
//
//    public boolean getLowerCenterCube() {
//        return getRawButtonPressed(2);
//    }
//
//    public boolean getLowerRightCone() {
//        return getRawButtonPressed(3);
//    }
//
//    public boolean getMiddleLeftCone() {
//        return getRawButtonPressed(4);
//    }
//
//    public boolean getMiddleCenterCube() {
//        return getRawButtonPressed(5);
//    }
//
//    public boolean getMiddleRightCone() {
//        return getRawButtonPressed(6);
//    }
//
//    public boolean getUpperLeftCone() {
//        return getRawButtonPressed(7);
//    }
//
//    public boolean getLUpperCenterCube() {
//        return getRawButtonPressed(8);
//    }
//
//    public boolean getUpperRightCone() {
//        return getRawButtonPressed(9);
//    }

//    public void neoPixel() {
//        GenericHID button = new Joystick(4);
//        new Trigger(() -> button.getRawButton(4)).whileTrue(neoPixel.setAll(fullColor));
//    }

}
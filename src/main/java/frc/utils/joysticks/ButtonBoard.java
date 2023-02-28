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



    public boolean ChargeStationBalance() {
        return getRawButtonPressed(1);
    }



}

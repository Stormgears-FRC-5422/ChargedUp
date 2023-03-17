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
    public boolean jumper() {
        return getRawButton(1);
    }

    public boolean store() {
        return getRawButton(2);
    }

    public boolean leftSub() {
    return getRawButton(12);
 }

    public boolean floor() {
        return getRawButton(11);
    }

    public boolean rightSub() {
    return getRawButton(10);
  }

    public boolean highMedium(){return getRawButton(6);}
    public boolean Low(){return getRawButton(5);}

    public boolean grid1() {
    return  getRawButton(12);
  }

    public boolean grid2() {
    return  getRawButton(11);
  }

    public boolean grid3() {
    return  getRawButton(10);
  }

    public boolean grid4() {
    return  getRawButton(9);
  }

    public boolean grid5() {
    return  getRawButton(8);
  }

    public boolean grid6() {
    return  getRawButton(7);
  }

    public boolean grid7() {
    return  getRawButton(6);
  }

    public boolean grid8() {
    return  getRawButton(5);
  }

    public boolean grid9() {
    return  getRawButton(4);
  }//w

    public boolean confirm() {
    return  getRawButton(3);
  }

    public boolean cancel() {
    return  getRawButton(4);
  }

    public boolean gripper(){return getRawButton(1);}
    public boolean cubeCone(){return getRawButton(7);}

    public boolean manualOverride(){return getRawButton(9);}

    public boolean kill() {
      return  getRawButton(8);
    }

  //HAVE NOT DONE switches (kill switch, manual arm control etc.)

}

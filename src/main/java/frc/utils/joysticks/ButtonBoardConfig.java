package frc.utils.joysticks;

import edu.wpi.first.wpilibj.Joystick;

import static frc.robot.constants.Constants.kXYArmManualSpeed;

public class ButtonBoardConfig {
    Joystick m_joystickBoard1;
    Joystick m_joystickBoard2;


    public ButtonBoardConfig() {
        m_joystickBoard1 = new Joystick(1);
        m_joystickBoard2 = new Joystick(2);
        if (!m_joystickBoard2.getRawButton(1)) {
            System.out.println("Switching ButtonBoard ports");
            m_joystickBoard1 = new Joystick(2);
            m_joystickBoard2 = new Joystick(1);
        } else {
            System.out.println("Not Switching ButtonBoard ports");
        }
    }

    public boolean kill() {
        return m_joystickBoard1.getRawButton(8);
    }

    public boolean getGridButton(int gridNum){
        return m_joystickBoard2.getRawButton(12 - gridNum + 1) ;
    }

    public boolean gripperClosed() {
        return m_joystickBoard2.getRawButton(3);
    }

    public boolean cubeCone() {
        return m_joystickBoard1.getRawButton(7);
    }

    public boolean topGrid() {
        return m_joystickBoard1.getRawButton(5) && !m_joystickBoard1.getRawButton(6);
    }

    public boolean middleGrid() {
        return !m_joystickBoard1.getRawButton(5) && !m_joystickBoard1.getRawButton(6);
    }

    public boolean bottomGrid() {
        return m_joystickBoard1.getRawButton(6) && !m_joystickBoard1.getRawButton(5);
    }

    public boolean confirm() {
        return m_joystickBoard1.getRawButton(3);
    }

    public boolean stow() {
        return m_joystickBoard1.getRawButton(2);
    }

    public boolean cancel() {
        return m_joystickBoard1.getRawButton(4);
    }

    public boolean pickFloor() {
        return m_joystickBoard1.getRawButton(11);
    }

    public boolean pickLeftSub() {
        return m_joystickBoard1.getRawButton(12);
    }

    public boolean pickRightSub() {
        return m_joystickBoard1.getRawButton(10);
    }

    public double armUpDown() {
        double y = m_joystickBoard2.getY();
        double dy = 0;
        // There is a slight voltage bias that causes the joystick to report != 0 at rest
        // any actual motion sets it to -1.0 or 1.0, so we just need some reasonable number in the middle here
        if (Math.abs(y) > 0.5) {
            if (y < 0)
                dy = kXYArmManualSpeed;
            else
                dy = -kXYArmManualSpeed;
        }

        //    if (dy != 0) System.out.println("armUpDown: dy = " + dy);
        return dy;
    }

    public double armInOut() {
        double x = m_joystickBoard1.getX();
        double dx = 0;
        // There is a slight voltage bias that causes the joystick to report != 0 at rest
        // any actual motion sets it to -1.0 or 1.0, so we just need some reasonable number in the middle here
        if (Math.abs(x) > 0.5) {
            if (x < 0)
                dx = kXYArmManualSpeed;
            else
                dx = -kXYArmManualSpeed;
        }
        //    if (dx != 0) System.out.println("armInOut: dx = " + dx);
        return dx;
    }
}

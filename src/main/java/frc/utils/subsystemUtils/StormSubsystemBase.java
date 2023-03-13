package frc.utils.subsystemUtils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** all of these methods are called before wpilib perioidcs */
public abstract class StormSubsystemBase extends SubsystemBase {
    public StormSubsystemBase() {
        StormSubsystemScheduler.getInstance().register(this);
        String className = this.getClass().getSimpleName();
        System.out.println(className + " created!");
    }

    /** use as periodic instead of periodic */
    public void stormPeriodic() {}
    public void enabledInit() {}
    public void enabledPeriodic() {}
    public void disabledInit() {}
    public void disabledPeriodic() {}
    public void autoInit() {}
    public void autoPeriodic() {}
    public void teleopInit() {}
    public void teleopPeriodic() {}
    /** Use this loop for telemetry etc. */
    public void lastPeriodic() {}
}

package frc.utils.subsystemUtils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StormSubsystemBase extends SubsystemBase {
    public StormSubsystemBase() {
        StormSubsystemScheduler.getInstance().register(this);
    }

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

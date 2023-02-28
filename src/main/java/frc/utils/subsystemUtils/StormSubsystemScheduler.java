package frc.utils.subsystemUtils;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.Arrays;

public final class StormSubsystemScheduler {

    private static StormSubsystemScheduler instance;

    public static StormSubsystemScheduler getInstance() {
        if (instance == null) instance = new StormSubsystemScheduler();
        return instance;
    }

    private final ArrayList<StormSubsystemBase> subsystems = new ArrayList<>();

    private boolean hasEnabled = false;
    private boolean autoHasEnabled = false;
    private boolean teleopHasEnabled = false;

    private StormSubsystemScheduler() {}

    public void run() {
        boolean isEnabled = DriverStation.isEnabled();
        boolean autoEnabled = DriverStation.isAutonomousEnabled();
        boolean teleopEnabled = DriverStation.isTeleopEnabled();

        for (var subsystem : subsystems) {
            if (isEnabled) {
                // rising edge of enabled-ness
                if (isEnabled && !hasEnabled) {
                    subsystem.enabledInit();
                    if (autoEnabled && !autoHasEnabled) subsystem.autoInit();
                    if (teleopEnabled && !teleopHasEnabled) subsystem.teleopInit();
                }
                subsystem.enabledPeriodic();
                if (autoEnabled) subsystem.autoPeriodic();
                if (teleopEnabled) subsystem.teleopPeriodic();
            } else {
                // falling edge of enabled-ness
                if (hasEnabled && !isEnabled) subsystem.disabledInit();
                subsystem.disabledPeriodic();
            }
            subsystem.lastPeriodic();
        }

        hasEnabled = isEnabled;
        autoHasEnabled = autoEnabled;
        teleopHasEnabled = teleopEnabled;
    }

    public void register(StormSubsystemBase... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }
}

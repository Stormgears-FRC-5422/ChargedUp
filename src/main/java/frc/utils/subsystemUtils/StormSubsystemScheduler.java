package frc.utils.subsystemUtils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Arrays;

public final class StormSubsystemScheduler {

    private static StormSubsystemScheduler instance;

    public static StormSubsystemScheduler getInstance() {
        if (instance == null) instance = new StormSubsystemScheduler();
        return instance;
    }

    private final ArrayList<StormSubsystemBase> subsystems = new ArrayList<>();

    private boolean wasEnabled = false;
    private boolean autoWasEnabled = false;
    private boolean teleopWasEnabled = false;

    private StormSubsystemScheduler() {}

    public void run() {
        double timeAtStart = Timer.getFPGATimestamp();
        boolean isEnabled = DriverStation.isEnabled();
        boolean autoEnabled = DriverStation.isAutonomousEnabled();
        boolean teleopEnabled = DriverStation.isTeleopEnabled();

        for (var subsystem : subsystems) {
            if (isEnabled) {
                subsystem.stormPeriodic();
                // rising edge of enabled-ness
                if (!wasEnabled) {
                    subsystem.enabledInit();
                    if (autoEnabled && !autoWasEnabled) subsystem.autoInit();
                    if (teleopEnabled && !teleopWasEnabled) subsystem.teleopInit();
                }
                subsystem.enabledPeriodic();
                if (autoEnabled) subsystem.autoPeriodic();
                if (teleopEnabled) subsystem.teleopPeriodic();
            } else {
                // falling edge of enabled-ness
                if (wasEnabled) subsystem.disabledInit();
                subsystem.disabledPeriodic();
            }
        }

        for (var subsystem : subsystems) {
            subsystem.lastPeriodic();
        }

        wasEnabled = isEnabled;
        autoWasEnabled = autoEnabled;
        teleopWasEnabled = teleopEnabled;
        double timeThroughLoop = Timer.getFPGATimestamp() - timeAtStart;
        if (timeThroughLoop > 0.05) {
            DriverStation.reportWarning("Time through StormSubsystemScheduler.run(): "
                    + timeThroughLoop, true);
        }
    }

    public void register(StormSubsystemBase... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }
}

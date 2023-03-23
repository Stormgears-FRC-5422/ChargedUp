package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutine {
    public final Command autoCommand;
    public final Pose2d startPose;
    public final String name;

    public AutoRoutine(Command autoCommand,
                       Pose2d startPose,
                       String name) {
        this.autoCommand = autoCommand;
        this.startPose = startPose;
        this.name = name;

        AutoRoutines.autoRoutines.add(this);
    }
}

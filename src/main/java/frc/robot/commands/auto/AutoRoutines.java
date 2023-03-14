package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.autoScoring.AutoScore;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.ArrayList;


public final class AutoRoutines {
    public static ArrayList<AutoCommand> autoCommands = new ArrayList<>();

    public static AutoCommand highConeBumpSide(DrivetrainBase drivetrain) {
        var node = FieldConstants.Grids.getCurrentGrid()[0][0];
        return new AutoCommand(
                new AutoScore(drivetrain, () -> node),
                node.scoringPosition,
                "HIGH CONE BUMP SIDE"
        );
    }
}

package frc.robot.commands.autoScoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pathFollowing.DriveToPose;
import frc.robot.subsystems.drive.DrivetrainBase;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class DriveToScoringNode extends SequentialCommandGroup {

    public DriveToScoringNode(DrivetrainBase drivetrain, ScoringNode node) {
        addCommands(
                new AlignToNode(drivetrain, node),
                new DriveToPose(drivetrain, node.scoringPosition, 0.6, 0.6)
        );
    }
}
package frc.robot.commands.autoScoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.pathFollowing.DriveToPose;
import frc.robot.subsystems.drive.DrivetrainBase;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class AutoScore extends SequentialCommandGroup {

    public AutoScore(DrivetrainBase drivetrain, ScoringNode node) {
        addCommands(
                new AlignToNode(drivetrain, node),
                new ParallelCommandGroup(
                        new DriveToPose(drivetrain, node.scoringPosition, 0.6, 0.6),
                        new PrintCommand("Lift Arm to: " + node.height)
                ),
                new PrintCommand("Drop + " + node.type + " in: " + node.height + ".")
        );
    }
}
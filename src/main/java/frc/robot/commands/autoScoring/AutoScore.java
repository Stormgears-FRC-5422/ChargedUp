package frc.robot.commands.autoScoring;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.pathFollowing.DriveToPose;
import frc.robot.subsystems.drive.DrivetrainBase;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class AutoScore extends SequentialCommandGroup {

    public AutoScore(DrivetrainBase drivetrain, ScoringNode node) {
//        addCommands(
//                new ParallelCommandGroup(
//                        new DriveToNode(drivetrain, node),
//                        new ConditionalCommand(
//                                new PrintCommand("Move arm to score on + " + node.height),
//
//                        )
//                )
//        );
//                new ParallelCommandGroup(
//                        new DriveToPose(drivetrain, node.scoringPosition, 0.6, 0.6),
//                        new PrintCommand("Lift Arm to: " + node.height)
//                )
    }
}
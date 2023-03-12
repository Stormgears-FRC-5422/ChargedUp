package frc.robot.commands.autoScoring;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(DrivetrainBase drivetrain, ScoringNode node) {
        addCommands(
                new ParallelCommandGroup(
                        new DriveToNode(drivetrain, node),
                        new WaitUntilCommand(() -> node.gridRegion.inRegion(RobotState.getInstance().getCurrentPose()))
                                .andThen(new PrintCommand("Moving arm to " + node.height))
                ),
                new PrintCommand("Dropping " + node.type + " in " + node.height + " node")
        );
        addRequirements(drivetrain);
    }
}
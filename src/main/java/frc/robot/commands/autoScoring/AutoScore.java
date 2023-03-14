package frc.robot.commands.autoScoring;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(DrivetrainBase drivetrain, Supplier<ScoringNode> nodeSupplier) {
        addCommands(
                new ParallelCommandGroup(
                        new DriveToNode(drivetrain, nodeSupplier),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() ->
                                        nodeSupplier.get().gridRegion.contains(RobotState.getInstance().getCurrentPose())),
                                new PrintCommand("Moving arm to " + nodeSupplier.get().height))
                ),
                new PrintCommand("Dropping " + nodeSupplier.get().type + " in " + nodeSupplier.get().height + " node")
        );
        addRequirements(drivetrain);
    }

    public AutoScore(DrivetrainBase drivetrain, ScoringNode node) {
        this(drivetrain, () -> node);
    }
}
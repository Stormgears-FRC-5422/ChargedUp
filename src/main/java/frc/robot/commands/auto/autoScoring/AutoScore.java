package frc.robot.commands.auto.autoScoring;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotState;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(DrivetrainBase drivetrain, Arm arm, Compression compression, Supplier<ScoringNode> nodeSupplier) {
        addCommands(
                new ParallelCommandGroup(
                        new DriveToNode(drivetrain, nodeSupplier),
                        new WaitUntilCommand(() ->
                                nodeSupplier.get().
                                        gridRegion.contains(RobotState.getInstance().getCurrentPose()))
                                .andThen(new ArmToNode(arm, nodeSupplier))
                ),
                new WaitCommand(0.5),
                new InstantCommand(compression::release),
                new ArmToTranslation(arm, Constants.ArmConstants.stowPosition, 2, 2)
        );
        addRequirements(drivetrain);
    }
}
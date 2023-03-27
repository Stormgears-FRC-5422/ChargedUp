package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotState;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(DrivetrainBase drivetrain, Arm arm, Compression compression, Supplier<ScoringNode> nodeSupplier) {
        // move the robot to the spot and when we are almost there
        // prepare the arm to run the command
        addCommands(
                compression.getGrabCommand(),
                new ParallelDeadlineGroup(
                        new DriveToNode(drivetrain, nodeSupplier),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> inRange(nodeSupplier)),
                                new ArmToTranslation(arm, () -> getMidwayTranslation(arm, nodeSupplier), 4, 4)
                        )
                ),
                new ArmToNode(arm, nodeSupplier),
                new WaitCommand(0.1),
                compression.getReleaseCommand(),
                new StowArm(arm)
        );
    }

    private Translation2d getMidwayTranslation(Arm arm, Supplier<ScoringNode> nodeSupplier) {
        double goalZ = nodeSupplier.get().translation.getZ();
        Translation3d current = arm.getGlobalTranslation();
        return Arm.fromGlobalTranslation(new Translation3d(current.getX(), 0, goalZ));
    }

    private boolean inRange(Supplier<ScoringNode> nodeSupplier) {
        Pose2d currentPose = RobotState.getInstance().getCurrentPose();
        return nodeSupplier.get().gridRegion.contains(currentPose);
    }
}
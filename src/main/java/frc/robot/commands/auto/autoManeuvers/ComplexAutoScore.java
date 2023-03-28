package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.joysticks.DriveJoystick;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class ComplexAutoScore extends ParallelCommandGroup {

    /** aligns to node with additional driver1 translation input
     * then driver2 presses confirm to place piece */
    public ComplexAutoScore(DrivetrainBase drivetrain, Arm arm, Compression compression,
                            Supplier<ScoringNode> nodeSupplier, DriveJoystick joystick, BooleanSupplier confirm) {

        // when close to node lift arm to get ready to place
        BooleanSupplier readyToPrepArm = () -> inGrid(nodeSupplier) && atRotationTolerance(nodeSupplier);
        Supplier<Translation2d> prepArmTranslation = () -> getPrepArm(arm, nodeSupplier);

        addCommands(
                new AlignToNode(drivetrain, joystick, nodeSupplier), // keep the drive command running
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(confirm), // prep the arm, but start placing when we press confirm
                        new SequentialCommandGroup(
                                new WaitUntilCommand(readyToPrepArm), // prep arm when close
                                new ArmToTranslation(arm, prepArmTranslation, 4, 4)
                        )
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(confirm), // wait until hit confirm to start running
                        new ArmToNode(arm, nodeSupplier),
                        compression.getReleaseCommand(),
                        new StowArm(arm)
                )
        );

    }

    private Boolean inGrid(Supplier<ScoringNode> nodeSupplier) {
        return nodeSupplier.get().gridRegion.contains(RobotState.getInstance().getCurrentPose());
    }

    private boolean atRotationTolerance(Supplier<ScoringNode> nodeSupplier) {
        Rotation2d currentRotation = RobotState.getInstance().getCurrentPose().getRotation();
        double degreeError = MathUtil.inputModulus(
                currentRotation.getDegrees() - nodeSupplier.get().scoringPosition.getRotation().getDegrees(),
                -180, 180);
        return Math.abs(degreeError) <= 40;
    }

    private Translation2d getPrepArm(Arm arm, Supplier<ScoringNode> nodeSupplier) {
        var node = nodeSupplier.get();
        Translation3d globalGripper = arm.getGlobalTranslation();
        if (node.height == ScoringNode.NodeHeight.HYBRID) {
            double xDist = Math.abs(node.translation.getX() - node.scoringPosition.getX());
            return Arm.fromGlobalTranslation(new Translation3d(xDist, 0, globalGripper.getZ()));
        }
        return Arm.fromGlobalTranslation(new Translation3d(globalGripper.getX(), 0, node.translation.getZ() * 0.75));
    }

}
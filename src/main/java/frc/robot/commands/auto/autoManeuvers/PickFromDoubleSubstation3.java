package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.commands.drive.AlignToDoubleSubstation;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.joysticks.DriveJoystick;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.constants.Constants.ArmConstants.pickDoubleSubstationCone;
import static frc.robot.constants.Constants.ArmConstants.pickDoubleSubstationCube;

public class PickFromDoubleSubstation3 extends ParallelCommandGroup {

    public PickFromDoubleSubstation3(DrivetrainBase drivetrain, DriveJoystick joystick, FieldConstants.Side side,
                                     Arm arm, BooleanSupplier pieceDetected, Compression compression) {

        AlignToDoubleSubstation alignCommand = new AlignToDoubleSubstation(drivetrain, joystick, side);

        BooleanSupplier readyForArmUp = () -> atRotationTolerance(() -> alignCommand.getTarget().getRotation());
        BooleanSupplier readyForStow = () -> atXToleranceForStow(() -> alignCommand.getTarget().getTranslation());

        addCommands(
                alignCommand,
                new SequentialCommandGroup(
                        compression.getReleaseCommand(),
                        new WaitUntilCommand(readyForArmUp),
                        new ArmToTranslation(arm, this::getPickingHeight, 4, 4),
                        new WaitUntilCommand(pieceDetected),
                        compression.getGrabCommand(),
                        new WaitUntilCommand(readyForStow),
                        new StowArm(arm)
                )
        );
    }

    private Translation2d getPickingHeight() {
        return (RobotState.getInstance().getLidarRange() == Constants.LidarRange.CONE) ?
                pickDoubleSubstationCone : pickDoubleSubstationCube;
    }

    private boolean atRotationTolerance(Supplier<Rotation2d> rotSetpoint) {
        Rotation2d currentRotation = RobotState.getInstance().getCurrentPose().getRotation();
        // if 30 degrees within aligned
        return Math.abs(Math.abs(currentRotation.getDegrees()) - rotSetpoint.get().getDegrees()) <= 30;
    }

    private boolean atXToleranceForStow(Supplier<Translation2d> translationSetpoint) {
        Translation2d currentTranslation = RobotState.getInstance().getCurrentPose().getTranslation();
        System.out.println(translationSetpoint.get());
        return Math.abs(translationSetpoint.get().getX() - currentTranslation.getX()) >= 0.8;
    }
}

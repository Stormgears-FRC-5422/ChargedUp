package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotState;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.commands.drive.AlignToDoubleSubstation;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.stormnet.StormNet;
import frc.utils.joysticks.DriveJoystick;

import java.util.function.Supplier;

import static frc.robot.constants.Constants.ArmConstants.pickDoubleSubstationCone;
import static frc.robot.constants.Constants.ArmConstants.pickDoubleSubstationCube;

public class PickFromDoubleSubstation2 extends ParallelCommandGroup {

    /** written to be triggered with while true trigger */
    public PickFromDoubleSubstation2(DrivetrainBase drivetrain, Arm arm, Compression compression, StormNet stormNet, NeoPixel neoPixel,
                                     DriveJoystick joystick, FieldConstants.Side side) {
        AlignToDoubleSubstation alignCommand = new AlignToDoubleSubstation(drivetrain, joystick, side);
        addCommands(
                new InstantCommand(() -> neoPixel.setLEDBlinkingState(true)),
                        alignCommand,
                new SequentialCommandGroup(
                        compression.getReleaseCommand(),
                        new WaitUntilCommand(
                                () -> atRotationTolerance(() -> alignCommand.getTarget().getRotation())),
                        new ArmToTranslation(arm, this::getPickingHeight, 4, 4),
                        new ArmToPickUp(arm, stormNet),
                        compression.getGrabCommand(),
                        new StowArm(arm),
                        new InstantCommand(() -> neoPixel.setLEDBlinkingState(false)))
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

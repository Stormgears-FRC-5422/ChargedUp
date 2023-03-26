package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.commands.drive.AlignToDoubleSubstation;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.stormnet.StormNet;
import frc.utils.joysticks.DriveJoystick;

import static frc.robot.constants.Constants.ArmConstants.pickDoubleSubstation;

public class PickFromDoubleSubstation2 extends ParallelCommandGroup {

    /** written to be triggered with while true trigger */
    public PickFromDoubleSubstation2(DrivetrainBase drivetrain, Arm arm, Compression compression, StormNet stormNet,
                                     DriveJoystick joystick, FieldConstants.Side side) {
        addCommands(
                new AlignToDoubleSubstation(drivetrain, joystick, side),
                new SequentialCommandGroup(
                        compression.openGripper(),
                        new WaitUntilCommand(this::atRotationTolerance),
                        new ArmToTranslation(arm, pickDoubleSubstation, 3, 3),
                        new ArmToPickUp(arm, stormNet),
                        compression.closeGripper(),
                        new WaitCommand(0.1),
                        new StowArm(arm)
                )
        );
    }

    private boolean atRotationTolerance() {
        Rotation2d currentRotation = RobotState.getInstance().getCurrentPose().getRotation();
        Rotation2d goalRotation = RobotState.getInstance().getCurrentAlliance() == DriverStation.Alliance.Red ?
                new Rotation2d(2 * Math.PI) : new Rotation2d();
        // if 30 degrees within aligned
        return Math.abs(Math.abs(currentRotation.getDegrees()) - goalRotation.getDegrees()) <= 30;
    }
}

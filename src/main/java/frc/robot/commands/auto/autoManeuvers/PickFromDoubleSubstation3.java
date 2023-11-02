package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.commands.drive.AlignToDoubleSubstation;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.joysticks.DriveJoystick;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.constants.Constants.ArmConstants.pickDoubleSubstationCone;
import static frc.robot.constants.Constants.ArmConstants.pickDoubleSubstationCube;

public class PickFromDoubleSubstation3 extends SequentialCommandGroup {

    public PickFromDoubleSubstation3(Arm arm, BooleanSupplier pieceDetected, Intake intake) {
        addCommands(
                new IntakeCommand(intake, false),
                new ArmToTranslation(arm, this::getPickingHeight, 4, 6),
                new WaitUntilCommand(pieceDetected),
                new IntakeCommand(intake, true),
                new WaitCommand(0.2)
        );
    }

    private Translation2d getPickingHeight() {
        return (RobotState.getInstance().getLidarRange() == Constants.LidarRange.CONE)?
                pickDoubleSubstationCone : pickDoubleSubstationCube;
    }
}

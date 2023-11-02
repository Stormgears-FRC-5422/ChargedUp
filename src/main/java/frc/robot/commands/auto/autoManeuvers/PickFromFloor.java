package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.arm.Arm;

import java.util.function.BooleanSupplier;

public class PickFromFloor extends SequentialCommandGroup {
    public PickFromFloor(Arm arm, Intake intake, BooleanSupplier pieceDetectedSupplier) {
        addCommands(
                new IntakeCommand(intake, false),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(pieceDetectedSupplier),
                        new ArmToTranslation(arm, Constants.ArmConstants.pickGround, 5, 6)
                ),
                new IntakeCommand(intake, true),
                new StowArm(arm)
        );

        addRequirements(arm, intake);
    }
}

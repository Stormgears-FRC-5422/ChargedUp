package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.arm.Arm;

import java.util.function.BooleanSupplier;

public class PickFromSingleSubstation extends SequentialCommandGroup {

    public PickFromSingleSubstation(Arm arm, Intake intake, BooleanSupplier pieceDetected) {
        addCommands(
                new AutoIntakeCommand(intake, true),
                new StowArm(arm),
                new WaitUntilCommand(pieceDetected),
                new AutoIntakeCommand(intake, false),
                new WaitCommand(0.1)
        );

        addRequirements(arm, intake);
    }
}

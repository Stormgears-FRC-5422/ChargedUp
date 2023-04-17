package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;

import java.util.function.BooleanSupplier;

public class PickFromSingleSubstation extends ParallelCommandGroup {

    public PickFromSingleSubstation(Arm arm, Compression compression, BooleanSupplier pieceDetected) {
        addCommands(
                new StowArm(arm),
                new SequentialCommandGroup(
                        compression.getReleaseCommand(),
                        new WaitUntilCommand(pieceDetected),
                        compression.getGrabCommand()
                )
        );
    }
}

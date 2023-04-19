package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;

import java.util.function.BooleanSupplier;

public class PickFromSingleSubstation extends SequentialCommandGroup {

    public PickFromSingleSubstation(Arm arm, Compression compression, BooleanSupplier pieceDetected) {
        addCommands(
                compression.getReleaseCommand(),
                new StowArm(arm),
                new WaitUntilCommand(pieceDetected),
                compression.getGrabCommand(),
                new WaitCommand(0.1)
        );

        addRequirements(arm, compression);
    }
}

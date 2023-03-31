package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.Supplier;

public class DropPieceSequence extends SequentialCommandGroup {
    public DropPieceSequence(Arm arm, Compression compression,
                             Supplier<FieldConstants.Grids.ScoringNode> nodeSupplier) {
        addCommands(
                new ArmToNode(arm, nodeSupplier),
                compression.getReleaseCommand(),
                new StowArm(arm)
        );
    }
}

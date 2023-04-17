package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;

import java.util.function.BooleanSupplier;

public class PickFromFloor extends SequentialCommandGroup {
    public PickFromFloor(Arm arm, Compression compression, BooleanSupplier pieceDetectedSupplier) {
        addCommands(
                new ArmToTranslation(arm, Constants.ArmConstants.pickGround, 3, 2),
                new WaitUntilCommand(pieceDetectedSupplier),
                compression.getGrabCommand(),
                new StowArm(arm)
        );

        addRequirements(arm, compression);
    }
}

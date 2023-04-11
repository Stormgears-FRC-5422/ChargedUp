package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;

public class FloorPickUp extends SequentialCommandGroup {
    public FloorPickUp(Arm arm, Compression compression) {
        addCommands(
                compression.getReleaseCommand(),
                new ArmToTranslation(arm, Constants.ArmConstants.pickGround, 4, 4),
                compression.getGrabCommand(),
                new StowArm(arm),
                new InstantCommand(() -> compression.setOnOffSolenoid(compression.isGripperButtonPosition()))
        );

    }
}

package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.Supplier;

public class PickFromSubstation extends SequentialCommandGroup {

    public PickFromSubstation(DrivetrainBase drivetrain, Arm arm, Compression compression,
                              Supplier<DriveToDoubleSubstation.POSITION> sideSupplier) {
        addCommands(
                new ParallelCommandGroup(
                    new DriveToDoubleSubstation(drivetrain, sideSupplier.get()),
                    new InstantCommand(compression::release),
                    new WaitCommand(0.1).andThen(
                            new ArmToTranslation(arm, Constants.ArmConstants.pickDoubleSubstation, 2, 2))
                )
        );

        addRequirements(drivetrain, arm, compression);
    }
}

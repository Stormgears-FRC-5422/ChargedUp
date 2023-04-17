package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.commands.drive.BalancePitchCommand;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.drive.DrivetrainBase;

public class AutoBalance extends SequentialCommandGroup {
    public AutoBalance(DrivetrainBase drivetrain, NavX navX) {
        addCommands(
                new OverChargeStation(drivetrain, navX::getPitch),
                new BalanceCommand(navX::getPitch, navX::getRoll, drivetrain)
        );
    }
}

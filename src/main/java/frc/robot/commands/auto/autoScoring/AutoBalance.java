package frc.robot.commands.auto.autoScoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.BalanceCommand;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.drive.DrivetrainBase;

public class AutoBalance extends SequentialCommandGroup {
    public AutoBalance(DrivetrainBase drivetrain, NavX navX) {
        addCommands(
                new DriveToChargingStation(drivetrain),
                new BalanceCommand(navX::getPitch, navX::getRoll, drivetrain));
    }
}

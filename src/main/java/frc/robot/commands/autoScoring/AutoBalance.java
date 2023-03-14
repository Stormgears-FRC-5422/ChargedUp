package frc.robot.commands.autoScoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoScoring.DriveToChargingStation;
import frc.robot.commands.drive.BalancePitchCommand;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.DoubleSupplier;

public class AutoBalance extends SequentialCommandGroup {
    public AutoBalance(DrivetrainBase drivetrain, DoubleSupplier pitchSupplier) {
        addCommands(
                new DriveToChargingStation(drivetrain),
                new BalancePitchCommand(drivetrain, pitchSupplier));
    }
}

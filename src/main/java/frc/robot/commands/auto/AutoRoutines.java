package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.autoManeuvers.AutoBalance;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.ArrayList;


public final class AutoRoutines {
    public static ArrayList<AutoCommand> autoCommands = new ArrayList<>();

    public static void initAutoRoutines(DrivetrainBase drivetrain, NavX navX, Arm arm) {
        highConeBumpSideChargingStation(drivetrain, arm, navX);
    }

    public static AutoCommand highConeBumpSideChargingStation(DrivetrainBase drivetrain, Arm arm, NavX navX) {
        var node = FieldConstants.Grids.getGrid()[0][0];
        return new AutoCommand(
                new SequentialCommandGroup(
//                        new AutoScore(drivetrain, arm, node),
                        new AutoBalance(drivetrain, navX)
                ),
                node.scoringPosition,
                "HIGH CONE BUMP SIDE CHARGING STATION"
        );
    }
}

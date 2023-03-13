package frc.robot.commands.auto;

import frc.robot.commands.drive.pathFollowing.PathFollowingCommand;
import frc.robot.subsystems.drive.DrivetrainBase;

public class AutoBalance extends PathFollowingCommand {

    public AutoBalance(DrivetrainBase drivetrain) {
        super(drivetrain);
    }

    @Override
    public void initialize() {

        super.initialize();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.joysticks.StormXboxController;

public class JoyDrive extends CommandBase {

    StormXboxController xboxController;
    DriveSubsystem drive;

    public JoyDrive(DriveSubsystem drive, StormXboxController xboxController) {
        this.xboxController = xboxController;
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        drive.driveArcade(xboxController::getRightJoystickX, xboxController::getLeftJoystickY);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}

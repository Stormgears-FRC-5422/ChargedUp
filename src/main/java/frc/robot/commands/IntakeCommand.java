package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.joysticks.StormXboxController;

public class IntakeCommand extends CommandBase {
    public double intakeSpeed = 0;

    StormXboxController xboxController;
    ShooterSubsystem shooterSubsystem;

    public IntakeCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        intakeSpeed = 0.5;
    }

    @Override
    public void execute() {
        if (!shooterSubsystem.getSensor()) {
            shooterSubsystem.setIntakeSpeed(0);
        } else {
            shooterSubsystem.setIntakeSpeed(intakeSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}

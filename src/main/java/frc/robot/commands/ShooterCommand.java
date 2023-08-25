package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.joysticks.StormXboxController;

public class ShooterCommand extends CommandBase {
    double motorSpeed;
    double intakeSpeed;

    StormXboxController xboxController;
    ShooterSubsystem shooterSubsystem;

    public ShooterCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        motorSpeed = 1;
        intakeSpeed = 0.5;
    }

    @Override
    public void execute() {
        if (shooterSubsystem.getMotorSpeed() >= motorSpeed) {
            shooterSubsystem.setintakemc2(intakeSpeed);
        } else {
            shooterSubsystem.setMotorSpeed(motorSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setintakemc2(0);
        shooterSubsystem.setMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}

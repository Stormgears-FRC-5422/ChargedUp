package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.joysticks.StormXboxController;

public class ShooterCommand extends CommandBase {
    enum shooterState {
        stop,
        prepare,
        start;
    }
    shooterState state;
    double targetSpeed;
    double intakeSpeed;
    double currentSpeed;

    StormXboxController xboxController;
    ShooterSubsystem shooterSubsystem;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, StormXboxController xboxController){
        this.shooterSubsystem = shooterSubsystem;
        this.xboxController = xboxController;



        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        intakeSpeed = 0.5;
        state = shooterState.prepare;
    }

    @Override
    public void execute() {
        double error;
        targetSpeed =0.35;
        currentSpeed = shooterSubsystem.getShooterSpeed() / 5320.00;

        error = targetSpeed - currentSpeed;
        switch (state) {
            case prepare:
                if (currentSpeed > targetSpeed-.05 && currentSpeed < targetSpeed+.05) {

                    state = shooterState.start;
                } else {
                    shooterSubsystem.setShooterSpeed(targetSpeed);
                    shooterSubsystem.setintakemc2(0);
                }
            break;
            case start:
                shooterSubsystem.setintakemc2(intakeSpeed);
                double kP = 1;
                shooterSubsystem.setShooterSpeed(targetSpeed+ kP*error);
                SmartDashboard.putNumber("Error", error);
            break;
        }


        //targetSpeed = xboxController.getRightTrigger();

        //System.out.println("Shooter Speed: " + shooterSubsystem.getMotorSpeed());
        //System.out.println("Trigger Speed: " + motorSpeed);


    }

    @Override
    public void end(boolean interrupted) {
        state = shooterState.stop;
        shooterSubsystem.setintakemc2(0);
        shooterSubsystem.setShooterSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}

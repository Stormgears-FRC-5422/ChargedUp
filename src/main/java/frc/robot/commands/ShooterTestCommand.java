package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.joysticks.StormXboxController;

public class ShooterTestCommand extends CommandBase {
    enum shooterState {
        stop,
        prepare,
        start;
    }
    ShooterCommand.shooterState state;
    double targetSpeed;
    double currentSpeed;

    StormXboxController xboxController;
    ShooterSubsystem shooterSubsystem;
    public ShooterTestCommand(ShooterSubsystem shooterSubsystem, StormXboxController xboxController){
        this.shooterSubsystem = shooterSubsystem;
        this.xboxController = xboxController;

        addRequirements(shooterSubsystem);
    }
    @Override
    public void initialize() {
        targetSpeed = 0.0;
        state = ShooterCommand.shooterState.prepare;
    }


    @Override
    public void execute() {
        targetSpeed = xboxController.getLeftTrigger();
        shooterSubsystem.setShooterSpeed(targetSpeed);



    }
}

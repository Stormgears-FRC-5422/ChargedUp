package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.joysticks.StormXboxController;

import java.sql.SQLOutput;

public class ShooterTestCommand extends CommandBase {
    enum shooterState {
        stop,
        prepare,
        start;
    }
    ShooterCommand.shooterState state;
    double targetSpeed;
    double currentSpeed;
    int counter;
    int seconds;
    double motorspeed;

    StormXboxController xboxController;
    ShooterSubsystem shooterSubsystem;
    public ShooterTestCommand(ShooterSubsystem shooterSubsystem, StormXboxController xboxController){
        this.shooterSubsystem = shooterSubsystem;
        this.xboxController = xboxController;
        addRequirements(shooterSubsystem);
    }
    @Override
    public void initialize() {
        motorspeed = 0;
        targetSpeed = 0.0;
        System.out.println("running initialize");
        counter = 0;

    }


    @Override
    public void execute() {
        counter=counter+1;
        //targetSpeed = xboxController.getLeftTrigger();
        if (counter%150==0) {
            motorspeed = motorspeed + 0.1;


        }
        if (motorspeed>1) {
            motorspeed=0;
        }




        shooterSubsystem.setShooterSpeed(motorspeed);



        currentSpeed = shooterSubsystem.getShooterSpeed();
        SmartDashboard.putNumber("Current Speed", currentSpeed);
        SmartDashboard.putNumber("Target Speed", motorspeed);

        seconds = counter * 20;
        System.out.println(motorspeed + "," + seconds + "," + currentSpeed);

    }
}
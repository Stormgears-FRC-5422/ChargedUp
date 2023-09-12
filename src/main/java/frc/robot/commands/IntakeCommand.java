package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

    Intake intake;


    int count = 0;

    boolean end = false;

    public IntakeCommand(Intake intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intake();
        count++;

        if (count==5){ //change how many repetition of periodic based on testing
            end = true;
            count=0;
            intake.hold();
        }
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}

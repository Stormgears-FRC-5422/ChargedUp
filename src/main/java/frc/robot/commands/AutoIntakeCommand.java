package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;


public class AutoIntakeCommand extends CommandBase {

    Intake intake;


    int count = 0;

    boolean end = false;

    private final double intakeCount = Constants.ArmConstants.intakecount;

    boolean direction;

    public AutoIntakeCommand(Intake intake, boolean direction) {
        this.intake = intake;
        this.direction = direction;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("Intake Command Starting");
        System.out.println("Intake Direction: " + direction);
        count = 0;
        end = false;
    }

    @Override
    public void execute() {


        if (direction) {
            intake.intake();
        } else {
            intake.out();
        }
        if (count==5) {
            end=true;
        }
        count++;
    }

    @Override
    public boolean isFinished() {
        return end;
    }

    @Override
    public void end(boolean interrupted) {
        intake.hold();
        count=0;
    }


}

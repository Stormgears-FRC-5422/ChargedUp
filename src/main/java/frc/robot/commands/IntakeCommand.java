package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

    Intake intake;


    int count = 0;

    boolean end = false;

    private final double intakeCount = Constants.ArmConstants.intakecount;

    boolean direction;

    public IntakeCommand(Intake intake, boolean direction) {
        this.intake = intake;
        this.direction = direction;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (direction) {
            intake.intake();
        } else {
            intake.out();
        }

        count++;

        if (count == intakeCount) { //change how many repetition of periodic based on testing
            end = true;
            count = 0;
            intake.hold();
        }
    }

    @Override
    public boolean isFinished() {
        return end;
    }
}

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmJointSpeeds;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class BasicArm extends CommandBase {
    Arm arm;
    DoubleSupplier shoulderOmegaSupplier;
    DoubleSupplier elbowOmegaSupplier;

    public BasicArm(Arm arm,
                    DoubleSupplier shoulderOmegaSupplier,
                    DoubleSupplier elbowOmegaSupplier) {

        this.arm = arm;
        this.shoulderOmegaSupplier = shoulderOmegaSupplier;
        this.elbowOmegaSupplier = elbowOmegaSupplier;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        System.out.println("BasicArm Command Initialize!");
        super.initialize();
    }

    @Override
    public void execute() {
        arm.setSpeedScale(kArmSpeedScale);
        arm.percentOutMoveArm(new ArmJointSpeeds(shoulderOmegaSupplier.getAsDouble(), elbowOmegaSupplier.getAsDouble()));
    }

    @Override
    public boolean isFinished() {
        return (false);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("BasicArm Command done!");
        // TODO - maybe this should be conditional - we don't want to undo what the interrupting command is trying to do.
        // but for now this is safer, and I don't think we'll keep this around long term.
        arm.stopArm();
    }

}

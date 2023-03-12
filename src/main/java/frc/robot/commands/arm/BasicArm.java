package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmJointSpeeds;
import java.util.function.DoubleSupplier;

import static frc.robot.constants.Constants.*;

public class BasicArm extends ArmCommand {
    Arm arm;
    DoubleSupplier dAphaSupplier;
    DoubleSupplier dBetaSupplier;

    public BasicArm(Arm arm,
                    DoubleSupplier dAphaSupplier,
                    DoubleSupplier dBetaSupplier) {

        this.arm = arm;
        this.dAphaSupplier = dAphaSupplier;
        this.dBetaSupplier = dBetaSupplier;

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
        arm.percentOutMoveArm(new ArmJointSpeeds(dAphaSupplier.getAsDouble(), dBetaSupplier.getAsDouble()));
    }

    @Override
    public boolean isFinished() {
        return (false);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("BasicArm Command done!");
        arm.stopArm();
    }

}

package frc.robot.commands.arm;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.arm.Arm;

import java.util.function.DoubleSupplier;

import static frc.robot.constants.Constants.kXYArmSpeedScale;

public class XYArm extends ArmCommand {
    Arm arm;
    DoubleSupplier XSpeedSupplier;
    DoubleSupplier YSpeedSupplier;

    public XYArm(Arm arm,
                 DoubleSupplier XSpeedSupplier,
                 DoubleSupplier YSpeedSupplier) {

        this.arm = arm;
        this.XSpeedSupplier = XSpeedSupplier;
        this.YSpeedSupplier = YSpeedSupplier;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        System.out.println("XYArm Command Initialize!");
        super.initialize();
    }

    @Override
    public void execute() {
        arm.setSpeedScale(kXYArmSpeedScale);
        arm.percentOutXYMoveArm(new ChassisSpeeds(XSpeedSupplier.getAsDouble(),
                                                  YSpeedSupplier.getAsDouble(),
                               0.0 ));
    }

    @Override
    public boolean isFinished() {
        return (false);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("XYArm Command done!");
        arm.stopArm();
    }

}

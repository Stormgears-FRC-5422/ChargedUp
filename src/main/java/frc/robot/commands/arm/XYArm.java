package frc.robot.commands.arm;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmJointSpeeds;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.kArmSpeedScale;

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
        arm.setSpeedScale(kArmSpeedScale);
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
        // TODO - maybe this should be conditional - we don't want to undo what the interrupting command is trying to do.
        // but for now this is safer, and I don't think we'll keep this around long term.
        arm.stopArm();
    }

}

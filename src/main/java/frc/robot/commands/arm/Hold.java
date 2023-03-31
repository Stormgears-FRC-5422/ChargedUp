package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import static frc.robot.constants.Constants.kXYArmManualSpeed;
import java.util.function.Supplier;

public class Hold extends CommandBase{

    Arm arm;
    Pose2d gripperPose;
    private final Supplier<Translation2d> goalSupplier;

    double dy;
    double dx;
    double yTarget;
    double xTarget;
    double y;
    double x;

    public Hold (Arm arm, Supplier<Translation2d> goalSupplier) {
        this.arm = arm;
        this.goalSupplier = goalSupplier;
    } 

    @Override
    public void initialize() {
        Translation2d goal = goalSupplier.get();  
        yTarget = goal.getY();
        xTarget = goal.getX();
        dx = 0;
        dy = 0;
    }

    @Override
    public void execute () {
        gripperPose = arm.getGripperPose();
        y = gripperPose.getY();
        x = gripperPose.getX();

        System.out.println("HOLD RUNNING");

        if ((y < yTarget - 0.005) || (dy > 0 && y < yTarget)) {
            dy = kXYArmManualSpeed;
        } else if ((y > yTarget + 0.005) || (dy > 0 && y < yTarget)) {
            dy = -kXYArmManualSpeed;
        } else {
            dy =0;
        }

        if ((x < xTarget - 0.005) || (dx > 0 && x < xTarget)) {
            dx = kXYArmManualSpeed;
        } else if ((x > xTarget + 0.005) || (dx > 0 && x < xTarget)) {
            dx = -kXYArmManualSpeed;
        } else {
            dx = 0;
        }
        arm.xyMoveArm(new ChassisSpeeds(dx, dy, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopArm();
    }
}

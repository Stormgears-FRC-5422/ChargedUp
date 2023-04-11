package frc.robot.commands.auto.autoManeuvers;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.DoubleSupplier;

import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

public class OverChargeStation extends CommandBase {

    private DrivetrainBase drivetrainBase;

    private DoubleSupplier getPitch;

    private final int tolerance = 5;
    private final int negTolerance = 5;



    public OverChargeStation(DrivetrainBase drivetrainBase, DoubleSupplier getPitch) {
        this.drivetrainBase = drivetrainBase;
        this.getPitch = getPitch;

        addRequirements(drivetrainBase);
    }

    @Override
    public void initialize() {
        System.out.println("Running OverChargeStation Command");
    }

    @Override
    public void execute() {
        while (getPitch.getAsDouble() > negTolerance) {
            //unsure about speeds
            drivetrainBase.drive(new ChassisSpeeds(-2, 0.0, 0.0), false);
        }
        while (getPitch.getAsDouble() < tolerance) {
            drivetrainBase.drive(new ChassisSpeeds(-5, 0.0, 0.0), false);
        }
//        flip from one side to other
        while (getPitch.getAsDouble() > tolerance) {
            drivetrainBase.drive(new ChassisSpeeds(-5, 0.0, 0.0), false);
        }


//        NEED TO ADD DRIVE FORWARD A BIT using pathpoint
        while (getPitch.getAsDouble() < tolerance) {
            drivetrainBase.drive(new ChassisSpeeds(5, 0.0, 0.0), false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainBase.drive(new ChassisSpeeds(0, 0.0, 0.0), false);
    }
}

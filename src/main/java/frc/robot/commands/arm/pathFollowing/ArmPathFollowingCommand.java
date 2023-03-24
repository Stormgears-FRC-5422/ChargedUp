package frc.robot.commands.arm.pathFollowing;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.arm.Arm;

public class ArmPathFollowingCommand extends CommandBase {

    private final Arm arm;
    private PathPlannerTrajectory path;

    private double startTime, currentTime;
    private int count = 0;

    private final GenericEntry dTranslationEntry = ShuffleboardConstants.getInstance().dTranslationEntryArm;

    private final PIDController xController = new PIDController(3.5, 0.1, 0.0);
    private final PIDController yController = new PIDController(3.5, 0.1, 0.0);
    private PPHolonomicDriveController controller = new PPHolonomicDriveController(
            xController, yController,
            new PIDController(0, 0, 0));

    private Translation2d currentTranslation, goalTranslation;

    private Timer timer = new Timer();

    public ArmPathFollowingCommand(Arm arm) {
        this.arm = arm;

        // always in tolerance for rotation
        controller.setTolerance(new Pose2d(0.01, 0.01, new Rotation2d(2 * Math.PI)));
//        xController.setIntegratorRange(0.01, 0.03);
//        yController.setIntegratorRange(0.01, 0.03);

        addRequirements(arm);
    }

    public ArmPathFollowingCommand(Arm arm, PathPlannerTrajectory path) {
        this(arm);
        this.path = path;
    }

    public ArmPathFollowingCommand addPath(PathPlannerTrajectory path) {
        this.path = path;
        return this;
    }

    @Override
    public void initialize() {
        if (path == null) {
            DriverStation.reportWarning(
                    "Arm Path Following Command cannot run without path", true);
            path = new PathPlannerTrajectory();
            return;
        }

        timer.reset();
        timer.start();

        currentTranslation = arm.getGripperPose().getTranslation();
        goalTranslation = path.getEndState().poseMeters.getTranslation();

        System.out.println("Starting Arm Path Following Command at: " + startTime);
        System.out.println("Position: " + arm.getGripperPose());
    }

    @Override
    public void execute() {
        currentTime = timer.get();
        currentTranslation = arm.getGripperPose().getTranslation();

        var setpoint = (PathPlannerTrajectory.PathPlannerState) path.sample(currentTime);

        dTranslationEntry
                .setDouble(currentTranslation.getDistance(setpoint.poseMeters.getTranslation()));


        ChassisSpeeds outputSpeeds = controller.calculate(
                new Pose2d(currentTranslation, new Rotation2d(0)),
                setpoint);

//        if (count++ % 25 == 0) {
//            System.out.println("*****");
//            System.out.println("time: " + currentTime);
//            System.out.println("current: " + currentTranslation);
//            System.out.println("--");
//            System.out.println("setpoint: " + setpoint.poseMeters.getTranslation());
//            System.out.println("dX: " + outputSpeeds.vxMetersPerSecond +
//                    ", dY: " + outputSpeeds.vyMetersPerSecond + " omega: " + outputSpeeds.omegaRadiansPerSecond);
//            System.out.println("*****");
//        }
        arm.xyMoveArm(new ChassisSpeeds(outputSpeeds.vxMetersPerSecond, outputSpeeds.vyMetersPerSecond, 0));
    }

    @Override
    public boolean isFinished() {
//        System.out.println(controller.atReference());
        return (currentTime >= path.getTotalTimeSeconds() && controller.atReference()) ||
                currentTime >= path.getTotalTimeSeconds() + 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending Arm Path Following Command at: " + RobotState.getInstance().getTimeSeconds());
        System.out.println();
        arm.stopArm();
    }
}

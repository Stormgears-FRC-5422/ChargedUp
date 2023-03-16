package frc.robot.commands.arm;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

public class ArmTrajectoryToPose extends CommandBase {

    private Arm arm;
    private Translation2d goalTranslation;
    private PathPlannerTrajectory path;

    private Pose2d currentPose, lastPose;
    private Rotation2d heading;
    private Timer timer = new Timer();
    private double currentTime, totalTime;

    private PPHolonomicDriveController controller = new PPHolonomicDriveController(
        new PIDController(0.5, 0, 0),
        new PIDController(1.0, 0, 0),
        new PIDController(0, 0, 0));

    public ArmTrajectoryToPose(Arm arm, Translation2d goalTranslation) {
        this.arm = arm;
        this.goalTranslation = goalTranslation;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
//        System.out.println("Starting trajectory command: " + goalTranslation);
        var startTranslation = arm.getGripperPose().getTranslation();
//        System.out.println("Starting Translation: " + startTranslation);
//        System.out.println("Goal Translation: " + goalTranslation);
        double initialHeading = calcHeading(startTranslation, goalTranslation);
        heading = Rotation2d.fromRadians(initialHeading);

        path = PathPlanner.generatePath(
            new PathConstraints(0.5, 0.2),
            new PathPoint(startTranslation, new Rotation2d(initialHeading)),
            new PathPoint(goalTranslation, new Rotation2d(initialHeading)));
        System.out.println("Start Translation: " + path.getInitialHolonomicPose().getTranslation());
        System.out.println("Middle Translation: " + path.getState(path.getStates().size() / 2).poseMeters.getTranslation());
        System.out.println("End Translation: " + path.getEndState().poseMeters.getTranslation());
        timer.start();
        totalTime = path.getTotalTimeSeconds();
    }

    @Override
    public void execute() {
        currentTime = timer.get();
        System.out.println(currentTime);

        var setpoint = (PathPlannerTrajectory.PathPlannerState) path.sample(currentTime);
        System.out.println(setpoint.poseMeters.getTranslation());
        currentPose = new Pose2d(arm.getGripperPose().getTranslation(), heading);
        ChassisSpeeds output = controller.calculate(currentPose, setpoint);
        SmartDashboard.putNumber("X output", output.vxMetersPerSecond);
        SmartDashboard.putNumber("Y output", output.vyMetersPerSecond);
        arm.xyMoveArm(
                new ChassisSpeeds(output.vxMetersPerSecond, output.vyMetersPerSecond, 0));
        // System.out.println(output);
//        heading = new Rotation2d(output.vxMetersPerSecond, output.vyMetersPerSecond);
//        System.out.println("heading: " + heading.getDegrees());
    }

    @Override
    public boolean isFinished() {
        return (currentTime >= totalTime) &&
                (currentPose.getTranslation().getDistance(goalTranslation) <= 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            arm.stopArm();}
        System.out.println("Arm follow trajectory command ended after: " + currentTime + " seconds");
    }
}
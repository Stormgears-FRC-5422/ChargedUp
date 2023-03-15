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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

public class ArmTrajectoryToPose extends CommandBase {

    private Arm arm;
    private Translation2d goalTranslation;
    private PathPlannerTrajectory path;

    private Pose2d currentPose;
    private Timer timer = new Timer();
    private double currentTime, totalTime;

    private PPHolonomicDriveController controller = new PPHolonomicDriveController(
        new PIDController(3, 0, 0),
        new PIDController(3, 0, 0),
        new PIDController(0, 0, 0));

    public ArmTrajectoryToPose(Arm arm, Translation2d goalTranslation) {
        this.arm = arm;
        this.goalTranslation = goalTranslation;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        var startTranslation = arm.getGripperPose().getTranslation();
        double initialHeading = calcHeading(startTranslation, goalTranslation);
        double endHeading = calcHeading(goalTranslation, startTranslation);

        path = PathPlanner.generatePath(
            new PathConstraints(1, 0.2),
            new PathPoint(startTranslation, new Rotation2d(initialHeading)),
            new PathPoint(goalTranslation, new Rotation2d(endHeading)));

        timer.start();
        totalTime = path.getTotalTimeSeconds();
    }

    @Override
    public void execute() {
        currentTime = timer.get();

        var setpoint = (PathPlannerTrajectory.PathPlannerState) path.sample(currentTime);

        currentPose = arm.getGripperPose();
        ChassisSpeeds output = controller.calculate(currentPose, setpoint);
        arm.xyMoveArm(new ChassisSpeeds(output.vxMetersPerSecond, output.vyMetersPerSecond, 0));
        // System.out.println(output);
    }

    @Override
    public boolean isFinished() {
        return (currentTime >= totalTime) && (currentPose.getTranslation().getDistance(goalTranslation) <= 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            arm.stopArm();}
        System.out.println("Arm follow trajectory command ended after: " + currentTime + " seconds");
    }

    private double calcHeading(Translation2d startTranslation, Translation2d endTranslation) {
        var aboutStart = endTranslation.minus(startTranslation);
        return Math.atan2(aboutStart.getY(), aboutStart.getX());
    }
}
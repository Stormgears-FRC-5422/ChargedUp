package frc.robot.commands.drive.pathFollowing;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.drive.DrivetrainBase;

/**
 * Command that follows a path. path must be assigned before execute either with
 * withPath() method or in constructor. Use this whenever following any paths for logging.
 */
public class PathFollowingCommand extends CommandBase {

    private final DrivetrainBase m_drivetrain;
    private PathPlannerTrajectory m_path;
    private boolean transformForAlliance = false;

    private Pose2d currentPose, endPose;

    private double startTime, currentTime, totalTime;

    private final Field2d fieldSim;
    private final FieldObject2d goalRobotPoseSim;
    private final GenericEntry dTranslationEntry, dRotationEntry;

    private static final double TRANSLATION_TOLERANCE_METERS = 0.025;
    private static final double ROTATION_TOLERANCE_DEGREES = 1.2;

    public PathFollowingCommand(DrivetrainBase drivetrain, PathPlannerTrajectory path, boolean transformForAlliance) {
        m_drivetrain = drivetrain;
        m_path = path;
        this.transformForAlliance = transformForAlliance;

        fieldSim = ShuffleboardConstants.getInstance().pathFollowingFieldSim;
        goalRobotPoseSim = fieldSim.getObject("Goal Pose");
        dTranslationEntry = ShuffleboardConstants.getInstance().dTranslationEntry;
        dRotationEntry = ShuffleboardConstants.getInstance().dRotationEntry;

        addRequirements(m_drivetrain);
    }

    public PathFollowingCommand(DrivetrainBase drivetrain) {
        m_drivetrain = drivetrain;

        fieldSim = ShuffleboardConstants.getInstance().pathFollowingFieldSim;
        goalRobotPoseSim = fieldSim.getObject("Goal Pose");
        dTranslationEntry = ShuffleboardConstants.getInstance().dTranslationEntry;
        dRotationEntry = ShuffleboardConstants.getInstance().dRotationEntry;

        addRequirements(m_drivetrain);
    }

    /** must be set in constructor or here before command schedule */
    public PathFollowingCommand withPath(PathPlannerTrajectory path) {
        m_path = path;
        return this;
    }

    public PathFollowingCommand withTransformForAlliance(boolean transformForAlliance) {
        this.transformForAlliance = transformForAlliance;
        return this;
    }

    @Override
    public void initialize() {
        m_path = transformForAlliance?
                PathPlannerTrajectory.transformTrajectoryForAlliance(m_path, DriverStation.getAlliance()) :
                m_path;

        currentPose = RobotState.getInstance().getCurrentPose();
        endPose = new Pose2d(
                m_path.getEndState().poseMeters.getTranslation(),
                m_path.getEndState().holonomicRotation
        );
        startTime = RobotState.getInstance().getTimeSeconds();
        totalTime = m_path.getTotalTimeSeconds();
        System.out.println("Following path starting at: " + startTime);
        System.out.println("Pose at start: " + currentPose + " to: " + endPose);

        fieldSim.getRobotObject().setTrajectory(m_path);
    }

    @Override
    public void execute() {
        currentTime = RobotState.getInstance().getTimeSeconds() - startTime;
        var goalState = (PathPlannerTrajectory.PathPlannerState) m_path.sample(currentTime);
        var currentPose = RobotState.getInstance().getCurrentPose();
        //Path Planner states are different to trajectory states
        var goalPose = new Pose2d(
                goalState.poseMeters.getTranslation(),
                goalState.holonomicRotation);

        //log error
        dTranslationEntry.setDouble(currentPose.getTranslation().getDistance(goalPose.getTranslation()));
        dRotationEntry.setDouble(currentPose.getRotation().minus(goalPose.getRotation()).getDegrees());
        //put it on field???
        fieldSim.setRobotPose(currentPose);
        goalRobotPoseSim.setPose(goalPose);

        m_drivetrain.goToPPTrajectoryState(goalState);
    }

    @Override
    public boolean isFinished() {
        return (currentTime >= totalTime) && m_drivetrain.atReferenceState() &&
                (currentPose.getTranslation().getDistance(endPose.getTranslation()) <= TRANSLATION_TOLERANCE_METERS) &&
                (Math.abs(currentPose.getRotation().getDegrees() - endPose.getRotation().getDegrees()) <= ROTATION_TOLERANCE_DEGREES);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Following path command ended at: " + RobotState.getInstance().getTimeSeconds());
        System.out.println("Pose at End: " + RobotState.getInstance().getCurrentPose());
        if (!interrupted)
            m_drivetrain.stopDrive();
    }
}

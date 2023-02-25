package frc.robot.commands.trajectory;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

public class FollowPathCommand extends CommandBase {

    private final PathPlannerTrajectory m_path;
    private final DrivetrainBase m_drivetrain;

    private double startTime, currentTime, endTime;

    //for logging
    private final GenericEntry dTranslationEntry;
    private final GenericEntry dRotationEntry;
    private final Field2d field = new Field2d();
    private final FieldObject2d goalRobotPose;

    public FollowPathCommand(PathPlannerTrajectory path, DrivetrainBase drivetrain) {
        m_path = path;
        m_drivetrain = drivetrain;

        ShuffleboardTab pathFollowTab = Shuffleboard.getTab("Path Following");
        dTranslationEntry = pathFollowTab.add("dTranslation", 0.0).getEntry();
        dRotationEntry = pathFollowTab.add("dRotation", 0.0).getEntry();
        pathFollowTab.add("Current Pose vs Goal", field)
                .withWidget(BuiltInWidgets.kField)
                .withSize(4, 3).withPosition(0, 1);
        goalRobotPose = field.getObject("Goal Pose");

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        startTime = RobotState.getInstance().getTimeSeconds();
        endTime = m_path.getTotalTimeSeconds();
        System.out.println("Following path starting at: " + startTime);
        System.out.println("Pose at start: " + RobotState.getInstance().getCurrentPose());
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
        field.setRobotPose(currentPose);
        goalRobotPose.setPose(goalPose);

        m_drivetrain.goToPPTrajectoryState(goalState);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ending with interrupted: " + interrupted);
        System.out.println("Following path command ended at: " + RobotState.getInstance().getTimeSeconds());
        System.out.println("Pose at End: " + RobotState.getInstance().getCurrentPose());
    }

    @Override
    public boolean isFinished() {
        return currentTime >= endTime;
    }
}

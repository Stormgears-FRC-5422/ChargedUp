package frc.robot.commands.trajectory;

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

public class FollowPathCommand extends CommandBase {

    private PathPlannerTrajectory m_path;
    private final DrivetrainBase m_drivetrain;

    private double startTime, currentTime, endTime;

    //for logging
//    private static final ShuffleboardTab pathFollowTab = Shuffleboard.getTab("Path Following");
//    private static final GenericEntry dTranslationEntry = pathFollowTab.add("dTranslation", 0.0).getEntry();
//    private static final GenericEntry dRotationEntry = pathFollowTab.add("dRotation", 0.0).getEntry();
//    private static final Field2d fieldSim = new Field2d();
//
//    static {
//        pathFollowTab.add("Current Pose vs Goal", fieldSim)
//                .withWidget(BuiltInWidgets.kField)
//                .withSize(4, 3).withPosition(1, 0);
//    }
//
//    private static final FieldObject2d goalRobotPoseSim = fieldSim.getObject("Goal Pose");


    private final Field2d fieldSim;
    private final FieldObject2d goalRobotPoseSim;
    private final GenericEntry dTranslationEntry, dRotationEntry;

    public FollowPathCommand(PathPlannerTrajectory path, DrivetrainBase drivetrain, boolean useAlliance) {
        m_path = path;
        if (useAlliance)
            m_path = PathPlannerTrajectory.transformTrajectoryForAlliance(m_path, DriverStation.getAlliance());
        m_drivetrain = drivetrain;

        //Logging
        fieldSim = ShuffleboardConstants.getInstance().pathFollowingFieldSim;
        dTranslationEntry = ShuffleboardConstants.getInstance().dTranslationEntry;
        dRotationEntry = ShuffleboardConstants.getInstance().dRotationEntry;
        goalRobotPoseSim = fieldSim.getObject("Goal Pose");

        addRequirements(m_drivetrain);
    }

    public FollowPathCommand(PathPlannerTrajectory path, DrivetrainBase drivetrain) {
        this(path, drivetrain, false);
    }

    @Override
    public void initialize() {
        startTime = RobotState.getInstance().getTimeSeconds();
        endTime = m_path.getTotalTimeSeconds();
        System.out.println("Following path starting at: " + startTime);
        System.out.println("Pose at start: " + RobotState.getInstance().getCurrentPose());

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
        System.out.println(goalPose);

        //log error
        dTranslationEntry.setDouble(currentPose.getTranslation().getDistance(goalPose.getTranslation()));
        dRotationEntry.setDouble(currentPose.getRotation().minus(goalPose.getRotation()).getDegrees());
        //put it on field???
        fieldSim.setRobotPose(currentPose);
        goalRobotPoseSim.setPose(goalPose);

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

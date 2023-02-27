package frc.robot.constants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class ShuffleboardConstants {
    public ShuffleboardTab robotStateTab = Shuffleboard.getTab("Robot State");
    public ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
    public ShuffleboardTab navXTab = Shuffleboard.getTab("Nav X");
    public ShuffleboardTab pathFollowingTab = Shuffleboard.getTab("Path Following");
    public ShuffleboardLayout pathFollowingList = pathFollowingTab
            .getLayout("Following Command", BuiltInLayouts.kList)
            .withPosition(0, 0).withSize(2, 4);
    public GenericEntry dTranslationEntry = pathFollowingList.add("dTranslation", 0.0).getEntry();
    public GenericEntry dRotationEntry = pathFollowingList.add("dRotation", 0.0).getEntry();
    public Field2d pathFollowingFieldSim = new Field2d();
    public Field2d poseEstimationFieldSim = new Field2d();

    private static ShuffleboardConstants instance;

    public static ShuffleboardConstants getInstance() {
        if (instance != null) return instance;
        instance = new ShuffleboardConstants();
        return instance;
    }

    private ShuffleboardConstants() {
        pathFollowingTab
                .add("Goal v. Current Pose", pathFollowingFieldSim)
                .withWidget(BuiltInWidgets.kField)
                .withSize(7, 4).withPosition(2, 0);

        ShuffleboardTab poseEstimationTab = Shuffleboard.getTab("Pose Estimation");
        poseEstimationTab
                .add("Pose Estimation", poseEstimationFieldSim)
                .withWidget(BuiltInWidgets.kField)
                .withSize(7, 4).withPosition(0, 0);
    }
}

package frc.robot.constants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class ShuffleboardConstants {
    public final ShuffleboardTab robotStateTab, drivetrainTab, navXTab, pathFollowingTab;
    public final Field2d poseEstimationFieldSim;

    // Shuffleboard stuff for path following
    public final ShuffleboardLayout pathFollowingList;
    public final GenericEntry dTranslationEntry, dRotationEntry;
    public final Field2d pathFollowingFieldSim;

    private static ShuffleboardConstants instance;

    public static ShuffleboardConstants getInstance() {
        if (instance != null) return instance;
        instance = new ShuffleboardConstants();
        return instance;
    }

    private ShuffleboardConstants() {
        robotStateTab = Shuffleboard.getTab("Robot State");
        drivetrainTab = Shuffleboard.getTab("Drivetrain");
        navXTab = Shuffleboard.getTab("Nav X");
        pathFollowingTab = Shuffleboard.getTab("Path Following");

        pathFollowingList = pathFollowingTab
                .getLayout("Following Command", BuiltInLayouts.kList)
                .withPosition(0, 0).withSize(2, 2);

        dTranslationEntry = pathFollowingList.add("dTranslation", 0.0).getEntry();
        dRotationEntry = pathFollowingList.add("dRotation", 0.0).getEntry();

        pathFollowingFieldSim = new Field2d();
        poseEstimationFieldSim = new Field2d();

        pathFollowingTab
                .add("Goal v. Current Pose", pathFollowingFieldSim)
                .withWidget(BuiltInWidgets.kField)
                .withSize(7, 4).withPosition(2, 0);

        Shuffleboard.getTab("Pose Estimation")
                .add("Poses", poseEstimationFieldSim)
                .withWidget(BuiltInWidgets.kField)
                .withSize(7, 4).withPosition(0, 0);
    }
}
package frc.robot.constants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.Map;

import static frc.robot.constants.Constants.Toggles.*;

public final class ShuffleboardConstants {
    public ShuffleboardTab robotStateTab, drivetrainTab, navXTab,
            pathFollowingTab, driverTab, armTab, preRoundTab;
    public Field2d poseEstimationFieldSim;

    // Shuffleboard stuff for path following
    public ShuffleboardLayout robotStateList, pathFollowingList, gridLayout,
    armStatusLayout, armPathFollowingList, autoSelectionLayout;
    public GenericEntry dTranslationEntry, dRotationEntry, dTranslationEntryArm;
    public GenericEntry alignToNodeIndicator, alignToSubstationIndicator;
    public Field2d pathFollowingFieldSim;

    private static ShuffleboardConstants instance;

    public static ShuffleboardConstants getInstance() {
        if (instance != null) return instance;
        instance = new ShuffleboardConstants();
        return instance;
    }

    private ShuffleboardConstants() {
        robotStateTab = Shuffleboard.getTab("Robot State");
        drivetrainTab = Shuffleboard.getTab("Drivetrain");
        navXTab = Shuffleboard.getTab("Gyro");
        pathFollowingTab = Shuffleboard.getTab("Path Following");
        armTab = Shuffleboard.getTab("Arm");
        driverTab = Shuffleboard.getTab("Driver");
        preRoundTab = Shuffleboard.getTab("Pre Round");

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

//        robotStateTab
//                .add("Field Sim", poseEstimationFieldSim)
//                .withWidget(BuiltInWidgets.kField)
//                .withSize(7, 4).withPosition(2, 0);

        driverTab
                .add("Field Sim", poseEstimationFieldSim)
                .withWidget(BuiltInWidgets.kField)
                .withSize(4, 2).withPosition(4, 0);

        robotStateList = robotStateTab
                .getLayout("State", BuiltInLayouts.kList)
                .withPosition(0, 0)
                .withSize(2, 4);

        gridLayout = driverTab
                .getLayout("Node Selector", BuiltInLayouts.kGrid)
                .withProperties(Map.of(
                        "Label position", "HIDDEN",
                        "Number of columns", 9,
                        "Number of rows", 3))
                .withPosition(0, 0).withSize(4, 2);

        armStatusLayout = armTab
                .getLayout("Arm Status", BuiltInLayouts.kGrid)
                .withProperties(Map.of(
                        "Number of columns", 2,
                        "Number of rows", 4))
                .withPosition(0, 0).withSize(2, 5);

        armPathFollowingList = armTab
                .getLayout("Arm Path Following", BuiltInLayouts.kList)
                .withPosition(4, 0).withSize(2, 5);

        autoSelectionLayout = preRoundTab
                .getLayout("Auto Selector", BuiltInLayouts.kList)
                .withPosition(0, 0).withSize(2, 5);

        ShuffleboardLayout checklist = preRoundTab
                .getLayout("Checks", BuiltInLayouts.kList)
                .withPosition(2, 0).withSize(2, 5);

        checklist.addBoolean("Drive", () -> useDrive);
        checklist.addBoolean("Arm", () -> useArm);
        checklist.addBoolean("Pneumatics", () -> usePneumatics);
        checklist.addBoolean("Vision", () -> useVision);
        checklist.addBoolean("Lidar", () -> useStormNet);
        checklist.addBoolean("Lights", () -> useStatusLights);
        checklist.addBoolean("Joystick", () -> useFirstXboxController || useLogitechController);
        checklist.addBoolean("Buttonboard", () -> useButtonBoard);

        dTranslationEntryArm = armPathFollowingList
                .add("dTranslation", 0.0).getEntry();

        alignToNodeIndicator = driverTab
                .add("Scoring", false)
                .withPosition(8, 0).withSize(1, 1)
                .getEntry();
        alignToSubstationIndicator = driverTab
                .add("Substation", false)
                .withPosition(8, 1).withSize(1, 1)
                .getEntry();
    }
}
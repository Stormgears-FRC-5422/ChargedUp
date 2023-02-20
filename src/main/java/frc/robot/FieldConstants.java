package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;
import java.util.List;


public final class FieldConstants {
    public static double FIELD_WIDTH = Units.feetToMeters(54) + Units.inchesToMeters(3.25);
    public static double FIELD_LENGTH = Units.feetToMeters(26) + Units.inchesToMeters(3.5);
    public static boolean debug = false;

    public static AprilTagFieldLayout APRILTAG_FIELD_LAYOUT;
    static {
        try {
            APRILTAG_FIELD_LAYOUT = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            if (!debug && DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                APRILTAG_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
            } else if (!debug && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                APRILTAG_FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
            } else {
                if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                    Pose3d origin_tag = new Pose3d(1.02743, 1.071626, 0.462788, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))); // April Tag 8
                    APRILTAG_FIELD_LAYOUT.setOrigin(origin_tag);
                } else {
                    Pose3d origin_tag = new Pose3d(15.513558, 1.071626, 0.462788, new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))); // April Tag 1
                    APRILTAG_FIELD_LAYOUT.setOrigin(origin_tag);
                }
            }
        } catch (IOException e) {
            System.out.println("Could not load april tag field layout! with message " + e);
            throw new RuntimeException(e);
        }
    }
    public static List<AprilTag> APRILTAG_POSITIONS = APRILTAG_FIELD_LAYOUT.getTags();
}
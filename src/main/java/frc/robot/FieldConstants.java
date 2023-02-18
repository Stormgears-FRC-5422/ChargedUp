package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;
import java.util.List;

public final class FieldConstants {
    public static double FIELD_WIDTH = Units.feetToMeters(54) + Units.inchesToMeters(3.25);
    public static double FIELD_LENGTH = Units.feetToMeters(26) + Units.inchesToMeters(3.5);

    public static AprilTagFieldLayout APRILTAG_FIELD_LAYOUT;
    static {
        try {
            APRILTAG_FIELD_LAYOUT = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            System.out.println("Could not load april tag field layout! with message " + e);
            throw new RuntimeException(e);
        }
    }

    public static List<AprilTag> APRILTAGS = APRILTAG_FIELD_LAYOUT.getTags();
}

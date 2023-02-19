package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.data.StormStruct;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Optional;
import java.util.Vector;

import static frc.robot.FieldConstants.APRILTAG_FIELD_LAYOUT;

public class Vision extends SubsystemBase {
    private final StormStruct storm_struct;
    private final Vector<HashMap<String, Double>> all_april_tags = new Vector<>();
    private Vector<HashMap<String, Double>> info;

    public Pose3d robot_position = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    public Vision() {
        NetworkTableInstance nt_inst = NetworkTableInstance.getDefault();
        storm_struct = new StormStruct(nt_inst, "vision-data", "tag_data");
    }

    private void initializeAprilTags() {
        for (double id = 1; id <= 8.0; id++) {
            HashMap<String, Double> april_tag = new HashMap<>();
            april_tag.put("id", id);
            april_tag.put("distance", 0.0);
            april_tag.put("roll", 0.0);
            april_tag.put("yaw", 0.0);
            april_tag.put("pitch", 0.0);
            april_tag.put("off-center", 0.0);
            all_april_tags.add(april_tag);
        }
    }

    private void updateAprilTagMap() {
        for (HashMap<String, Double> present_april_tag : info) {
            for (HashMap<String, Double> april_tag : all_april_tags) {
                if (present_april_tag.get("id").equals(april_tag.get("id"))) {
                    april_tag.put("distance", present_april_tag.get("distance"));
                    april_tag.put("roll", present_april_tag.get("roll"));
                    april_tag.put("pitch", present_april_tag.get("pitch"));
                    if (april_tag.containsKey("off-center")) { // REMOVE THIS LINE AFTER ADDING IT TO PYTHON SIDE
                        april_tag.put("off-center", present_april_tag.get("off-center"));
                    }
                    break;
                } else {
                    april_tag.put("distance", 0.0);
                    april_tag.put("roll", 0.0);
                    april_tag.put("pitch", 0.0);
                    april_tag.put("off-center", 0.0);
                }
            }
        }
    }

    @Override
    public void periodic() {
        storm_struct.intialize();
        initializeAprilTags();
        // Temporary Testing Code
        info = storm_struct.get_data("april_tag");
        updateAprilTagMap();
        SmartDashboard.putString("Info: ", info.toString());

        displayAprilTagInfo();

        double[] position_array = new double[8];
        if (info.size() >= 2) {
            double[] closestAprilTags = getClosestAprilTags();
            position_array = triangulatePosition(closestAprilTags[0], closestAprilTags[1]);
            displayPosition(position_array);
        }

        updateRobotPosition(position_array);
    }

    public double[] triangulatePosition(double id1, double id2) {
        double dist1, dist2;
        double dist_between_markers;
        try {
            dist1 = getAprilTagInfo("id", id1, "distance");
            dist2 = getAprilTagInfo("id", id2, "distance");
            dist_between_markers = getDistBetweenMarkers(id1, id2);
        } catch (IllegalArgumentException e) {
            throw new RuntimeException(e.toString());
        }
        // double tag1_angle = Math.toDegrees(Math.acos((Math.pow(dist_between_markers, 2) + Math.pow(dist1, 2) - Math.pow(dist2, 2)) / (2 * dist_between_markers * dist1)));
        double tag1_angle = getAngle(dist2, dist_between_markers, dist1);
        double x_tag1 = Math.cos(Math.toRadians(tag1_angle)) * dist1;
        double y_tag1 = Math.sin(Math.toRadians(tag1_angle)) * dist1;

        // double tag2_angle = Math.toDegrees(Math.acos((Math.pow(dist_between_markers, 2) + Math.pow(dist2, 2) - Math.pow(dist1, 2)) / (2 * dist_between_markers * dist2)));
        double tag2_angle = getAngle(dist1, dist_between_markers, dist2);
        double x_tag2 = Math.cos(Math.toRadians(tag2_angle)) * dist2;
        double y_tag2 = Math.sin(Math.toRadians(tag2_angle)) * dist2;

        return new double[]{id1, tag1_angle, x_tag1, y_tag1, id2, tag2_angle, x_tag2, y_tag2};
    }

    public void updateRobotPosition(double[] position_array) {
        Translation3d robot_translation = new Translation3d();
        Optional<Pose3d> april_tag = APRILTAG_FIELD_LAYOUT.getTagPose((int)position_array[0]);
        if (april_tag.isPresent()) {
            robot_translation = new Translation3d(april_tag.get().getX() + position_array[2], april_tag.get().getY() + position_array[3], april_tag.get().getZ());
        }
        Rotation3d robot_rotation = new Rotation3d(0.0, 0.0, position_array[1]); // TODO: Need to implement NavX and off-center angle
        robot_position = robot_position.relativeTo(new Pose3d(robot_translation, robot_rotation));
        SmartDashboard.putString("Robot Position: ", robot_position.toString());
    }

    private double[] getClosestAprilTags() {
        if (info.size() > 2) {
            double[] largest_april_tag = new double[]{info.get(0).get("id"), info.get(0).get("distance")};
            double[] second_largest_april_tag = new double[]{info.get(1).get("id"), info.get(1).get("distance")};
            for (HashMap<String, Double> april_tag : info) {
                if (april_tag.get("distance") >= largest_april_tag[1]) {
                    second_largest_april_tag = largest_april_tag;
                    largest_april_tag = new double[]{april_tag.get("id"), april_tag.get("distance")};
                } else if (april_tag.get("distance") >= second_largest_april_tag[1]) {
                    second_largest_april_tag = new double[]{april_tag.get("id"), april_tag.get("distance")};
                }
            }
            return new double[]{largest_april_tag[0], second_largest_april_tag[0]};
        } else {
            return new double[]{info.get(0).get("id"), info.get(1).get("id")};
        }
    }

    private double getDistBetweenMarkers(double id1, double id2) throws IllegalArgumentException {
        if (APRILTAG_FIELD_LAYOUT.getTagPose((int) id1).isPresent() && APRILTAG_FIELD_LAYOUT.getTagPose((int) id2).isPresent()) {
            return Math.abs(APRILTAG_FIELD_LAYOUT.getTagPose((int) id1).get().getX() - APRILTAG_FIELD_LAYOUT.getTagPose((int) id2).get().getX());
        }
        throw new IllegalArgumentException("One/both of the IDs provided do not exist.");
    }

    private double getAngle(double a, double b, double c) {
        return Math.toDegrees(Math.acos((Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2)) / (2 * b * c)));
    }

    private double getAprilTagInfo(String query_key, double query_value, String return_key) throws IllegalArgumentException {
        for (HashMap<String, Double> present_april_tag : all_april_tags) {
            if (present_april_tag.containsKey(query_key) && present_april_tag.get(query_key) == query_value) {
                return present_april_tag.get(return_key);
            }
        }
        throw new IllegalArgumentException("Could not find key from in the Vectors.");
    }

    private void displayPosition(double[] position_array) {
        SmartDashboard.putString("Displaying Position Array", Arrays.toString(position_array));
        SmartDashboard.putNumber("AprilTag " + position_array[0] + " Angle", position_array[1]);
        SmartDashboard.putNumber("AprilTag " + position_array[0] + " X Distance", position_array[2]);
        SmartDashboard.putNumber("AprilTag " + position_array[0] + " Y Distance", position_array[3]);
        SmartDashboard.putNumber("AprilTag " + position_array[4] + " Angle", position_array[5]);
        SmartDashboard.putNumber("AprilTag " + position_array[4] + " X Distance", position_array[6]);
        SmartDashboard.putNumber("AprilTag " + position_array[4] + " Y Distance", position_array[7]);
    }

    private void displayAprilTagInfo() {
        for (HashMap<String, Double> april_tag : all_april_tags) {
            if (april_tag.get("distance") != 0.0) {
                SmartDashboard.putNumber("AprilTag " + april_tag.get("id") + " Distance", april_tag.get("distance"));
                SmartDashboard.putNumber("AprilTag " + april_tag.get("id") + " Roll", april_tag.get("roll"));
                SmartDashboard.putNumber("AprilTag " + april_tag.get("id") + " Yaw", april_tag.get("yaw"));
                SmartDashboard.putNumber("AprilTag " + april_tag.get("id") + " Pitch", april_tag.get("pitch"));
            }
        }
    }
}

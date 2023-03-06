package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.constants.FieldConstants.FIELD_LENGTH;
import static frc.robot.constants.FieldConstants.getTagPose;
import static frc.robot.subsystems.vision.Vision.AprilTagData;
import static frc.robot.constants.Constants.VisionConstants.*;

import java.util.Vector;

//TODO: see if yaw angles reported by vision can be used
public final class AprilTagPoseEstimationStrategy {

    /** @return camera pose from list of april tag data */
    public static Pose2d fromAprilTagData(Vector<AprilTagData> dataList, Rotation2d cameraAngle) {
        if (dataList.size() < 1) {
            System.out.println("Gave data list of nothing to April tag pose estimator!!!");
            return CAMERA_POSITION.toPose2d();
        }

        if (dataList.size() == 1) return fromOneAprilTag(dataList.get(0), cameraAngle);
        if (dataList.size() == 2) return triangulateFromAprilTags(dataList.get(0), dataList.get(1), cameraAngle);

        //TODO: there is def a quicker way to do this
        AprilTagData closest = null;
        double closestDist = Double.MAX_VALUE;
        for (var data : dataList) {
            if (data.dist < closestDist) {
                closest = data;
                closestDist = data.dist;
            }
        }
        dataList.remove(closest);
        AprilTagData secondClosest = null;
        closestDist = Double.MAX_VALUE;
        for (var data : dataList) {
            if (data.dist < closestDist) {
                secondClosest = data;
                closestDist = data.dist;
            }
        }
        if (closest == null || secondClosest == null) return CAMERA_POSITION.toPose2d();
        return triangulateFromAprilTags(closest, secondClosest, cameraAngle);
    }

    /** gives camera pose based on two tags and camera angle */
    private static Pose2d fromOneAprilTag(AprilTagData data, Rotation2d cameraAngle) {
        //shift gyro over 90 so when its positive when looking at blue and negative when looking at red
        double cameraAngleDeg = cameraAngle.rotateBy(Rotation2d.fromDegrees(90)).getDegrees();
        // get angle made by hypot. (distance) and adjacent leg (wpiY)
        // all assuming that when april tag is left of center the off center is negative
        // and that when it is in right it is positive
        // this all works out so that the transformations are the correct sign
        double theta = cameraAngleDeg + data.offCenterDegrees;
        double hypotLength = data.dist;
        // in terms of field coordinates wpi
        double xDist = Math.cos(Math.toRadians(theta)) * hypotLength;
        double yDist = Math.sin(Math.toRadians(theta)) * hypotLength;

        var aprilTagPose = getTagPose(data.id);
        return new Pose2d(
                aprilTagPose.getX() + xDist,
                aprilTagPose.getY() + yDist,
                cameraAngle);
    }

    /** gives camera pose based on two tags and camera angle <br>
     * give from tags left to right */
    private static Pose2d triangulateFromAprilTags(AprilTagData tag1, AprilTagData tag2, Rotation2d cameraAngle) {
        // TODO: see if this works for all types of configurations of tags (red side tags as well)
        //  drive the robot so closer tags switch see if its being added on to pose correctly
        var closerTagData = (tag1.dist <= tag2.dist)? tag1 : tag2;
        var fartherTagData = (tag1.dist <= tag2.dist)? tag2 : tag1;
        // grab pose2d objects for tags
        var closerTagTranslation = getTagPose(closerTagData.id).getTranslation();
        var fartherTagTranslation = getTagPose(fartherTagData.id).getTranslation();
        double distBetween = closerTagTranslation.getDistance(fartherTagTranslation);
        // calculate transformations
        double angleMadeByCloseTagAndWall = getAngle(fartherTagData.dist, distBetween, closerTagData.dist);
        double xTransform = Math.sin(angleMadeByCloseTagAndWall) * closerTagData.dist;
        double yTransform = Math.cos(angleMadeByCloseTagAndWall) * closerTagData.dist;
        // should add or subtract transformations based on their position on the field
        // if wpi X is less than the other one then we should add otherwise we should subtract
        double signX = (closerTagTranslation.getX() < fartherTagTranslation.getX())? 1.0 : -1.0;
        // if it is on the red side of the field then we should subtract the y transformation
        double signY = (closerTagTranslation.getY() < FIELD_LENGTH / 2.0)? 1.0 : -1.0;
        // add the transforms onto the closer tag
        return new Pose2d(
                closerTagTranslation.getX() + xTransform * signX,
                closerTagTranslation.getY() + yTransform * signY,
                cameraAngle
        );
    }

    /** @return angle in radians opposite side a given triangle lengths <br>
     * (c^2 + b^2 - a^2)/(2bc) --> A */
    private static double getAngle(double a, double b, double c) {
        double x = (b*b + c*c - a*a) / (2.0 * b * c);
        return Math.acos(x);
    }
}

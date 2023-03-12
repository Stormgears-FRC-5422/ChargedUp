package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.constants.FieldConstants.FIELD_LENGTH;
import static frc.robot.constants.FieldConstants.getTagPose;
import static frc.robot.subsystems.vision.Vision.AprilTagData;
import static frc.robot.constants.Constants.VisionConstants.*;

import java.util.Comparator;
import java.util.Vector;

//TODO: see if yaw angles reported by vision can be used
public final class AprilTagPoseEstimationStrategy {

    /** @return camera pose from list of april tag data */
    public static Pose2d fromAprilTagData(Vector<AprilTagData> data, Rotation2d camAngle) {
        if (data.size() == 0) {
            DriverStation.reportWarning("Can't ask pose estimation strategy with no data!", true);
            return new Pose2d();
        }

        boolean useYaw = data.get(0).dist <= kAprilTagYawTrustMeters;
        if (data.size() == 1) {
            if (useYaw) return _oneAprilTagYaw(data.get(0));
            return _oneAprilTagCameraAngle(data.get(0), camAngle);
        } else {
            data.sort(Comparator.comparingDouble(tag -> tag.dist));
            var closer = data.get(0);
            var farther = data.get(1);
            Translation2d translation = _triangulateTranslation(data.get(0), data.get(1));
            if (useYaw) {
                var tagRotation = getTagPose(closer.id).getRotation().toRotation2d();
                Rotation2d camRotation = _getAngleFromYaw(tagRotation, closer.yawDegrees);
                return new Pose2d(translation, camRotation);
            }
            return new Pose2d(translation, camAngle);
        }
    }

//    public static Pose2d fromAprilTagData(Vector<AprilTagData> dataList, Rotation2d cameraAngle) {
//        if (dataList.size() < 1) {
//            System.out.println("Gave data list of nothing to April tag pose estimator!!!");
//            return CAMERA_POSITION.toPose2d();
//        }
//        if (dataList.size() == 1) return _oneAprilTagCameraAngle(dataList.get(0), cameraAngle);
//        if (dataList.size() == 2) return new Pose2d(
//                _triangulateTranslation(dataList.get(0), dataList.get(1)),
//                cameraAngle);
//
//        //TODO: there is def a quicker way to do this
//        AprilTagData closest = null;
//        double closestDist = Double.MAX_VALUE;
//        for (var data : dataList) {
//            if (data.dist < closestDist) {
//                closest = data;
//                closestDist = data.dist;
//            }
//        }
//        dataList.remove(closest);
//        AprilTagData secondClosest = null;
//        closestDist = Double.MAX_VALUE;
//        for (var data : dataList) {
//            if (data.dist < closestDist) {
//                secondClosest = data;
//                closestDist = data.dist;
//            }
//        }
//        if (closest == null || secondClosest == null) return CAMERA_POSITION.toPose2d();
//        return new Pose2d(
//                _triangulateTranslation(closest, secondClosest),
//                cameraAngle
//        );
//    }

    /** get camera pose from one tag data */
    private static Pose2d _oneAprilTagYaw(AprilTagData tag) {
        // get x transform and y transform known angle made by camera and apriltag
        double xTransform = Math.sin(Math.toRadians(tag.yawDegrees)) * tag.dist;
        double yTransform = Math.cos(Math.toRadians(tag.yawDegrees)) * tag.dist;
        // add transforms
        var tagPose = getTagPose(tag.id).toPose2d();
        return new Pose2d(
                tagPose.getX() + xTransform,
                tagPose.getY() + yTransform,
                _getAngleFromYaw(tagPose.getRotation(), tag.yawDegrees)
        );
    }

    /** gives camera pose based on one tag and camera angle derived from gyro */
    private static Pose2d _oneAprilTagCameraAngle(AprilTagData tag, Rotation2d cameraAngle) {
        // shift gyro over 90 so when its positive when looking at blue and negative when looking at red
        double cameraAngleDeg = cameraAngle.rotateBy(Rotation2d.fromDegrees(90)).getDegrees();
        // get angle made by hypot. (distance) and adjacent leg (wpiY)
        // all assuming that when april tag is left of center the off center is negative
        // and that when it is in right it is positive
        // this all works out so that the transformations are the correct sign
        double angleTagAndWall = cameraAngleDeg + tag.offCenterDegrees;
        // in terms of field coordinates wpi
        double dist = _get2dDist(tag.id, tag.dist);
        double xTransform = Math.cos(Math.toRadians(angleTagAndWall)) * dist;
        double yTransform = Math.sin(Math.toRadians(angleTagAndWall)) * dist;

        var aprilTagPose = getTagPose(tag.id);
        return new Pose2d(
                aprilTagPose.getX() + xTransform,
                aprilTagPose.getY() + yTransform,
                cameraAngle);
    }

    /** only get translation of robot from tag distances */
    private static Translation2d _triangulateTranslation(AprilTagData closerTag, AprilTagData fartherTag) {
        // grab pose2d objects for tags
        var closerTagTranslation = getTagPose(closerTag.id).getTranslation().toTranslation2d();
        var fartherTagTranslation = getTagPose(fartherTag.id).getTranslation().toTranslation2d();
        double distBetween = closerTagTranslation.getDistance(fartherTagTranslation);
        // get distances in 2d
        double closerDist = _get2dDist(closerTag.id, closerTag.dist);
        double fartherDist = _get2dDist(fartherTag.id, fartherTag.dist);
        // calculate transformations
        double angleMadeByCloseTagAndWall = _getAngleFromTriangle(fartherDist, distBetween, closerDist);
        double xTransform = Math.sin(angleMadeByCloseTagAndWall) * closerDist;
        double yTransform = Math.cos(angleMadeByCloseTagAndWall) * closerDist;
        // should add or subtract transformations based on their position on the field
        // if wpi X is less than the other one then we should add otherwise we should subtract
        double signX = (closerTagTranslation.getX() < fartherTagTranslation.getX())? 1.0 : -1.0;
        // if it is on the red side of the field then we should subtract the y transformation
        double signY = (closerTagTranslation.getY() < FIELD_LENGTH / 2.0)? 1.0 : -1.0;
        // add the transforms onto the closer tag
        return new Translation2d(
                closerTagTranslation.getX() + xTransform * signX,
                closerTagTranslation.getY() + yTransform * signY
        );
    }

    private static Rotation2d _getAngleFromYaw(Rotation2d tagAngle, double yaw) {
        return Rotation2d.fromDegrees(yaw - tagAngle.getDegrees());
    }

    /** get distance to tag in 2d field positions */
    private static double _get2dDist(int tagID, double dist3d) {
        double heightDiff = Math.abs(CAMERA_POSITION.getZ() - getTagPose(tagID).getZ());
        return (dist3d * dist3d) - (heightDiff * heightDiff);
    }

    /** @return angle in radians opposite side a given triangle lengths <br>
     * (c^2 + b^2 - a^2)/(2bc) --> A */
    private static double _getAngleFromTriangle(double a, double b, double c) {
        double x = (b*b + c*c - a*a) / (2.0 * b * c);
        return Math.acos(x);
    }
}

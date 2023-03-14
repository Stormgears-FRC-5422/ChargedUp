package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.constants.FieldConstants.FIELD_LENGTH;
import static frc.robot.constants.FieldConstants.getTagPose;
import static frc.robot.subsystems.vision.Vision.AprilTagData;
import static frc.robot.constants.Constants.VisionConstants.*;

import java.util.Comparator;
import java.util.HashMap;
import java.util.Vector;

//TODO: see if yaw angles reported by vision can be used
public final class AprilTagPoseEstimationStrategy {

    private static int logCounter = 0;

    private static HashMap<Integer, Double> heightDiffs = new HashMap<>();
    static {
        for (int i = 1; i <= 8; i++) {
            heightDiffs.put(i, Math.abs(getTagPose(i).getZ() - CAMERA_POSITION.getZ()));
        }
    }

    /** @return camera pose from list of april tag data */
    public static Pose2d fromAprilTagData(Vector<AprilTagData> data, Rotation2d camAngle) {
        if (data.size() == 0) {
            DriverStation.reportWarning("Can't ask pose estimation strategy with no data!", true);
            return new Pose2d();
        }
        if (data.size() > 1)
            data.sort(Comparator.comparingDouble(tag -> tag.dist));

        AprilTagData closest = data.get(0);
        boolean useYaw = closest.dist <= kAprilTagYawTrustMeters;
        Pose3d closeTag = getTagPose(closest.id);
        
        // rotation is either the state as is or new one based on yaw
        Rotation2d rotation = camAngle;
        if (useYaw)
            rotation = _getCamAngleFromYaw(closeTag.getRotation().toRotation2d(), closest.yawDegrees);
        
        Translation2d translation;
        if (data.size() > 2) {
            AprilTagData secondClosest = data.get(1);
            translation = _twoAprilTags(closest, secondClosest);
            // if both tags are in range then we can average their yaw values for rotation
            if (useYaw && secondClosest.dist <= kAprilTagYawTrustMeters) {
                var secondCloseTagRotation = getTagPose(secondClosest.id).getRotation().toRotation2d();
                Rotation2d secondRotation = _getCamAngleFromYaw(secondCloseTagRotation, secondClosest.yawDegrees);
                double averageRotation = (secondRotation.getDegrees() + rotation.getDegrees()) / 2.0;
                rotation = Rotation2d.fromDegrees(averageRotation);
            }
        } else {
            double dist2d = _get2dDist(closest.id, closest.dist);
            if (useYaw)
                translation = _oneAprilTag(closest.yawDegrees, closest.offCenterDegrees, closeTag.toPose2d(), dist2d);
            else {
                double yaw = _getYawFromCamAngle(camAngle, closeTag.getRotation().toRotation2d());
                translation = _oneAprilTag(yaw, closest.offCenterDegrees, closeTag.toPose2d(), dist2d);
            }
        }

        return new Pose2d(translation, rotation);
    }

//    /** get camera pose from one tag data */
//    private static Pose2d _oneAprilTagYaw(AprilTagData tag) {
//        // get x transform and y transform known angle made by camera and apriltag
//        double xTransform = Math.sin(Math.toRadians(tag.yawDegrees)) * tag.dist;
//        double yTransform = Math.cos(Math.toRadians(tag.yawDegrees)) * tag.dist;
//        // add transforms
//        var tagPose = getTagPose(tag.id).toPose2d();
//        return new Pose2d(
//                tagPose.getX() + xTransform,
//                tagPose.getY() + yTransform,
//                _getCamAngleFromYaw(tagPose.getRotation(), tag.yawDegrees)
//        );
//    }

    /** gives camera pose based on one tag and camera angle derived from gyro */
//    private static Pose2d _oneAprilTagCameraAngle(AprilTagData tag, Rotation2d cameraAngle) {
//        // shift gyro over 90 so when its positive when looking at blue and negative when looking at red
//        double shiftedAngle = cameraAngle.rotateBy(Rotation2d.fromDegrees(90)).getDegrees();
//        // get angle made by hypot. (distance) and adjacent leg (wpiY)
//        // all assuming that when april tag is left of center the off center is negative
//        // and that when it is in right it is positive
//        // this all works out so that the transformations are the correct sign
//        double angleTagAndWall = shiftedAngle + tag.offCenterDegrees;
//        System.out.println("theta: " + angleTagAndWall);
//        // in terms of field coordinates wpi
//        double dist = _get2dDist(tag.id, tag.dist);
//        double xTransform = Math.cos(Math.toRadians(angleTagAndWall)) * -dist;
//        double yTransform = Math.sin(Math.toRadians(angleTagAndWall)) * -dist;
//
//        var aprilTagPose = getTagPose(tag.id);
//        return new Pose2d(
//                aprilTagPose.getX() + xTransform,
//                aprilTagPose.getY() + yTransform,
//                cameraAngle);
//    }

    private static Translation2d _oneAprilTag(double yawFromTag, double offset, Pose2d tagPose, double dist2d) {
        double theta = -offset + (yawFromTag + tagPose.getRotation().getDegrees());
        double xTransform = Math.cos(Math.toRadians(theta)) * dist2d;
        double yTransform = Math.sin(Math.toRadians(theta)) * dist2d;

        return new Translation2d(
                tagPose.getX() + xTransform,
                tagPose.getY() + yTransform
        );
    }

    /** only get translation of robot from tag distances */
    private static Translation2d _twoAprilTags(AprilTagData closerTag, AprilTagData fartherTag) {
        // grab pose2d objects for tags
        var closerTagTranslation = getTagPose(closerTag.id).getTranslation().toTranslation2d();
        var fartherTagTranslation = getTagPose(fartherTag.id).getTranslation().toTranslation2d();
        double distBetween = closerTagTranslation.getDistance(fartherTagTranslation);
        // get distances in 2d
        double closerDist = _get2dDist(closerTag.id, closerTag.dist);
        double fartherDist = _get2dDist(fartherTag.id, fartherTag.dist);
        // calculate transformations
        double angleMadeByCloseTagAndWall = _getAngleFromTriangle(closerDist, distBetween, fartherDist);
        double xTransform = Math.sin(angleMadeByCloseTagAndWall) * closerDist;
        double yTransform = Math.cos(angleMadeByCloseTagAndWall) * closerDist;
        // should add or subtract transformations based on their position on the field
        // if wpi y is less than the other one then we should add otherwise we should subtract
        double signY = (closerTagTranslation.getY() < fartherTagTranslation.getY())? 1.0 : -1.0;
        // if it is on the red side of the field then we should subtract the x transformation
        double signX = (closerTagTranslation.getX() < FIELD_LENGTH / 2.0)? 1.0 : -1.0;
        // add the transforms onto the closer tag
        return new Translation2d(
                closerTagTranslation.getX() + xTransform * signX,
                closerTagTranslation.getY() + yTransform * signY
        );
    }

    /** get supposed yaw angle of tag based on current cam angle */
    private static double _getYawFromCamAngle(Rotation2d camAngle, Rotation2d tagAngle) {
        return (camAngle.getDegrees() - (180.0 - tagAngle.getDegrees()));
    }

    /** get angle of camera from yaw value */
    private static Rotation2d _getCamAngleFromYaw(Rotation2d tagAngle, double yaw) {
        return Rotation2d.fromDegrees(yaw + tagAngle.getDegrees() - 180.0);
    }

    /** get distance to tag in 2d field positions */
    private static double _get2dDist(int tagID, double dist3d) {
        double heightDiff = heightDiffs.get(tagID);
        return Math.sqrt((dist3d*dist3d) - (heightDiff*heightDiff));
    }

//    private static double _get2dDist(Pose3d tagPose, double dist3d) {
//        double heightDiff = Math.abs(CAMERA_POSITION.getZ() - tagPose.getZ());
//        return Math.sqrt((dist3d * dist3d) - (heightDiff * heightDiff));
//    }

    /** @return angle in radians made by a and b given side lengths <br>
     * (a^2 + b^2 - c^2)/(2ab) --> A */
    private static double _getAngleFromTriangle(double a, double b, double c) {
        double x = (a*a + b*b - c*c) / (2.0 * a * b);
        return Math.acos(x);
    }
}

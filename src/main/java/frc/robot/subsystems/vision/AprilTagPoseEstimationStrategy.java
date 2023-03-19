package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.*;
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

    private static HashMap<Integer, Pose3d> tagPoses = new HashMap<>();

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
        Pose3d closeTag = _getTagPose(closest.id);
        
        // rotation is either the state as is or new one based on yaw
        Rotation2d rotation = camAngle;
        if (useYaw)
            rotation = _getCamAngleFromYaw(closeTag.getRotation().toRotation2d(), closest.yawDegrees);
        
        Translation2d translation;
        if (data.size() > 1) {
            System.out.println("Using triangulation method");
            AprilTagData secondClosest = data.get(1);
            translation = _twoAprilTags(closest, secondClosest);
            // if both tags are in range then we can average their yaw values for rotation
            if (useYaw && secondClosest.dist <= kAprilTagYawTrustMeters) {
                var secondCloseTagRotation = _getTagPose(secondClosest.id).getRotation().toRotation2d();
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
        var closerTagTranslation = _getTagPose(closerTag.id).getTranslation().toTranslation2d();
        var fartherTagTranslation = _getTagPose(fartherTag.id).getTranslation().toTranslation2d();
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

    private static Pose3d _getTagPose(int tagID) {
        if (tagPoses.containsKey(tagID))
            return tagPoses.get(tagID);
        var pose = getTagPose(tagID);
        tagPoses.put(tagID, pose);
        return pose;
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
        double heightDiff;
        if (heightDiffs.containsKey(tagID))
            return heightDiffs.get(tagID);
        heightDiff = Math.abs(_getTagPose(tagID).getZ() - CAMERA_POSITION.getZ());
        heightDiffs.put(tagID, heightDiff);
        return Math.sqrt((dist3d*dist3d) - (heightDiff*heightDiff));
    }

    /** @return angle in radians made by a and b given side lengths <br>
     * (a^2 + b^2 - c^2)/(2ab) --> A */
    private static double _getAngleFromTriangle(double a, double b, double c) {
        double x = (a*a + b*b - c*c) / (2.0 * a * b);
        return Math.acos(x);
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.utils.configfile.StormProp;

public final class Constants {

    public static final String robotName = StormProp.getString("robotName", "");

    public static final double kStickNullSize = StormProp.getNumber("StickNullSize", 0.1);
    public static final int kLogitechControllerPort = StormProp.getInt("LogitechControllerPort", -1);
    public static final double kTemperatureRampThreshold = StormProp.getNumber("SparkMaxTemperatureRampThreshold", 45.0);
    public static final double kTemperatureRampLimit = StormProp.getNumber("SparkMaxTemperatureRampLimit", 60.0);
    public static final double kSparkMaxCurrentLimit = StormProp.getNumber("SparkMaxCurrentLimit", 35.0);
    public static final double kSparkMaxCurrentLimit550 = StormProp.getNumber("SparkMax550CurrentLimit", 25.0);

    public static final double kNeoFreeSpeedRPM = StormProp.getNumber("SparkMaxFreeSpeedRPM", 0.0);
    public static final double kDriveSpeedScale = StormProp.getNumber("driveSpeedScale", 0.0);
    public static final double kPrecisionSpeedScale = StormProp.getNumber("precisionSpeedScale", 0.0);
    public static final double kArmSpeedScale = StormProp.getNumber("angleArmSpeedScale", 0.0);
    public static final double kXYArmSpeedScale = StormProp.getNumber("xyArmSpeedScale", 0.0);
    public static final double kXYArmManualSpeed = StormProp.getNumber("xyArmManualSpeed", 0.0);
    public static final int frontLeftDriveID = StormProp.getInt("frontLeftDriveID", 0);
    public static final int frontRightDriveID = StormProp.getInt("frontRightDriveID", 0);
    public static final int backLeftDriveID = StormProp.getInt("backLeftDriveID", 0);
    public static final int backRightDriveID = StormProp.getInt("backRightDriveID", 0);

    public static final int frontLeftSwivelID = StormProp.getInt("frontLeftSwivelID", 0);
    public static final int frontRightSwivelID = StormProp.getInt("frontRightSwivelID", 0);
    public static final int backLeftSwivelID = StormProp.getInt("backLeftSwivelID", 0);
    public static final int backRightSwivelID = StormProp.getInt("backRightSwivelID", 0);

    public static final int frontLeftEncoderID = StormProp.getInt("frontLeftEncoderID", 0);
    public static final int frontRightEncoderID = StormProp.getInt("frontRightEncoderID", 0);
    public static final int backLeftEncoderID = StormProp.getInt("backLeftEncoderID", 0);
    public static final int backRightEncoderID = StormProp.getInt("backRightEncoderID", 0);

    public static final int kSwivelEncoderTicksPerRotation = StormProp.getInt("swivelEncoderTicksPerRotation", 0);
    public static final int frontLeftOffsetTicks = StormProp.getInt("frontLeftOffsetTicks", 0);
    public static final int frontRightOffsetTicks = StormProp.getInt("frontRightOffsetTicks", 0);
    public static final int backLeftOffsetTicks = StormProp.getInt("backLeftOffsetTicks", 0);
    public static final int backRightOffsetTicks = StormProp.getInt("backRightOffsetTicks", 0);

    public static final double TURN_RADIUS = Math.sqrt(
            Math.pow(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, 2) + Math.pow(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0, 2));
    public static final double ROBOT_WIDTH = StormProp.getNumber("robotWidth", 0.);
    public static final double ROBOT_LENGTH = StormProp.getNumber("robotLength", 0.);
    public static final double BUMPER_THICKNESS = StormProp.getNumber("bumperThickness", 0.);

    // <mecanum> Not needed for swerve, just mecanum
    public static final double kWheelRadiusMeters = StormProp.getNumber("wheelRadiusMeters", 0.);
    public static final double kWheelMaxRPM = StormProp.getNumber("wheelMaxRPM", 0.);
    // </mecanum>

    // Map to SDS implementation constants
    public static final class DriveConstants {
        public static final String kMK4iModuleKind = StormProp.getString("mk4iModuleKind", "");
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = StormProp.getNumber("drivetrainTrackwidthMeters", 0.);
        public static final double DRIVETRAIN_WHEELBASE_METERS = StormProp.getNumber("drivetrainWheelbaseMeters", 0.);
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = frontLeftDriveID;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = frontLeftSwivelID;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = frontLeftEncoderID;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(360.0 * frontLeftOffsetTicks / kSwivelEncoderTicksPerRotation);
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = frontRightDriveID;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = frontRightSwivelID;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = frontRightEncoderID;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(360.0 * frontRightOffsetTicks / kSwivelEncoderTicksPerRotation);
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = backLeftDriveID;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = backLeftSwivelID;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = backLeftEncoderID;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(360.0 * backLeftOffsetTicks / kSwivelEncoderTicksPerRotation);
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = backRightDriveID;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = backRightSwivelID;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = backRightEncoderID;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(360.0 * backRightOffsetTicks / kSwivelEncoderTicksPerRotation);

        public static final String driveType = StormProp.getString("driveType", "");

        public static final double driveXkp = StormProp.getNumber("driveXkp", 1.0);
        public static final double driveXki = StormProp.getNumber("driveXki", 0.0);
        public static final double driveXkd = StormProp.getNumber("driveXkd", 0.0);
        public static final double driveYkp = StormProp.getNumber("driveYkp", 1.0);
        public static final double driveYki = StormProp.getNumber("driveYki", 0.0);
        public static final double driveYkd = StormProp.getNumber("driveYkd", 0.0);
        public static final double turnkp = StormProp.getNumber("turnkp", 1.0);
        public static final double turnki = StormProp.getNumber("turnki", 0.0);
        public static final double turnkd = StormProp.getNumber("turnkd", 0.0);
    }

    public static final int kCompressorModuleId = StormProp.getInt("CompressorModuleId", -1);
    public static final int onOffSolenoidChannel = StormProp.getInt("onOffSolenoidChannel", -1);
    public static final int cubeConeSolenoidChannel = StormProp.getInt("cubeConeSolenoidChannel", -1);
    public static final int kMagEncoderTicksPerRotation = StormProp.getInt("magEncoderTicksPerRotation", 0);
    public static final String navXConnection = StormProp.getString("navXConnection", "");

    public enum LidarRange {
        CONE(0.13, 0.26),
        CUBE(0.0, 0.15);

        private final double min, max, center;
        public static final double maxLidarDetectionRange = 0.75;

        LidarRange(double min, double max) {
            this.min = min;
            this.max = max;
            this.center = (min + max) / 2.0;
        }

        public double getMin() {
            return min;
        }
        public double getMax() {
            return max;
        }
        public double getCenter() {
            return center;
        }
    }

    public static final int pieceDetectorPort = StormProp.getInt("pieceDetectorPort", 0);

    public static class ArmConstants {

        public static final int armShoulderID = StormProp.getInt("armShoulderID", -1);
        public static final int armElbowID = StormProp.getInt("armElbowID", -1);
        public static final int armShoulderEncoderID = StormProp.getInt("armShoulderEncoderID", -1);
        public static final int armElbowEncoderID = StormProp.getInt("armElbowEncoderID", -1);
        public static final int armShoulderEncoderOffsetTicks = StormProp.getInt("armShoulderEncoderOffsetTicks", -1);
        public static final int armElbowEncoderOffsetTicks = StormProp.getInt("armElbowEncoderOffsetTicks", -1);
        public static final double armElbowGearRatio = StormProp.getNumber("armElbowGearRatio", 1.0);
        public static final double armShoulderGearRatio = StormProp.getNumber("armShoulderGearRatio", 1.0);
	    public static final double armInlay = StormProp.getNumber("armInlay", 1.0);
	    public static final double gripperWheelRadius = StormProp.getNumber("gripperWheelRadius",1.0);
	    public static final double armForwardSafetyBuffer = StormProp.getNumber("armForwardSafetyBuffer", 0.0);
	    public static final double armUpwardSafetyBuffer = StormProp.getNumber("armUpwardSafetyBuffer", 1.0);
	    public static final double armBackwardSafetyBuffer = StormProp.getNumber("armBackwardSafetyBuffer", 1.0);
	    public static final double armDownwardSafetyBuffer = StormProp.getNumber("armDownwardSafetyBuffer", 1.0);
	    public static final double chassisHeight = StormProp.getNumber("chassisHeight", 1.0);
	    public static final double forwardConstraint = StormProp.getNumber("forwardConstraint", 1.0);
	    public static final double upwardConstraint = StormProp.getNumber("upwardConstraint", 1.0);
	    public static final double backwardConstraint = StormProp.getNumber("backwardConstraint", 1.0);
	    public static final double downwardConstraint = StormProp.getNumber("downwardConstraint", 1.0);
        public static final double kA1Length = StormProp.getNumber("A1Length", 1.0);
        public static final double kA2Length = StormProp.getNumber("A2Length", 1.0);

        private static final double armOriginX = StormProp.getNumber("armOriginX", 0.1);
        private static final double armOriginY = StormProp.getNumber("armOriginY", 0.1);

        public static final Translation2d armTranslation = new Translation2d(
                Units.inchesToMeters(armOriginX),
                Units.inchesToMeters(armOriginY));

        public static final Translation2d pickGround = new Translation2d(0.55, -0.3);
        public static final Translation2d stowPosition = new Translation2d(0.21, 0.10);
        private static final double outToDoubleSubstation = Units.feetToMeters(2.0);
        public static final Translation2d pickDoubleSubstationCone = new Translation2d(0.86, 0.85);
        public static final Translation2d pickDoubleSubstationCube = new Translation2d(0.86, 0.85);

        public static Translation2d tempArmPickUpLocation = Arm.fromGlobalTranslation(
                new Translation3d(1.15, 0.0, 1.235)
        );

        public static Translation2d getPickupLocation() {
            Translation2d pickingHeight = (RobotState.getInstance().getLidarRange() == Constants.LidarRange.CONE) ?
                    pickDoubleSubstationCone : pickDoubleSubstationCube;

            System.out.println(pickingHeight);

            return new Translation2d(pickingHeight.getX(), pickingHeight.getY() + 0.04);
        }
    }

    public static class VisionConstants {
          private static final double kCameraXTranslation = Units.inchesToMeters(StormProp.getNumber("CameraWpiX", 0.0));
          private static final double kCameraYTranslation = Units.inchesToMeters(StormProp.getNumber("CameraWpiY", 0.0));
          private static final double kCameraZTranslation = Units.inchesToMeters(StormProp.getNumber("CameraWpiZ", 0.0));
          private static final double kCameraYaw = StormProp.getNumber("CameraYaw", 0.0);
          private static final double kCameraPitch = StormProp.getNumber("CameraPitch", 0.0);
          public static final double kMaxAprilTagYawTrustMeters =
                  StormProp.getNumber("MaxAprilTagYawTrustMeters", 5.0);
          public static final double kMaxAprilTagLinearVelTrustMetersPerSec =
                  StormProp.getNumber("MaxAprilTagLinearVelTrustMetersPerSec", 2.0);
          public static final double kMaxAprilTagRotationVelTrustDegPerSec =
                  StormProp.getNumber("MaxAprilTagRotationalVelTrustDegPerSec", 50.0);
          public static final double kMaxTranslationDeviation =
                  StormProp.getNumber("MaxTranslationDeviation", 1.0);
          public static final double kMaxRotationDeviation =
                  StormProp.getNumber("MaxRotationDeviation", 1.0);

          public static double kDistanceTrustWeight =
                  StormProp.getNumber("DistanceTrustWeight", 0.2);
          public static double kLinearVelTrustWeight =
                  StormProp.getNumber("LinearVelTrustWeight", 0.2);
          public static double kRotationalVelTrustWeight =
                  StormProp.getNumber("RotationalVelTrustWeight", 0.2);

          private static double sumWeights = kDistanceTrustWeight + kLinearVelTrustWeight + kRotationalVelTrustWeight;

          static {
              kDistanceTrustWeight /= sumWeights;
              kLinearVelTrustWeight /= sumWeights;
              kRotationalVelTrustWeight /= sumWeights;
          }

          public static final Pose3d CAMERA_POSITION = new Pose3d(
                  kCameraXTranslation, kCameraYTranslation, kCameraZTranslation,
                  new Rotation3d(0,
                          Math.toRadians(kCameraPitch),
                          Math.toRadians(kCameraYaw))
          );

          // could be done a lot easier
          public static final Transform3d CAMERA_ROBOT_TRANSFORM = new Transform3d(
                  CAMERA_POSITION,
                  new Pose3d()
          );
          public static final Transform2d CAMERA_ROBOT_TRANSFORM2D = new Transform2d(
                  CAMERA_POSITION.toPose2d(),
                  new Pose2d()
          );
      }


    public static class Toggles {
        // **********
        // Usage members aren't actually final. They can be overridden if the system fails to come online (etc)
        // **********
        public static boolean useDrive = StormProp.getBoolean("useDrive", false);
        public static boolean useNavX = StormProp.getBoolean("useNavX", false);
        public static boolean useLogitechController = StormProp.getBoolean("useLogitechController", false);
        public static boolean useFirstXboxController = StormProp.getBoolean("useFirstXboxController", false);
        public static boolean useSecondXboxController = StormProp.getBoolean("useSecondXboxController", false);
        public static boolean useButtonBoard = StormProp.getBoolean("useButtonBoard", false);
        public static boolean usePneumatics = StormProp.getBoolean("usePneumatics", false);

        public static boolean useIntake = StormProp.getBoolean("useIntake", false);

        public static boolean useStormNet = StormProp.getBoolean("useStormNet", false);
        public static boolean useArm = StormProp.getBoolean("useArm", false);
        public static boolean useXYArmMode = StormProp.getBoolean("useXYArmMode", false) && useArm;
        public static boolean useVision = StormProp.getBoolean("useVision", false);
        public static boolean useStatusLights = StormProp.getBoolean("useStatusLights", false);
        public static boolean usePoseEstimator = false;
        public static boolean useNodeSelector = StormProp.getBoolean("useNodeSelector", false);
        public static boolean useLogs = true;
        public static boolean usePieceDetector = StormProp.getBoolean("usePieceDetector", false);
    }
    // **********
    // Don't put other variables after the usage members
    // **********
}

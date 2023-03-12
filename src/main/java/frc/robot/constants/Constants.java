// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
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

    public static final String kMK4iModuleKind = StormProp.getString("mk4iModuleKind", "");
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = StormProp.getNumber("drivetrainTrackwidthMeters", 0.);
    public static final double DRIVETRAIN_WHEELBASE_METERS = StormProp.getNumber("drivetrainWheelbaseMeters", 0.);
    public static final double ROBOT_WIDTH = StormProp.getNumber("robotWidth", 0.);
    public static final double ROBOT_LENGTH = StormProp.getNumber("robotLength", 0.);
    public static final double BUMPER_THICKNESS = StormProp.getNumber("bumperThickness", 0.);

    // <mecanum> Not needed for swerve, just mecanum
    public static final double kWheelRadiusMeters = StormProp.getNumber("wheelRadiusMeters", 0.);
    public static final double kWheelMaxRPM = StormProp.getNumber("wheelMaxRPM", 0.);
    // </mecanum>

    // Map to SDS implementation constants
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

    public static final int kCompressorModuleId = StormProp.getInt("CompressorModuleId", -1);
    public static final int onOffSolenoidChannel = StormProp.getInt("onOffSolenoidChannel", -1);
    public static final int cubeConeSolenoidChannel = StormProp.getInt("cubeConeSolenoidChannel", -1);
    public static final int armShoulderID = StormProp.getInt("armShoulderID", -1);
    public static final int armElbowID = StormProp.getInt("armElbowID", -1);
    public static final int armShoulderEncoderID = StormProp.getInt("armShoulderEncoderID", -1);
    public static final int armElbowEncoderID = StormProp.getInt("armElbowEncoderID", -1);
    public static final int armShoulderEncoderOffsetTicks = StormProp.getInt("armShoulderEncoderOffsetTicks", -1);
    public static final int armElbowEncoderOffsetTicks = StormProp.getInt("armElbowEncoderOffsetTicks", -1);
    public static final int kMagEncoderTicksPerRotation = StormProp.getInt("magEncoderTicksPerRotation", 0);
    public static final double armElbowGearRatio = StormProp.getNumber("armElbowGearRatio", 1.0);
    public static final double armShoulderGearRatio = StormProp.getNumber("armShoulderGearRatio", 1.0);
    public static final double kA1Length = StormProp.getNumber("A1Length", 1.0);
    public static final double kA2Length = StormProp.getNumber("A2Length", 1.0);
    public static final String navXConnection = StormProp.getString("navXConnection", "");

    public static final double driveXkp = StormProp.getNumber("driveXkp", 1.0);
    public static final double driveYkp = StormProp.getNumber("driveYkp", 1.0);
    public static final double turnkp = StormProp.getNumber("turnkp", 1.0);

    public static final double driveXki = StormProp.getNumber("driveXki", 1.0);
    public static final double driveYki = StormProp.getNumber("driveYki", 1.0);

      public static class VisionConstants {
          private static final double kCameraXTranslation = Units.inchesToMeters(StormProp.getNumber("CameraWpiX", 0.0));
          private static final double kCameraYTranslation = Units.inchesToMeters(StormProp.getNumber("CameraWpiY", 0.0));
          private static final double kCameraZTranslation = Units.inchesToMeters(StormProp.getNumber("CameraWpiZ", 0.0));
          private static final double kCameraYaw = StormProp.getNumber("CameraYaw", 0.0);
          private static final double kCameraPitch = StormProp.getNumber("CameraPitch", 0.0);
          public static final double kAprilTagYawTrustMeters = StormProp.getNumber("AprilTagYawTrustMeters", 0.5);

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


    public static class SubsystemToggles {
        // **********
        // Usage members aren't actually final. They can be overridden if the system fails to come online (etc)
        // **********
        public static boolean useDrive = StormProp.getBoolean("useDrive", false);
        public static boolean useNavX = StormProp.getBoolean("useNavX", false);
//        public static boolean useController = StormProp.getBoolean("useController", false);
        public static boolean useLogitechController = StormProp.getBoolean("useLogitechController", false);
        public static boolean useXboxController = StormProp.getBoolean("useXboxController", false);
        public static boolean usePneumatics = StormProp.getBoolean("usePneumatics", false);
        public static boolean useStormNet = StormProp.getBoolean("useStormNet", false);
        public static boolean useArm = StormProp.getBoolean("useArm", false);
        public static boolean useVision = StormProp.getBoolean("useVision", false);
        public static boolean useStatusLights = StormProp.getBoolean("useStatusLights", false);
        public static boolean usePoseEstimator = false;
        public static boolean useNodeSelector = StormProp.getBoolean("useNodeSelector", false);
    }
    // **********
    // Don't put other variables after the usage members
    // **********
}

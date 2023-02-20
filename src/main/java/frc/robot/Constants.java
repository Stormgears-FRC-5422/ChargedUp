// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.utils.configfile.StormProp;


  /**
   * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
   * constants. This class should not be used for any other purpose. All constants should be declared
   * globally (i.e. public static). Do not put anything functional in this class.
   *
   * <p>It is advised to statically import this class (or one of its inner classes) wherever the
   * constants are needed, to reduce verbosity.
   */
public final class Constants {

    public static final String robotName = StormProp.getString("robotName", "");

    public static final boolean useCompressor = StormProp.getBoolean("useCompressor",true);

    public static final double kStickNullSize = StormProp.getNumber("StickNullSize", 0.1);
    public static final int kLogitechControllerPort = StormProp.getInt("LogitechControllerPort", -1);
    public static final double kTemperatureRampThreshold = StormProp.getNumber("SparkMaxTemperatureRampThreshold", 45.0);
    public static final double kTemperatureRampLimit = StormProp.getNumber("SparkMaxTemperatureRampLimit", 60.0);
    public static final double kSparkMaxCurrentLimit = StormProp.getNumber("SparkMaxCurrentLimit", 35.0);
    public static final double kSparkMaxCurrentLimit550 = StormProp.getNumber("SparkMax550CurrentLimit", 25.0);

    public static final double kSparkMaxFreeSpeedRPM = StormProp.getNumber("SparkMaxFreeSpeedRPM", 0.0);
    public static final double kDriveSpeedScale = StormProp.getNumber("driveSpeedScale", 0.0);
    public static final double kPrecisionSpeedScale = StormProp.getNumber("precisionSpeedScale", 0.0);

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

    // <mecanum> Not needed for swerve, just mecanum
    public static final double kWheelRadiumMeters = StormProp.getNumber("wheelRadiusMeters", 0.);
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
    public static final int kSolendoidChannel = StormProp.getInt("SolenoidChannel", -1);
    public static final int armShoulderID = StormProp.getInt("armShoulderID", -1);
    public static final int armElbowID = StormProp.getInt("armElbowID", -1);
    
    public static final String navXConnection = StormProp.getString("navXConnection", "");


    // Usage members aren't actually final. They can be overridden if the system fails to come online (etc)
    public static boolean useDrive = StormProp.getBoolean("useDrive", false);
    public static boolean useNavX = StormProp.getBoolean("useNavX", false);
    public static boolean useController = StormProp.getBoolean("useController", false);
    public static boolean usePneumatics = StormProp.getBoolean("usePneumatics",false);
    public static boolean useStormNet = StormProp.getBoolean("useStormNet",false);
	public static boolean useArm = StormProp.getBoolean("useArm", false);
}

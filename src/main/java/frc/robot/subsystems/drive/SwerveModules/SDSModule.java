package frc.robot.subsystems.drive.SwerveModules;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.utils.stormSwerveLib.StormSwerveMk4ProtoHelper;

public class SDSModule implements SwerveModuleBase {

    SwerveModule sdsModule;

    public SDSModule(ShuffleboardTab tab, StormSwerveMk4ProtoHelper.GearRatio gearRatio, int driveMotorID, int steerMotorID, int steerEncoderID, double steerOffset) {
        sdsModule = StormSwerveMk4ProtoHelper.createNeo( tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0),
                gearRatio,
                driveMotorID,
                steerMotorID,
                steerEncoderID,
                steerOffset);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return sdsModule.getPosition();
    }

    @Override
    public double getDriveVel() {
        return sdsModule.getDriveVelocity();
    }

    @Override
    public Rotation2d getSteerAngle() {
        return Rotation2d.fromDegrees(sdsModule.getSteerAngle());
    }

    @Override
    public void setDesiredState(double speed, Rotation2d angle) {
        sdsModule.set(speed, angle.getRadians());
    }
}

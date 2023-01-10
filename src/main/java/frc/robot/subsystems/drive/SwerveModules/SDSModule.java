package frc.robot.subsystems.drive.SwerveModules;

import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public double getPosition() {
        //FIXME: if we use the other fork by the guy below we can just access the motor and grap encoder values for this.
        //5431 Programming Lead
        //Feb '22
        //Also, my fork of swerve-lib corrects this issue and provides other tweaks like exposing the motors and encoders:
        //
        //See the edit to the vendordep json: https://github.com/frc5431/RobotCode2022/blob/master/vendordeps/SdsSwerveLib.json 60
        //
        //GitHub: https://github.com/democat3457/swerve-lib/tree/prod 73
        //
        //I made this change after realizing that the swerve-lib code explicitly sets the
        // boot configuration to boot_to_zero upon constructing the swerve modules,
        // which were causing our swerve wheel alignments to break on power cycle.
        return 0;
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

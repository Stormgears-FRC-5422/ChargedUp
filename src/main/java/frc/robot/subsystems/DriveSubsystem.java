package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.motorcontrol.StormSpark;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {

    private final StormSpark frontLeft = new StormSpark(Constants.kMasterLeftId, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
    private final StormSpark frontRight = new StormSpark(Constants.kMasterRightId, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
    private final StormSpark backLeft = new StormSpark(Constants.kSlaveLeftId, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
    private final StormSpark backRight = new StormSpark(Constants.kSlaveRightId, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);

    private final DifferentialDrive differentialDrive = new DifferentialDrive(frontLeft, frontRight);

    public DriveSubsystem() {
        backLeft.follow(frontLeft);
        backRight.follow(frontRight);
    }


    public void driveArcade(DoubleSupplier xSpeed, DoubleSupplier zSpeed) {
        differentialDrive.arcadeDrive(xSpeed.getAsDouble()*0.15, zSpeed.getAsDouble()*0.2, false);

    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        super.setDefaultCommand(defaultCommand);
    }
}

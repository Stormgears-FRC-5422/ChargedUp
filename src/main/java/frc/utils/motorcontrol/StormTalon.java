package frc.utils.motorcontrol;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;

import static frc.robot.constants.Constants.kMagEncoderTicksPerRotation;
import static java.lang.Math.floor;

public class StormTalon extends WPI_TalonSRX {
  private double scale = 1.0;
  // Offset in ticks
  private int offset = 0;
  private int timeoutMs = 15;
  public boolean atHome = false;

  private int maybeNegate = 1;

  public enum AngleRangeType {
    rangeAbsolute,
    range0to1,
    rangeNegToPos
  }

  public StormTalon(int deviceID) {
    super(deviceID);
    this.getSensorCollection().setQuadraturePosition(getPWMPositionTicks(), timeoutMs);
    this.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
  }

  public void setOffset(int offset) {
    this.offset = offset;
  }

  public void setNegatePosition(boolean negatePosition) {
    this.maybeNegate = negatePosition ? -1 : 1;
  }
  public void setOffsetRadians(double offset) {
    setOffset( (int)Math.round(kMagEncoderTicksPerRotation * ( offset / (2 * Math.PI) ) ) );
  }

  public double getPositionRadians() {
    return getPositionRadians(AngleRangeType.range0to1);
  }

  public double getPositionRadians(AngleRangeType rangeType) {
    double rotation = (double)getPositionTicks() / kMagEncoderTicksPerRotation;

    switch(rangeType) {
      case range0to1:
        rotation = rotation - floor(rotation);
        break;
      case rangeNegToPos:
        rotation = rotation - floor(rotation);
        rotation = rotation > 0.5 ? rotation - 1 : rotation;
        break;
      case rangeAbsolute:
      default:
    }

    return (2. * Math.PI * rotation);
  }

  public double getPositionDegrees() {
    return ( Math.toDegrees(getPositionRadians()) );
  }

  public double getPositionDegrees(AngleRangeType rangeType) {
    return ( Math.toDegrees(getPositionRadians(rangeType)) );
  }

  // This is a bit funny. The offset is a number that gets *added* to the current position.
  // It is done this way to align with the sense of offset used by SDS in a few different places in the Swerve code.
  public int getPWMPositionTicks() {
    return maybeNegate * (this.getSensorCollection().getPulseWidthPosition() + offset);
  }

  public int getPositionTicks() {
    return maybeNegate * (this.getSensorCollection().getQuadraturePosition() + offset);
  }

  @Override
  public void set(double speed) {
    // Could put safety features - e.g. temperature control - here.
    super.set(scale * speed);
  }

  // Should be between 0.0 and 1.0 - to account for oddities in the drive train
  // e.g. two different gear ratios
  public void setSpeedScale(double scale) {
    this.scale = MathUtil.clamp(scale, 0, 1.0);
  }

}

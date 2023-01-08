package frc.utils.motorcontrol;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.*;

public class StormTalon extends WPI_TalonSRX {
  private double scale = 1.0;
  // Offset in ticks
  private int offset = 0;
  private int timeoutMs = 15;
  public boolean atHome = false;

  public StormTalon(int deviceID) {
    super(deviceID);
    this.getSensorCollection().setQuadraturePosition(getPWMPositionTicks(), timeoutMs);
  }

  public void setOffset(int offset) {
    this.offset = offset;
  }

  public void setOffsetRadians(double offset) {
    setOffset( (int)Math.round(kSwivelEncoderTicksPerRotation * ( offset / (2 * Math.PI) ) ) );
  }

  public int getPWMPositionTicks() {
    return this.getSensorCollection().getPulseWidthPosition() + offset;
  }

  public int getPositionTicks() {
    return this.getSensorCollection().getQuadraturePosition() + offset;
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

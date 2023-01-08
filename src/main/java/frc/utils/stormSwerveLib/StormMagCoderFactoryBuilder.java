package frc.utils.stormSwerveLib;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.swervedrivespecialties.swervelib.AbsoluteEncoder;
import com.swervedrivespecialties.swervelib.AbsoluteEncoderFactory;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CanCoderFactoryBuilder;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;

import com.ctre.phoenix.motorcontrol.StatusFrame;

public class StormMagCoderFactoryBuilder extends CanCoderFactoryBuilder {
    // Need these variables since I can't access the private parent variables
    private CanCoderFactoryBuilder.Direction thisDirection;
    private int thisPeriodMilliseconds;

    @Override
    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        return super.withReadingUpdatePeriod(this.thisPeriodMilliseconds = periodMilliseconds);
    }

    @Override
    public CanCoderFactoryBuilder withDirection(CanCoderFactoryBuilder.Direction direction) {
        return super.withDirection(this.thisDirection = direction);
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return (configuration) -> {
            CANCoderConfiguration config = new CANCoderConfiguration();
            config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            config.magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
            config.sensorDirection = this.thisDirection == CanCoderFactoryBuilder.Direction.CLOCKWISE;
            StormMagCoder encoder = new StormMagCoder(configuration.getId(), configuration.getOffset());
            CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");
            CtreUtils.checkCtreError(encoder.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, this.thisPeriodMilliseconds, 250), "Failed to configure CANCoder update rate");
            return new StormMagCoderFactoryBuilder.EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final StormMagCoder encoder;
        private EncoderImplementation(StormMagCoder encoder) {
            this.encoder = encoder;
        }

        public double getAbsoluteAngle() {
            double angle = Math.toRadians(this.encoder.getAbsolutePositionDegrees());
            angle %= 6.283185307179586;
            if (angle < 0.0) {
                angle += 6.283185307179586;
            }

            return angle;
        }
    }
}

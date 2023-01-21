package frc.utils.stormSwerveLib;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import frc.utils.motorcontrol.StormTalon;

import static frc.robot.Constants.kSwivelEncoderTicksPerRotation;

public class StormMagCoder {
    private final StormTalon talon;

    public StormMagCoder(int id, double offset) {
        this.talon = new StormTalon(id);
        this.talon.setOffsetRadians(offset);
    }

    public ErrorCode configAllSettings(CANCoderConfiguration config, int i) {
        // TODO - save config settings
        return ErrorCode.OK;
    }

    public ErrorCode setStatusFramePeriod(StatusFrame sensorData, int periodMilliseconds, int i) {
        // TODO - check that this is getting the right frame
        talon.setStatusFramePeriod(sensorData, periodMilliseconds, i);
        return ErrorCode.OK;
    }

    public double getAbsolutePositionDegrees() {
        return 360. * ((double)talon.getPositionTicks() / kSwivelEncoderTicksPerRotation);
    }

}

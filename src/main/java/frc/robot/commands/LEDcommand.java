package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.stormnet.StormNet;

import static frc.robot.constants.Constants.LidarRange;

public class LEDcommand extends CommandBase {
    NeoPixel neoPixel;
    StormNet stormNet;

    private int count = 0;
    // measured in iterations describes how often to
    private int blinkRate = 1;
    // slowest it goes at the max distance away
    private static int lowestBlinkRate = 60;

    Compression compression;
    private final int[] segments = {4, 5};

    public LEDcommand(StormNet stormNet, NeoPixel neoPixel, Compression compression) {
        this.neoPixel = neoPixel;
        this.stormNet = stormNet;
        this.compression = compression;
        addRequirements(neoPixel);
    }

    @Override
    public void initialize() {
        System.out.println("LidarLights starting");
    }

    @Override
    public void execute() {
        double distance = stormNet.getLidarDistance();
        LidarRange currentRange = RobotState.getInstance().getLidarRange();
//        System.out.println("Lidar: " + distance);
        if (distance <= LidarRange.maxLidarDetectionRange) {
            double error = distance - currentRange.getCenter();
            Color8Bit color =  (currentRange == LidarRange.CONE) ? NeoPixel.YELLOW_COLOR : NeoPixel.PURPLE_COLOR;
            blinkRate = 0;
            if (compression.isSolenoidSet()) {
                blinkRate = (int) (MathUtil.applyDeadband(error,
                        currentRange.getMax() - currentRange.getCenter(),
                        LidarRange.maxLidarDetectionRange - currentRange.getCenter())
                        * lowestBlinkRate);


            }

            // by default set the color to yellow or correct
         
            // blink rate will be positive or negative if too far or close
            if (blinkRate < 0 )
                color = NeoPixel.RED_COLOR;
            else if (blinkRate > 0 )
                color = NeoPixel.BLUE_COLOR;

            // make sure we are doing modulo of positive number... does it matter?
            blinkRate = Math.abs(blinkRate);
            if (blinkRate == 0) {
                neoPixel.setSpecificSegmentColor(segments, color);
            } else {
                count++;
                if (count % blinkRate == 0) {
                    neoPixel.setSpecificSegmentColor(segments, color);
                } else {
                    neoPixel.setSpecificSegmentColor(segments, NeoPixel.NO_COLOR);
                }
            }
        } else {
            neoPixel.setSpecificSegmentColor(segments, NeoPixel.NO_COLOR);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}



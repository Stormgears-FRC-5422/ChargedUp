package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.stormnet.StormNet;

public class LEDcommand extends CommandBase {
    NeoPixel neoPixel;
    StormNet stormNet;

    private int count = 0;
    // measured in iterations describes how often to
    private int blinkRate = 1;
    // slowest it goes at the max distance away
    private static int lowestBlinkRate = 30;


    private boolean blink;

    int[] segments = {4, 5};

    public LEDcommand(StormNet stormNet, NeoPixel neoPixel) {
        this.neoPixel = neoPixel;
        this.stormNet = stormNet;
        addRequirements(neoPixel);
    }

    @Override
    public void initialize() {
        System.out.println("LidarLights starting");
    }

    @Override
    public void execute() {
        double distance = stormNet.getLidarDistance();
        Constants.LidarRange currentRange = RobotState.getInstance().getLidarRange();
//        System.out.println("Lidar: " + distance);
        if (distance <= 0.75) {
            if (blink) {
            double error = distance - currentRange.getCenter();
            blinkRate = (int) (MathUtil
                    .applyDeadband(error,
                            currentRange.getMax() - currentRange.getCenter(),
                            0.75 - currentRange.getCenter()) * lowestBlinkRate - 1) + 1;

            // by default set the color to yellow or correct
            Color8Bit color = (currentRange == Constants.LidarRange.CONE)? NeoPixel.YELLOW_COLOR : NeoPixel.PURPLE_COLOR;
            // blink rate will be positive or negative if too far or close
            if (blinkRate < 0)
                color = NeoPixel.RED_COLOR;
            else if (blinkRate > 0)
                color = NeoPixel.BLUE_COLOR;

            // make sure we are doing modulo of positive number... does it matter?
            blinkRate = Math.abs(blinkRate);
            if (blinkRate == 0) {
                neoPixel.setSpecificSegmentColor(segments, color);
                return;
            }

            if (++count % blinkRate == 0)
                neoPixel.setSpecificSegmentColor(segments, color);
            else
                neoPixel.setSpecificSegmentColor(segments, NeoPixel.NO_COLOR);

        } else {
                if (currentRange == Constants.LidarRange.CONE) {
                    if (distance < currentRange.getMin()) {
                        neoPixel.setSpecificSegmentColor(segments, NeoPixel.RED_COLOR);
                    } else if (distance <= currentRange.getMax()) {
                        neoPixel.setSpecificSegmentColor(segments, NeoPixel.YELLOW_COLOR);
                    } else {
                        neoPixel.setSpecificSegmentColor(segments, NeoPixel.BLUE_COLOR);
                    }
                } else {
                    if (distance < currentRange.getMin()) {
                        neoPixel.setSpecificSegmentColor(segments, NeoPixel.RED_COLOR);
                    }
                    else if (distance <= currentRange.getMax()) {
                        neoPixel.setSpecificSegmentColor(segments, NeoPixel.PURPLE_COLOR);
                    } else {
                        neoPixel.setSpecificSegmentColor(segments, NeoPixel.BLUE_COLOR);
                    }
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

    public void setBlink(boolean blink) {
        this.blink = blink;
    }

}



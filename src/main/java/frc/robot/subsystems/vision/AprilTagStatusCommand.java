package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoPixel;

public class AprilTagStatusCommand extends CommandBase {

    NeoPixel neoPixel;
    Vision vision;

    boolean pastAprilTagStatus = false;

    public AprilTagStatusCommand(Vision vision, NeoPixel neoPixel) {
        this.vision = vision;
        this.neoPixel = neoPixel;
    }

    @Override
    public void execute() {
        if (vision.getAprilTagStatus() && !pastAprilTagStatus) {
            neoPixel.setColor(0, NeoPixel.RED_COLOR);
            pastAprilTagStatus = true;
        } else if (!vision.getAprilTagStatus() && pastAprilTagStatus) {
            neoPixel.setColor(0, NeoPixel.NO_COLOR);
            pastAprilTagStatus = false;
        }
    }
}

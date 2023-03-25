package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.vision.Vision;

public class AprilTagStatusCommand extends CommandBase {
  NeoPixel neoPixel;
  Vision vision;

  boolean pastAprilTagStatus = false;

  public AprilTagStatusCommand(NeoPixel neoPixel, Vision vision){
    this.neoPixel =  neoPixel;
    this.vision =  vision;
  }
  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    if (vision.getAprilTagStatus() != pastAprilTagStatus) {
      if (vision.getAprilTagStatus()) {
        neoPixel.setColor(0, NeoPixel.GREEN_COLOR);
      } else {
        neoPixel.setColor(0, NeoPixel.NO_COLOR);
      }
      pastAprilTagStatus = vision.getAprilTagStatus();
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}

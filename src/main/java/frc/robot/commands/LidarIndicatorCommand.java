package frc.robot.commands;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.stormnet.StormNet;


public class LidarIndicatorCommand extends CommandBase {
  NeoPixel neoPixel;
  StormNet m_stormNet;

  int[] segments = {4, 5};

  public LidarIndicatorCommand(StormNet stormNet, NeoPixel neoPixel) {
    this.neoPixel = neoPixel;
    m_stormNet = stormNet;
  }

  @Override
  public void initialize() {
    System.out.println("LidarLights starting");
  }

  @Override
  public void execute() {
    double distance = m_stormNet.getLidarDistance();
//    SmartDashboard.putNumber("distance", distance);
//    System.out.println(distance);
    if (distance >= 0.13 && distance <= 0.257
    ) {
      neoPixel.setSpecificSegmentColor(segments, NeoPixel.GREEN_COLOR);
    }
    if (distance < 0.13) {
      neoPixel.setSpecificSegmentColor(segments, NeoPixel.RED_COLOR);
    }
    if (distance > 0.257) {
      neoPixel.setSpecificSegmentColor(segments, NeoPixel.YELLOW_COLOR);
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
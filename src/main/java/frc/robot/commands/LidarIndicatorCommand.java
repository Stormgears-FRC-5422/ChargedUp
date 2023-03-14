package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.stormnet.StormNet;
import frc.utils.configfile.StormProp;


public class LidarIndicatorCommand extends CommandBase {

  NeoPixel neoPixel;

  StormNet m_stormNet;



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
    System.out.println(distance);
    if (distance >= 0.1 && distance <= 0.2) {
      neoPixel.setColor(5, NeoPixel.PURPLE_COLOR);
    }
    if (distance < 0.1) {
      neoPixel.setColor(5, NeoPixel.RED_COLOR);
    }
    if (distance > 0.3) {
      neoPixel.setColor(5, NeoPixel.YELLOW_COLOR);
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
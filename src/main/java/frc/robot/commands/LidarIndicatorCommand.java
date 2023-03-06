package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.stormnet.StormNet;
import frc.utils.configfile.StormProp;


public class LidarIndicatorCommand extends CommandBase {

  NeoPixel m_neoPixel = new NeoPixel();

  StormNet m_stormNet;



  public LidarIndicatorCommand(StormNet stormNet) {
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
      m_neoPixel.setAll(NeoPixel.fullColorG);
    }
    if (distance < 0.1) {
      m_neoPixel.setAll(NeoPixel.fullColorR);
    }
    if (distance > 0.3) {
      m_neoPixel.setAll(NeoPixel.fullColorY);
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
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.stormnet.StormNet;
import frc.utils.joysticks.ButtonBoardConfig;
import frc.utils.joysticks.CubeCone;


public class LEDcommand extends CommandBase {
  NeoPixel neoPixel;
  StormNet stormNet;

  ButtonBoardConfig buttonBoardConfig;


  int[] segments = {4, 5};

  public LEDcommand(StormNet stormNet, NeoPixel neoPixel, ButtonBoardConfig buttonBoardConfig) {
    this.neoPixel = neoPixel;
    this.stormNet = stormNet;
    this.buttonBoardConfig = buttonBoardConfig;
    addRequirements(neoPixel);
  }

  @Override
  public void initialize() {
    System.out.println("LidarLights starting");
  }

  @Override
  public void execute() {
    double distance = stormNet.getLidarDistance();
    CubeCone m_cubeCone = buttonBoardConfig.m_cubeCone;
    if (distance < 0.5) {
      if (m_cubeCone.equals(CubeCone.CONE)) {
        if (distance <0.13) {
          neoPixel.setSpecificSegmentColor(segments, NeoPixel.RED_COLOR);
        } else if (distance <= 0.257) {
          neoPixel.setSpecificSegmentColor(segments, NeoPixel.YELLOW_COLOR);
        } else {
          neoPixel.setSpecificSegmentColor(segments, NeoPixel.BLUE_COLOR);
        }
      } else {
        if (distance <= 0.257) {
          neoPixel.setSpecificSegmentColor(segments, NeoPixel.PURPLE_COLOR);
        } else {
          neoPixel.setSpecificSegmentColor(segments, NeoPixel.BLUE_COLOR);
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



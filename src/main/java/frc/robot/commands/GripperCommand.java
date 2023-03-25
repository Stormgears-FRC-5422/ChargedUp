package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.stormnet.StormNet;

public class GripperCommand extends CommandBase {
  Compression compression;

  StormNet stormNet;

  boolean GrippercommandStatus;

  public GripperCommand(Compression compression, StormNet stormNet) {
    this.compression = compression;
    this.stormNet = stormNet;
    addRequirements(compression);
  }

  @Override
  public void initialize() {
    System.out.println("Starting Gripper Command");
  }

  @Override
  public void execute() {
    compression.grabCubeOrCone();
    GrippercommandStatus = true;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return GrippercommandStatus;
  }
}

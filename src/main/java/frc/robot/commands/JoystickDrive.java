
package frc.robot.commands;

import frc.robot.subsystems.MecanumDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class JoystickDrive extends CommandBase {


  private MecanumDrive mecanumdrive;

  private final DoubleSupplier x;
  private final DoubleSupplier y;
  private final DoubleSupplier z;
  private final DoubleSupplier speed;
  private final Double maxSpeed = 0.25;

  /**
   * Creates a new ExampleCommand.
   *
   *
   */
  public JoystickDrive(MecanumDrive mecanumdrive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z, DoubleSupplier speed) {
    this.mecanumdrive = mecanumdrive;
    this.x = x;
    this.y = y;
    this.z = z;
    this.speed = speed;


    addRequirements(mecanumdrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // TODO - make a config to allow us to choose between these approaches
    // drive.drive(x.getAsDouble() * speed.getAsDouble(), y.getAsDouble() * speed.getAsDouble(), z.getAsDouble() 
    //     * speed.getAsDouble());

    mecanumdrive.drive(x.getAsDouble() * maxSpeed, y.getAsDouble() * maxSpeed, z.getAsDouble() * maxSpeed);

  }

  @Override
  public void end(boolean interrupted) {
    mecanumdrive.drive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
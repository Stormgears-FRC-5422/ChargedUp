package frc.robot.commands.drive;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.function.DoubleSupplier;


public class BalanceCommand extends CommandBase {

  private DoubleSupplier getRoll, getPitch;
  private DrivetrainBase drive;

  private final PIDController controller = new PIDController(0.02, 0.0, 0.00);
  private double m_currTilt;


  public BalanceCommand(DoubleSupplier getPitch, DoubleSupplier getRoll, DrivetrainBase drive) {
    this.getPitch = getPitch;
    this.getRoll = getRoll;
    this.drive = drive;

    controller.setTolerance(2.5);

    addRequirements(drive);
  }


  @Override
  public void initialize() {
    System.out.println("Balance Command Starting");
  }


  @Override
  public void execute() {
    double pidOut = 0;
    String directionType = null;
    System.out.println("Running");

    if (getPitch.getAsDouble() > 5 && getRoll.getAsDouble() > 5 || getPitch.getAsDouble() < 5 && getRoll.getAsDouble() < 5) {
      m_currTilt = getPitch.getAsDouble();
      pidOut = controller.calculate(getPitch.getAsDouble(), 0);
      directionType = "Pitch";

    } else if (getPitch.getAsDouble() > 5 || getPitch.getAsDouble() < -5) {
      m_currTilt = getPitch.getAsDouble();
      pidOut = controller.calculate(getPitch.getAsDouble(), 0);
      directionType = "Pitch";
    } else if (getRoll.getAsDouble() > 5 || getRoll.getAsDouble() < -5) {
      m_currTilt = getRoll.getAsDouble();
      pidOut = controller.calculate(getRoll.getAsDouble(), 0);
      System.out.println("Right");
      directionType = "Roll";
    }

    if (m_currTilt > 0 && directionType.equals("Pitch")) {
      drive.drive(new ChassisSpeeds(-pidOut, 0.0, 0.0), false);
    } else if (m_currTilt < 0 && directionType.equals("Pitch")) {
      drive.drive(new ChassisSpeeds(pidOut, 0.0, 0.0), false);
    } else if (m_currTilt > 0 && directionType.equals("Roll")) {
      drive.drive(new ChassisSpeeds(0.0, pidOut, 0.0), false);
    } else if (m_currTilt < 0 && directionType.equals("Roll")) {
      drive.drive(new ChassisSpeeds(0.0, pidOut, 0.0), false);
    }
  }


  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      drive.stopDrive();

  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}

package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DrivetrainBase;

public class GyroCommand extends CommandBase {

    private DrivetrainBase m_drivetrain;

    private double m_targetValue;
    private double m_currAngle;



    public GyroCommand(DrivetrainBase drivetrain, double targetValue){
        m_drivetrain = drivetrain;
        m_targetValue = targetValue;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("GyroCommand Starting");
        m_currAngle = m_drivetrain.getGyroscopeRotation().getDegrees();
    }

    @Override
    public void execute() {
        m_currAngle = m_drivetrain.getGyroscopeRotation().getDegrees();
        if(m_currAngle < m_targetValue) {
            m_drivetrain.percentOutDrive(
                    new ChassisSpeeds(
                            0.,
                            0.,
                            -0.1
                    ),
                    false
            );
        }
        if(m_currAngle > m_targetValue) {
            m_drivetrain.percentOutDrive(
                new ChassisSpeeds(
                        0.,
                        0.,
                        0.1
                    ),
                false
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.percentOutDrive(
                new ChassisSpeeds(
                        0.,
                        0.,
                        0.0
                ),
                false
        );
    }

    @Override
    public boolean isFinished() {
        return m_currAngle < 1.1 * m_targetValue && m_currAngle > 0.9 * m_targetValue;
    }

}

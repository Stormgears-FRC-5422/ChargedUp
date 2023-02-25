package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DrivetrainBase;

public class GyroCommand extends CommandBase {

    private DrivetrainBase m_drivetrain;

    private double m_targetValue;
    private double m_currAngle;
    private double m_error;
    private double Kp = 0.0008;

    public GyroCommand(DrivetrainBase drivetrain, double targetValue){
        m_drivetrain = drivetrain;
        m_targetValue = targetValue;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("GyroCommand Starting");
        m_currAngle = RobotState.getInstance().getCurrentPose().getRotation().getDegrees();
    }

    @Override
    public void execute() {
        m_currAngle = RobotState.getInstance().getCurrentPose().getRotation().getDegrees();
        m_error = m_targetValue - m_currAngle;
        double turnSpeed = m_error * Kp;
        if(m_currAngle < m_targetValue) {
            m_drivetrain.percentOutDrive(
                    new ChassisSpeeds(
                            0.,
                            0.,
                            -turnSpeed
                    ),
                    false
            );
        }
        if(m_currAngle > m_targetValue) {
            m_drivetrain.percentOutDrive(
                new ChassisSpeeds(
                        0.,
                        0.,
                        turnSpeed
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
        System.out.println("Current Angle: " + m_currAngle);
        System.out.println("Target Angle:" + m_targetValue);
//        double  m_Posthres = m_targetValue - 0.5;
//        double  m_Negthres = m_targetValue - 0.5;

        System.out.println("Error: " + m_error);
        return (m_error < 0.5 ) && (m_error > -0.5);
    }

}

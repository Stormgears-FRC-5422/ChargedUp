package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pathFollowing.DriveToPose;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DrivetrainBase;

import static frc.robot.constants.Constants.BUMPER_THICKNESS;
import static frc.robot.constants.Constants.ROBOT_LENGTH;

public class DriveToScoringNode extends SequentialCommandGroup {

    private final DrivetrainBase m_drivetrain;
    private final FieldConstants.Grids.ScoringNode m_node;

    public DriveToScoringNode(DrivetrainBase drivetrain, FieldConstants.Grids.ScoringNode node) {
        m_drivetrain = drivetrain;
        m_node = node;

        var scoringPose = m_node.scoringPosition;
        //drive in from 1.2 feet out
        double middleX = Units.inchesToMeters(54.05) + (ROBOT_LENGTH / 2.0) + BUMPER_THICKNESS + Units.feetToMeters(1.2);
        if (m_node.alliance == DriverStation.Alliance.Red) middleX = FieldConstants.mirrorXPosition(middleX);
        Pose2d middlePose = new Pose2d(middleX, scoringPose.getY(), scoringPose.getRotation());

        addCommands(
                new DriveToPose(middlePose, m_drivetrain, 3, 2),
                new DriveToPose(scoringPose, m_drivetrain, 0.6, 0.6));
    }
}

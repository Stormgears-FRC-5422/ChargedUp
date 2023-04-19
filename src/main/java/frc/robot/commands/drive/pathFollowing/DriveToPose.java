package frc.robot.commands.drive.pathFollowing;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class DriveToPose extends PathFollowingCommand {
    private final DrivetrainBase drivetrain;
    private Supplier<Pose2d> poseSupplier;
    private final double maxVel, maxAcc;

    public DriveToPose(DrivetrainBase drivetrain, Supplier<Pose2d> poseSupplier, double maxVel, double maxAcc) {
        super(drivetrain);
        this.drivetrain = drivetrain;
        this.poseSupplier = poseSupplier;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
    }

    public DriveToPose(DrivetrainBase drivetrain, Pose2d goalPose,
                       double maxVel, double maxAcc) {
        this(drivetrain, () -> goalPose, maxVel, maxAcc);
    }

    @Override
    public void initialize() {
        var currPose = RobotState.getInstance().getCurrentPose();
//        setPath(Paths.getPathFromPose(currPose, currVel, poseSupplier.get(), maxVel, maxAcc));

        List<PathPoint> points = new ArrayList<>();

        points.add(
                PathPoint.fromCurrentHolonomicState(currPose, drivetrain.getCurrentChassisSpeeds())
                        .withNextControlLength(0.01));

        boolean isRed = DriverStation.getAlliance() == DriverStation.Alliance.Red;
        // command only used for node alignment
        points.add(
                new PathPoint(
                        poseSupplier.get().getTranslation(),
                        new Rotation2d(isRed? 0.0 : 180.0),
                        poseSupplier.get().getRotation()
                )
                        .withPrevControlLength(Constants.TURN_RADIUS + Units.inchesToMeters(1.5)));

        setPath(PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAcc),
                points
        ));
        super.initialize();
    }
}
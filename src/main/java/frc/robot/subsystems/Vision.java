package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.data.StormStruct;

import java.util.HashMap;
import java.util.Vector;

/*
Vision Subsystem:
 -

April tag Class:
 - Get all data from network tables about april tags
 - If one april tag is found, approximate distance from two walls (should return x and y)
 - If at least two april tags are found, triangulate position and angles

Field Element Class:
 - Get all data from network tables about april tags
 - Get translation vectors and yaw for each field element found

Retro Reflective Class:
 - Get all data from network tables about april tags
 - Just return pixel difference for each reflective tape (Use PID in drive to line up)
 */
public class Vision extends SubsystemBase {
    private final StormStruct storm_struct;
    public Vision() {
        NetworkTableInstance nt_inst = NetworkTableInstance.getDefault();
        storm_struct = new StormStruct(nt_inst, "vision-data", "tag_data");
        getAprilTagInfo();
    }

    public void getAprilTagInfo() {
        Vector<HashMap<String,Double>> info = storm_struct.get_data("april_tag");
        for (int index = 0; index <= info.size() - 1; index++) {
            System.out.println(info.get(index).toString());
        }
        System.out.println(info);

    }

    public int[] getVisibleIds() {
        return new int[8];
    }

    public int get_distance_to_marker() {
        // TODO
        return 0;
    }

    public int triangulatePosition() {
        // TODO
        return 0;
    }

//    public static Pose2d estimateFieldToRobot(
//            double cameraHeightMeters,
//            double targetHeightMeters,
//            double cameraPitchRadians,
//            double targetPitchRadians,
//            Rotation2d targetYaw,
//            Rotation2d gyroAngle,
//            Pose2d fieldToTarget,
//            Transform2d cameraToRobot) {
//        return PhotonUtils.estimateFieldToRobot(
//                PhotonUtils.estimateCameraToTarget(
//                        PhotonUtils.estimateCameraToTargetTranslation(
//                                PhotonUtils.calculateDistanceToTargetMeters(
//                                        cameraHeightMeters, targetHeightMeters, cameraPitchRadians, targetPitchRadians),
//                                targetYaw),
//                        fieldToTarget,
//                        gyroAngle),
//                fieldToTarget,
//                cameraToRobot);

}

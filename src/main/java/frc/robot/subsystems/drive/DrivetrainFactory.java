package frc.robot.subsystems.drive;

public class DrivetrainFactory {
    protected static DrivetrainBase instance;

    public static DrivetrainBase getInstance(String driveType) {
        switch (driveType) {
            case "sdsSwerve":
                if (instance == null) {
                    instance = new SDSDrivetrain();
                }
                return instance;
            case "mecanum":
                if (instance == null) {
                    instance = new MecanumDrivetrain();
                }
                return instance;
            default:
                return null;
        }
    }

}

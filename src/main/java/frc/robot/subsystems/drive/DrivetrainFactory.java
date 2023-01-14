package frc.robot.subsystems.drive;

public class DrivetrainFactory {
    private static DrivetrainFactory instance;

    public DrivetrainFactory getInstance() {
        if (instance == null) instance = new DrivetrainFactory();
        return instance;
    }

    public static DrivetrainBase create(String robotName) {
        switch (robotName) {
            case ("SwerveBot"):
                return new SDSDrivetrain();
            case ("DeepSpaceBot"):
                return new DeepSpaceDrivetrain();
            default:
                System.out.println("Drivetrain type not identified!!!!!!!!!!!!!");
                return new SDSDrivetrain();
        }
    }
}

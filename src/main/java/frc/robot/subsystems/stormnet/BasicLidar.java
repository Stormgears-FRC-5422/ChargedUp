package frc.robot.subsystems.stormnet;

public class BasicLidar extends StormNetSensor {
	private final short[] sensorDetails;


	public BasicLidar(StormNetVoice voice) {
		super(voice);

		// TODO magic number
		// [0] is the distance in mm from sensor 0, [1] is sensor quality indicator
		sensorDetails = new short[1];
		this.m_deviceString = voice.getDeviceString();
	}

	public void pollDetails() {
		fetchShorts("L", "LidarDistance", sensorDetails);
	}

	// Distance from Lidar. Arrives in mm, returned in m
	public double getDistance() {
		pollDetails();
		return ((double) sensorDetails[0] / 1000.);
	}

	public boolean getSensorQuality() {
		pollDetails();
		return( sensorDetails[1] != 0);
	}

}

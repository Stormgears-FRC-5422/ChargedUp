// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.utils.subsystemUtils.StormSubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.constants.Constants.*;

public class Compression extends StormSubsystemBase {
    private Compressor mainCompressor;
    private Solenoid onOffSolenoid;
    private boolean running;
    private boolean solenoidSet;
    public Compression() {
        mainCompressor = new Compressor(kCompressorModuleId, PneumaticsModuleType.CTREPCM);
        mainCompressor.disable();

        onOffSolenoid = new Solenoid(kCompressorModuleId, PneumaticsModuleType.CTREPCM, onOffSolenoidChannel);
        onOffSolenoid.set(false);

        printStatus();
    }
    public void startCompressor() {
        running = true;
        mainCompressor.enableDigital();
        printStatus();
    }
    public void stopCompressor(){
        running = false;
        mainCompressor.disable();
        printStatus();
    }

    public void toggleCompressor() {
        if (running) {
            stopCompressor();
        } else {
            startCompressor();
        }
    }
    public void grabCubeOrCone(){
        solenoidSet = true;
        onOffSolenoid.set(true);
    }
    public void release(){
        solenoidSet = false;
        onOffSolenoid.set(false);
    }
    private void printStatus() {
        System.out.println("\n\nShould be running: " + running);
        System.out.println("Solenoid should be enabled(): " + solenoidSet);
        System.out.println("Compressor isEnabled: " + mainCompressor.isEnabled());
        System.out.println("Compressor getPressureSwitchValve: " + mainCompressor.getPressureSwitchValue());
        System.out.println("Solenoid isEnabled(): " + !onOffSolenoid.isDisabled());
    }

    @Override
    public void stormPeriodic() {
        SmartDashboard.putBoolean("The compressor is enabled", mainCompressor.isEnabled());
        SmartDashboard.putBoolean("Solenoid isEnabled(): ", !onOffSolenoid.isDisabled());
        SmartDashboard.putBoolean("Compressor getPressureSwitchValve: ", mainCompressor.getPressureSwitchValue());
    }
}

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Compression extends SubsystemBase {
    private boolean running;
    private boolean onOffSolenoidSet;
    private boolean cubeConeSolenoidSet;
    private Compressor mainCompressor;
    private Solenoid onOffSolenoid;
    private Solenoid cubeConeSolenoid;

    public Compression() {
        mainCompressor = new Compressor(kCompressorModuleId, PneumaticsModuleType.CTREPCM);
        mainCompressor.disable();

        onOffSolenoid = new Solenoid(kCompressorModuleId, PneumaticsModuleType.CTREPCM, onOffSolenoidChannel);
        onOffSolenoid.set(false);

        cubeConeSolenoid = new Solenoid(kCompressorModuleId, PneumaticsModuleType.CTREPCM, cubeConeSolenoidChannel);
        cubeConeSolenoid.set(false);

        running = false;
        onOffSolenoidSet = false;
        cubeConeSolenoidSet = false;
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

    public void grabCube(){
        cubeConeSolenoid.set(true);
        onOffSolenoid.set(true);
    }
    public void grabCone(){
        cubeConeSolenoid.set(false);
        onOffSolenoid.set(true);
    }

    public void release(){
        cubeConeSolenoid.set(false);
        onOffSolenoid.set(false);
    }

    private void printStatus() {
        System.out.println("\n\nShould be running: " + running);
        System.out.println("Solenoid should be enabled(): " + onOffSolenoidSet);
        System.out.println("Compressor isEnabled: " + mainCompressor.isEnabled());
        System.out.println("Compressor getPressureSwitchValve: " + mainCompressor.getPressureSwitchValue());
        System.out.println("Solenoid isEnabled(): " + !onOffSolenoid.isDisabled());
    }
}

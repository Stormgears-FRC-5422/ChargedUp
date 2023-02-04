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


 import edu.wpi.first.networktables.NetworkTableEntry;
 import edu.wpi.first.wpilibj.Compressor;
 import edu.wpi.first.wpilibj.CompressorConfigType;
 import edu.wpi.first.wpilibj.PneumaticsModuleType;
 import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
 import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import frc.robot.Robot;
 // import org.usfirst.frc5422.Minimec.commands.Pneumatics.RunCompressor;
 import frc.utils.configfile.StormProp;
 // import org.usfirst.frc5422.Minimec.Robot;

 public class Compression extends SubsystemBase {
     private boolean running;
     public Compressor mainCompressor;
    
     public Compression() {
         mainCompressor = new Compressor(StormProp.getInt("CompressorModuleId",-1), PneumaticsModuleType.CTREPCM);
         
         running = false;
     }

    // @Override
    // public void initDefaultCommand() {
    //     setDefaultCommand(new RunCompressor());
    // }

    // @Override
    // public void periodic() {

    // }

    // public boolean isActiveAndCharged() {
    //     return (running && mainCompressor.getPressureSwitchValue());
    // }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
     public void startCompressor() {
         running = true;


         mainCompressor.enableDigital();
//
//         if (Robot.debug) System.out.println("Compressor status: " + mainCompressor.isEnabled());
//         if (Robot.debug) System.out.println("Compressor pressure switch value: " + mainCompressor.getPressureSwitchValue());
//         if (Robot.debug) System.out.println("Compressor current (amps): " + mainCompressor.getCurrent());
     }

     public void stopCompressor(){
         running = false;
        
         mainCompressor.disable();
         if (Robot.debug) System.out.println("Compressor off: " + !mainCompressor.isEnabled());

     }
 }
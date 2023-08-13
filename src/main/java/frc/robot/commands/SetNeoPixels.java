package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NeoPixels;
import frc.utils.joysticks.StormXboxController;

public class SetNeoPixels extends CommandBase {
    StormXboxController xboxController;
    NeoPixels neoPixels;

    public SetNeoPixels(StormXboxController xboxController, NeoPixels neoPixels) {
        this.xboxController = xboxController;
        this.neoPixels = neoPixels;
        addRequirements(neoPixels);

    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        if ( xboxController.getAButtonIsHeld() ) {
            neoPixels.setColor();
        }
        else {
            neoPixels.setNullColor();
        }

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.NeoPixels;
import frc.utils.joysticks.StormXboxController;
import frc.utils.lights.LightColors;
import java.util.Timer;

public class SetNeoPixels extends CommandBase {
    String color = "";
    StormXboxController xboxController;
    NeoPixels neoPixels;

    public SetNeoPixels(StormXboxController xboxController, NeoPixels neoPixels) {
        this.xboxController = xboxController;
        this.neoPixels = neoPixels;
        addRequirements(neoPixels);

    }

    @Override
    public void initialize() {
        color = "red";
    }

    @Override
    public void execute() {
        neoPixels.setColor(color);
        switch (color) {
            case "red":
                color = "orange";
                break;
            case "orange":
                color = "yellow";
                break;
            case "yellow":
                color = "green";
                break;
            case "green":
                color = "blue";

                break;
            case "blue":
                color = "indigo";
                break;
            case "indigo":
                color = "violet";
                break;
            case "violet":
                color = "red";
                break;
        }

    }

    @Override
    public void end(boolean interrupted) {
        neoPixels.setNullColor();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}

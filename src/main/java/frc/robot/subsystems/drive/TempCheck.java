package frc.robot.subsystems.drive;
import frc.robot.constants.Constants;
import java.util.*;

import static frc.robot.constants.Constants.*;
import static java.lang.Math.max;
import static java.lang.Math.min;

public class TempCheck {
    private static final double temperatureRampThreshold = kTemperatureRampThreshold;
    private static final double temperatureRampLimit = kTemperatureRampLimit;
    private final double delta = min(temperatureRampLimit-temperatureRampThreshold,1.0);
    public double scale;


    HashMap<Integer, Double> tempMap = new HashMap<Integer, Double>();
    public void setTemperature(int id, double temperature) {
        tempMap.put(id, temperature);
    }

    public double checkTemp(double speed){
        Collection<Double> values = tempMap.values();
        double maxValue = Collections.max(values);
        if (maxValue>temperatureRampThreshold ) {
            scale = max((temperatureRampLimit-maxValue)/delta, 0.0);
        }
        return scale * speed;
    }




}

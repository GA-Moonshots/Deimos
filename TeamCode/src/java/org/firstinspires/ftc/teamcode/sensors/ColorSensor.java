package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensor {
    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor;
    private double multiplier;

    public ColorSensor(HardwareMap hardwareMap, double multiplier) {
        this.multiplier = multiplier;
        colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
    }

    public double getIntensity() {
        return (getRed() + getGreen() + getBlue()) / 3;
    }
    public int getRed() {
        return colorSensor.red();
    }
    public int getGreen() {
        return colorSensor.green();
    }
    public int getBlue() {
        return colorSensor.blue();
    }
    public boolean isRed() {
        return (getRed() > getGreen() && getRed() > getBlue());
    }
    public boolean isGreen() {
        return (getGreen() > getRed() && getGreen() > getBlue());
    }
    public boolean isBlue() {
        return (getBlue() > getRed() && getBlue() > getGreen());
    }
    public double getDistance() {
        if (colorSensor instanceof DistanceSensor) {
            return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        }
        return -1;
    }
}

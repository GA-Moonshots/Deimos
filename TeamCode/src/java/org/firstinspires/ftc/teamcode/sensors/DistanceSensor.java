package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    private Rev2mDistanceSensor distanceSensor;

    public DistanceSensor(HardwareMap hardwareMap, int id) {
        com.qualcomm.robotcore.hardware.DistanceSensor converter;
        converter = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, id + "_distance");
        distanceSensor = (Rev2mDistanceSensor)converter;
    }

    public DistanceSensor(HardwareMap hardwareMap, String id) {
        com.qualcomm.robotcore.hardware.DistanceSensor converter;
        converter = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, id + "_distance");
        distanceSensor = (Rev2mDistanceSensor)converter;
    }

    public double getDistance() {
        return getDistance(DistanceUnit.INCH);
    }

    public double getDistance(DistanceUnit unit) {
        return distanceSensor.getDistance(unit);
    }
}

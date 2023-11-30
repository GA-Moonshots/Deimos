package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Elevator {
    private DcMotor motor;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotor.class, Constants.ELEVATOR_MOTOR_NAME);
    }

    public void move(double str) {
        motor.setPower(str);
    }

    public void stop() {
        motor.setPower(0.0d);
    }
}

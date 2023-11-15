package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CascadeArm {
    public static final int MAXIMUM = -2200;
    public static final int MINIMUM = 0;
    public static final double POWER = 0.9;
    protected DcMotor motor;
    protected Telemetry telemetry;

    public CascadeArm(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotor.class, "cascade");
        this.telemetry = telemetry;
    }
    public void move(double strength){
        if(telemetry != null) {
            telemetry.addData("Arm Pos", motor.getCurrentPosition());
            telemetry.addData("Arm Power", motor.getPower());
        }

        int position = motor.getCurrentPosition();
        if((strength < 0 && position >= MAXIMUM) || (strength > 0 && position <= MINIMUM)) motor.setPower(strength * POWER);
        else motor.setPower(0);
    }



}
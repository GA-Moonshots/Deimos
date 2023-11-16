package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RailRocket {
    // telemetry is neeeded for robotics stuff
    protected Telemetry telemetry;
    // static variables can't go in the method, must go at top
    // final = unchangablE
    public static final double POWER = 1;
    // the position of the motor when opened
    public static final double OPENED_POSITION = 1;
    // the position of the motor when closed
    public static final double CLOSED_POSITION = 0;
    // these types are all caps "POWER" - standard

    /*
    There should be a code to make sure that the servo is pulling way (to take the rod out) on
    comand + hold the rilingsand place corpse in place .
    Servo do something
    This file will be all coded by Ed + Stacy. Last modified 10/20/2023 11:54am worked on comments and basically just started
     */

    // there is already a Servo class :) - turtle
    private Servo servo;

    // when creating a constructor for the robot these parameters are standard for most of this stuff
    public RailRocket(HardwareMap hardwareMap, Telemetry telemetry){
        // need the given telemetry because robotics
        this.telemetry = telemetry;

        // look for a servo with a release pin and set this servo equal to that servo
        this.servo = hardwareMap.get(Servo.class, "releasePin");
        // at this point we have the servo, time to make it do stuff :)

    }
    // create a method that can be called that moves the motor. It shouldn't take in or return any values.
    public void release(){
        servo.setPosition(OPENED_POSITION);
    }
    public void reset(){
        servo.setPosition(CLOSED_POSITION);
    }




}

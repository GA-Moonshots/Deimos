package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class Launcher {
    private LinearOpMode opMode;
    // static variables can't go in the method, must go at top
    // the position of the motor when opened
    public static final double OPENED_POSITION = 0;
    // the position of the motor when closed
    public static final double CLOSED_POSITION = 1;
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
    public Launcher(LinearOpMode opMode){
        this.opMode = opMode;

        // look for a servo with a release pin and set this servo equal to that servo
        servo = opMode.hardwareMap.get(Servo.class, Constants.LAUNCHER_SERVO_NAME);
        lock();
        // at this point we have the servo, time to make it do stuff :)

    }
    // create a method that can be called that moves the motor. It shouldn't take in or return any values.
    public void release(){
        servo.setPosition(OPENED_POSITION);
    }
    public void lock(){
        servo.setPosition(CLOSED_POSITION);
    }
}

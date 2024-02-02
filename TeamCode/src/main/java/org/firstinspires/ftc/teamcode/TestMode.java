/*
 * TestMode Class - FTC Robot Test OpMode
 *
 * This class represents a test OpMode for a FTC (First Tech Challenge) robot. The OpMode allows
 * testing of various robot subsystems, including MecanumDrive, PIDController, and Elevator.
 *
 * Author: [Everyone who worked on it]
 * Last Modified: 12/8/2023 12:53pm
 * Version: [self explanatory but idk what it is]
 *
 * Class Hierarchy:
 *   - TestMode
 *       - LinearOpMode
 *       - MecanumDrive
 *       - PIDController
 *       - Elevator
 *       - Arm
 *       - Camera
 *
 * Fields:
 *   - MecanumDrive drive: MecanumDrive instance for controlling the robot's mecanum drive system.
 *
 * Methods:
 *   - runOpMode(): Entry point for the OpMode, where specific test methods can be called.
 *   - driveTest(): Method for testing the mecanum drive system and IMU angles.
 *   - pidTest(): Method for testing the PID controller with the mecanum drive system.
 *   - elevatorTest(): Method for testing the elevator system and mecanum drive integration.
 *
 * Note: The OpMode contains specific test methods for different robot subsystems. The selected test
 * method can be changed in the runOpMode() method.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.systems.Elevator;
import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.sensors.PIDController;

import java.io.IOException;

@TeleOp(name = "Test Mode")
public class TestMode extends LinearOpMode {

    MecanumDrive drive;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(this);
        // Switch this to the test that we want
        imuTest();
    }

    public void imuTest() {
        while(opModeInInit()) {
            telemetry.addData("imu", drive.imu.getZAngle());
            telemetry.update();
        }
    }

    public void driveTest() {

        // LETS PREVENT OUR ROBOT FROM SHUTTING DOWN
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(this);

        while(opModeInInit()) {
            telemetry.addData("ang", drive.imu.getZAngle());
            telemetry.update();
            sleep(25);
        }


        this.terminateOpModeNow();

    }

    public void pidTest() {

        MecanumDrive drive = new MecanumDrive(this);
        PIDController controller = new PIDController(telemetry, "theta");
        telemetry.addData("IMU Rotation", "(%.2f, %.2f, %.2f)",
                drive.imu.getXAngle(), drive.imu.getYAngle(), drive.imu.getZAngle());
        double turnStrength = controller.getPIDControlledValue(Math.toRadians(drive.imu.getZAngle()), Math.PI / 2);
        telemetry.update();
        waitForStart();

        try {
            controller.resetPID();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        do {
            telemetry.addData("turnStrength", turnStrength);
            telemetry.update();
            drive.drive(0.0d, 0.0d, turnStrength);
            turnStrength = controller.getPIDControlledValue(Math.toRadians(drive.imu.getZAngle()), Math.PI / 2
            );
        } while(turnStrength >= 0.5 && opModeIsActive());
        try {
            controller.resetPID();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        drive.stop();
    }

    public void elevatorTest() {
        Elevator elevator = new Elevator(this);
        MecanumDrive drive = new MecanumDrive(this);
        waitForStart();

        boolean b = false;

        while(opModeIsActive()) {
            elevator.move(gamepad1.right_stick_y);
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if(gamepad1.b && !b && !gamepad1.start)
                elevator.toggleLock();
            b = gamepad1.b;
        }
    }
}

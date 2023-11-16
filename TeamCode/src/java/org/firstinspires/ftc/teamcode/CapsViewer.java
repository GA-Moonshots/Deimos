package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.teamcode.drives.Drivetrain;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.drives.SwerveDrive;
import org.firstinspires.ftc.teamcode.sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.teamcode.wrappers.IMU;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.IOException;
import java.util.List;

@TeleOp(name = "Capability Viewer")
public class CapsViewer extends LinearOpMode {
    private Drivetrain drive;
    private Camera camera;
    private DistanceSensor rearDistance;
    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;


    private boolean next = false;
    private boolean prev = false;
    private boolean runDrive = false;
    private boolean lastUpDPad = false;

    public enum DriveType {
        SWERVE,
        MECANUM,
        TANK,
        NONE
    }

    public enum Page {
        IMU_VIEW,
        SENSOR_VIEW,
        CV_VIEW,
        SERVO_VIEW,
    }

    public enum CameraState {
        AVAILABLE,
        UNAVAILABLE
    }

    private DriveType driveType = DriveType.NONE;
    private CameraState cameraState = CameraState.UNAVAILABLE;

    private Page currentPage = Page.IMU_VIEW;
    private Page nextPage = Page.SENSOR_VIEW;
    private Page prevPage = Page.CV_VIEW;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        IMU imu;
        try {
            try {
                drive = new SwerveDrive(hardwareMap, telemetry);
                driveType = DriveType.SWERVE;

            } catch (Exception e) {
                drive = new MecanumDrive(hardwareMap, telemetry);
                driveType = DriveType.MECANUM;
            }
            imu = drive.getIMU();
            camera = drive.getCamera();
            cameraState = CameraState.AVAILABLE;
        } catch (Exception e) {
            imu = new IMU(hardwareMap, new com.qualcomm.robotcore.hardware.IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            ));
            try {
                camera = new Camera(hardwareMap, telemetry);
                cameraState = CameraState.AVAILABLE;
            } catch (Exception f) {
                camera = null;
                cameraState = CameraState.UNAVAILABLE;
            }

            driveType = DriveType.NONE;
        }



        rearDistance = new DistanceSensor(hardwareMap, "rear");
        leftDistance = new DistanceSensor(hardwareMap, "left");
        rightDistance = new DistanceSensor(hardwareMap, "right");

        waitForStart();

        while(opModeIsActive()) {
            switch(driveType) {
                case SWERVE:
                    telemetry.addData("Drive Type", "Swerve");
                    break;
                case MECANUM:
                    telemetry.addData("Drive Type", "Mecanum");
                    break;
                case TANK:
                    telemetry.addData("Drive Type", "Tank");
                    break;
                case NONE:
                    telemetry.addData("Drive Type", "Not Found");
                    break;
            }

            switch(currentPage) {
                case IMU_VIEW:
                    telemetry.addData("Page", "IMU");
                    telemetry.addData("Angle", "(%.2f, %.2f, %.2f)", imu.getXAngle(), imu.getYAngle(), imu.getZAngle());
                    telemetry.addData("Position", "(%.2f, %.2f, %.2f)", imu.getXPosition(), imu.getYPosition(), imu.getZPosition());
                    telemetry.addData("Velocity", "(%.2f, %.2f, %.2f)", imu.getXVelocity(), imu.getYVelocity(), imu.getZVelocity());
                    prevPage = Page.SERVO_VIEW;
                    nextPage = Page.SENSOR_VIEW;
                    break;
                case SENSOR_VIEW:
                    telemetry.addData("Page", "Sensors");
                    telemetry.addData("Rear Distance Sensor", rearDistance.getDistance());
                    telemetry.addData("Left Distance Sensor", leftDistance.getDistance());
                    telemetry.addData("Right Distance Sensor", rightDistance.getDistance());
                    prevPage = Page.IMU_VIEW;
                    nextPage = Page.CV_VIEW;
                    break;
                case CV_VIEW:
                    telemetry.addData("Page", "CV");
                    if(cameraState != CameraState.UNAVAILABLE) {
                        telemetry.addData("Camera", "Available");
                        List<AprilTagDetection> currentDetections = camera.getDetections();
                        telemetry.addData("# AprilTags Detected", currentDetections.size());

                        for (AprilTagDetection detection : currentDetections) {
                            if (detection.metadata != null) {
                                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                            } else {
                                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                            }
                        }
                        prevPage = Page.SENSOR_VIEW;
                        nextPage = Page.SERVO_VIEW;
                    } else
                        telemetry.addData("Camera", "Unavailable");
                    break;
                case SERVO_VIEW:
                    telemetry.addData("Page", "Servo");
                    prevPage = Page.CV_VIEW;
                    nextPage = Page.IMU_VIEW;
                    break;
            }

            if(drive != null && runDrive) {
                telemetry.addData("Drive", "Listening to LSX, LSY, RSX");
                double forward = gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                // DEADZONES
                if (Math.abs(forward) <= Constants.INPUT_THRESHOLD) forward = 0.0d;
                if (Math.abs(strafe) <= Constants.INPUT_THRESHOLD)  strafe = 0.0d;
                if (Math.abs(turn) <= Constants.INPUT_THRESHOLD) turn = 0.0d;

                drive.drive(forward, strafe, turn);
            } else if(drive != null) {
                drive.stop();
            }

            telemetry.update();

            if(gamepad1.right_bumper && ! next) {
                currentPage = nextPage;
            } else if(gamepad1.left_bumper && ! prev) {
                currentPage = prevPage;
            }

            next = gamepad1.right_bumper;
            prev = gamepad1.left_bumper;

            if(gamepad1.dpad_up && !lastUpDPad) {
                runDrive = !runDrive;
            }
            lastUpDPad = gamepad1.dpad_up;
        }
    }




}

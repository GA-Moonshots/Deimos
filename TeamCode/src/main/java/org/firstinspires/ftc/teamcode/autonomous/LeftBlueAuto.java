package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensors.DistanceSensor;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.vision.Camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Left Blue Autonomous")
public class LeftBlueAuto extends LinearOpMode {
    // SUBSYSTEMS
    private MecanumDrive drive;
    private Arm arm;

    private enum TargetPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(this);
        arm = new Arm(hardwareMap, null);

        waitForStart();

        // APPROACH THE RANDOMIZER
        drive.goToDistanceFromWall(20);

        // TURN UNTIL WE SEE SOMETHING
        drive.turnUntilWeSeeProp();

        drive.turnRobotByDegree(180);

        // nudge forward drop claw than get back to square
        drive.toggleFieldCentric();
        drive.nudge(-0.2);
        sleep(100);
        // lowers claw and drops pixel
        arm.goToPickUp();

        sleep(1000);

        arm.close();
        arm.travelMode();

        drive.nudge(0.3);

        while(Math.abs(drive.getIMU().getZAngle()) >= 1 && opModeIsActive()) {
            drive.drive(0.0, 0.0, Math.toRadians(drive.getIMU().getZAngle()));
        }
        drive.stop();

        drive.goToDistanceToWall(4);

        drive.strafeUntilWall(-0.3);
        // Turning fieldCentric back on at end of auto
        //TODO turn off fieldCentric all toghether?
        drive.toggleFieldCentric();


        /*
        while((drive.rearDistance.getDistance() >= 6 || drive.leftDistance.getDistance() >= 24) && opModeIsActive()) {
            telemetry.addData("Rear Distance", drive.rearDistance.getDistance());
            telemetry.addData("Rear Distance", drive.leftDistance.getDistance());
            telemetry.addData("IMU Angle", drive.getIMU().getZAngle());
            telemetry.addData("Inputs", "(%.2f, %.2f, %.2f)", (drive.rearDistance.getDistance() - 6),
                    -(drive.leftDistance.getDistance() - 24),
                    Math.toRadians(drive.getIMU().getZAngle()));
            telemetry.update();

            drive.drive(
                    Range.clip((drive.rearDistance.getDistance() - 6), -1, 1) / 4,
                    Range.clip((-(drive.leftDistance.getDistance() - 24)), -1, 1) / 4,
                    Math.toRadians(drive.getIMU().getZAngle()));
        }
        */
        // DONE: CLEAN UP
        drive.stop();
        drive.camera.shutdown();
        this.terminateOpModeNow();
        /*
        sleep(100);
        drive.toggleFieldCentric();
        while(Math.abs(drive.getIMU().getZAngle() + 90) >= 1 && opModeIsActive()) {
            drive.drive(-0.1, 0.0, Math.toRadians(drive.getIMU().getZAngle() + 90));
        }
        drive.toggleFieldCentric();
        drive.stop();

        while(drive.camera.getDetections().size() == 0 && opModeIsActive()) {
            drive.drive(-0.2, 0.0, 0.0);
        }
        drive.stop();

        TargetPosition targetPosition;

        if(targetAngle <= -22.5) {
            targetPosition = TargetPosition.RIGHT;
        } else if(targetAngle >= 22.5) {
            targetPosition = TargetPosition.LEFT;
        } else {
            targetPosition = TargetPosition.CENTER;
        }

        while(opModeIsActive()) {
            List<AprilTagDetection> detections = drive.camera.getDetections();
            AprilTagDetection activeDetection = null;
            for(AprilTagDetection detection : detections) {
                switch(targetPosition) {
                    case LEFT:
                        if(detection.metadata.name.toLowerCase().contains("left")) {
                            activeDetection = detection;
                        }
                        break;
                    case CENTER:
                        if(detection.metadata.name.toLowerCase().contains("center")) {
                            activeDetection = detection;
                        }
                        break;
                    case RIGHT:
                        if(detection.metadata.name.toLowerCase().contains("right")) {
                            activeDetection = detection;
                        }
                        break;
                }
            }
            if(activeDetection == null) {
                drive.drive(-0.15, 0.0, 0.0);
            } else {
                telemetry.addData("Detection (X, Y, Theta", "(%.2f, %.2f, %.2f", activeDetection.ftcPose.x, activeDetection.ftcPose.y, activeDetection.ftcPose.yaw);
                telemetry.update();
                drive.drive( -activeDetection.ftcPose.x / 5, 0, -Math.toRadians(activeDetection.ftcPose.yaw));
                if(Math.abs(activeDetection.ftcPose.x) <= 0.1 && Math.abs(Math.toRadians(activeDetection.ftcPose.yaw)) <= 0.1) {
                    break;
                }
            }
            // PX, PY, AZ
        }

        while(opModeIsActive() && drive.rearDistance.getDistance() >= 8) {
            drive.drive(0.0, -0.15, 0.0);
        }

        while(opModeIsActive() && drive.rightDistance.getDistance() <= 42) {
            drive.drive(-1.0, 0.0, 0.0);
        }

        while(opModeIsActive() && drive.rearDistance.getDistance() >= 1) {
            drive.drive(0.0, -0.2, 0.0);
        }

         */

    }
}

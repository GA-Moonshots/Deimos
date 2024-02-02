package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
    private boolean isLeft = false;
    private boolean isRed = false;
    private boolean goForBoard = false;
    private boolean gotoOpposite = false;
    private boolean isNear;

    private MecanumDrive drive;
    private Arm arm;

    private Camera.AprilTagToAlign align;
    private List<AprilTagDetection> detections;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(this);
        arm = new Arm(this);

        // query user about starting point and scoring agenda
        while(opModeInInit()) {
            // LEFT OR RIGHT?
            if (gamepad1.dpad_left)
                isLeft = true;
            else if (gamepad1.dpad_right)
                isLeft = false;
            // RED OR BLUE
            if (gamepad1.x)
                isRed = false;
            else if (gamepad1.b && !gamepad1.start)
                isRed = true;
            // 25 OR 45 ATTEMPT?
            if (gamepad1.y)
                goForBoard = true;
            else if (gamepad1.a && !gamepad1.start)
                goForBoard = false;
            // SHOW SELECTION
            telemetry.addData("Position", "%s Team, %s Side", isRed ? "Red" : "Blue", isLeft ? "Left" : "Right");
            telemetry.addData("Target Points", "%s", goForBoard ? "45" : "25");
            drive.postDistanceReadouts();
            telemetry.update();
        }
        isNear = isRed != isLeft;

        drive.makeFieldCentric();
        // lower claw, prepare to drop
        arm.wristTo(0.3);

        // approach the prop
        drive.gotoBackDistance(0.15, 25, 4);
        arm.rollTo(Constants.ROLL_UP);
        align = drive.faceTheProp(0.3, 2);
        arm.wristTo(Constants.WRIST_ON_GROUND);
        drive.makeRobotCentric();
        if(align == Camera.AprilTagToAlign.RIGHT)
            drive.autonomouslyDriveByTime(-0.2, 0.0, 0.0, 1);
        stop();
        // drop left pixel
        arm.openLeft();
        sleep(250);
        // lift claw to get out of the way
        arm.travelMode();
        if(isNear && align == Camera.AprilTagToAlign.RIGHT)
            drive.autonomouslyDriveByTime(0.2, 0.0, 0.0, 1);
        drive.makeFieldCentric();


        // approach the parking area and board
        if(isNear) {
            near();
        } else {
            far();
        }

        // go for the extra 20 points or just park?
        if(goForBoard) goForBoard();
        else if(isNear)
            parkInside();
         else parkOutside();
    }

    public void near() {
        // This needs to be in field centric mode since it is going to go backwards regardless of what
        // position the robot goes to in the faceTheProp method.
        drive.autonomouslyDriveByTime(0.6, isRed ? 0.2 : -0.2, 0.0, 1);

        // straighten out
        drive.goToZeroAngle();

        // back up to the wall the rest of the way
        drive.gotoBackDistance(4);
    }

    public void far() {
        // This needs to be in field centric mode since it is going to go backwards regardless of what
        // position the robot goes to in the faceTheProp method.
        drive.autonomouslyDriveByTime(0.6, 0.0, 0.0, 1);
        // straighten out
        drive.goToZeroAngle();
        // back up to starting wall
        drive.gotoBackDistance(.4,10,3);
        drive.goToZeroAngle();
        if(align != Camera.AprilTagToAlign.CENTER) {
            drive.autonomouslyDriveByTime(-0.5, 0.0, 0.0, 3);
        } else {
            double targetDistance;
            if(isRed) {
                targetDistance = drive.leftDistance.doubleCheckDistance();
                drive.gotoLeftDistance(10);
            } else {
                targetDistance = drive.rightDistance.doubleCheckDistance();
                drive.gotoRightDistance(10);
            }

            drive.autonomouslyDriveByTime(-0.3, 0.0, 0.0, 5);

            if(isRed) {
                drive.gotoLeftDistance(targetDistance);
                drive.autonomouslyDriveByTime(0, 0.6, 0, 1);
            } else {
                drive.gotoRightDistance(targetDistance);
                drive.autonomouslyDriveByTime(0, -0.6, 0, 1);
            }
        }

        drive.autonomouslyDriveByTime(0,isRed ? 0.6 : -0.6,0,3); // move past the prop
        drive.goToZeroAngle();

        if(isRed)
            drive.gotoRightDistance(.7, 24, 4);
        else
            drive.gotoLeftDistance(.7, 24, 4);
    }

    public void parkInside() {
        // park for 5 points
        drive.autonomouslyMove(0.3, 5,
                //boolean statement ? if true : if false
                isRed ? MecanumDrive.HowToMove.RIGHT : MecanumDrive.HowToMove.LEFT,
                4.75);
        arm.wristTo(Constants.WRIST_ON_GROUND);
        sleep(500);
    }

    public void parkOutside() {
        if(gotoOpposite)
            drive.autonomouslyDriveByTime(-0.3, 0.0, 0.0, 3.75);
        drive.goToZeroAngle();
        if(isRed)
            drive.gotoRightDistance(4);
        else
            drive.gotoLeftDistance(4);
        drive.autonomouslyDriveByTime(0.1, 0.0, 0.0, 3);
        arm.wristTo(Constants.WRIST_ON_GROUND);
        sleep(500);
    }

    public void goForBoard() {
        // approach board after running near()
        if(isNear){
            if(isRed)
                drive.gotoRightDistance(0.5, 30, 4);
            else
                drive.gotoLeftDistance(0.5, 30, 4);
        }
        // approach board after crossing the field
        else {
            drive.makeRobotCentric();
            if(isRed)
                drive.gotoLeftDistance(0.5, 25, 1);
            else
                drive.gotoRightDistance(0.5, 25, 1);
            drive.makeFieldCentric();
        }
        drive.gotoBackDistance(0.3, 20, 4);

        drive.gotoAngle(1, isRed ? 90 : -90, 2);

        drive.makeFieldCentric();
        // inch up to board to prepare to drop
        approachTag();

        // place pixel
        arm.open();

        sleep(100);

        // drop arm
        drive.makeRobotCentric();
        while(opModeIsActive() && !arm.goToPickUp()) {
            drive.drive(0.0, isNear ? -0.1 : 0.1, 0.0);
        }

        // if inside, move out of the way, if outside, the time is too close
        if(isNear) {
            drive.makeFieldCentric();
            drive.goToZeroAngle();
            drive.gotoBackDistance(5);
            parkInside();
        }
    }

    public void approachTag() {
        while(opModeIsActive()) {
            AprilTagDetection target = fetchTarget();
            telemetry.addData("imu", drive.imu.getZAngle() - (isRed ? 90 : -90));

            if(!arm.goToDropOff())
                telemetry.update();
            else if (target == null) {
                drive.drive(-0.2, 0.0, (drive.imu.getZAngle() - (isRed ? 90 : -90)) / 10);
                sleep(30);
                telemetry.update();
            } else {
                // fix our delta x value for alignment
                if(Math.abs(target.ftcPose.x) > Constants.DISTANCE_THRESHOLD) {
                    drive.drive((isRed ? 1 : -1) * target.ftcPose.x / 12, 0.0, 2 * Math.toRadians(drive.imu.getZAngle() - (isRed ? 90 : -90)));
                } else {
                    stop();
                    break;
                }
                updateTargetTelemetry(target);
            }

        }

        while(opModeIsActive() && !arm.goToDropOff()) {
            stop();
            telemetry.update();
        }

        drive.gotoAngle(isRed ? 90 : -90, 4);

        drive.makeRobotCentric();
        drive.gotoBackDistance(2, 2);
    }

    public AprilTagDetection fetchTarget() {
        detections = drive.camera.getDetections();

        for(AprilTagDetection april : detections)
            if(april.metadata.name.toLowerCase().contains(align.name().toLowerCase()))
                return april;

        return null;

    }
    public void updateTargetTelemetry(AprilTagDetection target) {
        telemetry.addData("target x", target.ftcPose.x);
        telemetry.addData("target y", target.ftcPose.y);
        telemetry.addData("target range", target.ftcPose.range);
        telemetry.addData("# of detects", detections.size());
        sleep(2);

    }

}

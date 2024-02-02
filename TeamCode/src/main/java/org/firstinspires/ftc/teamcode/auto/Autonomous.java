package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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


        int cameraID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraID);
        FtcDashboard.getInstance().startCameraStream(camera, 30);

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
        if(align != Camera.AprilTagToAlign.CENTER)
            drive.autonomouslyDriveByTime(-0.2, 0.0, 0.0, 0.5);
        stop();
        // drop left pixel
        arm.openLeft();
        sleep(250);
        // lift claw to get out of the way
        arm.travelMode();
        if(align != Camera.AprilTagToAlign.CENTER)
            drive.autonomouslyDriveByTime(0.2, 0.0, 0.0, 0.5);
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
        drive.autonomouslyDriveByTime(0.6, 0.0, 0.0, 1);

        // straighten out
        drive.goToZeroAngle();

        // back up to the wall the rest of the way
        drive.gotoBackDistance(4);
    }

    public void far() {
        // scoot back
        drive.autonomouslyDriveByTime(0.6, 0.0, 0.0, 0.5);
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
        MecanumDrive.HowToMove move = isRed ? MecanumDrive.HowToMove.RIGHT : MecanumDrive.HowToMove.LEFT;
        drive.autonomouslyMove(.7, 24, move, 4);
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
            drive.gotoBackDistance(0.12, 25, 4);
        }
        // approach board after crossing the field
        else {
            //drive.gotoAngle(90, 5);
            drive.makeRobotCentric();
            if(isRed)
                drive.gotoLeftDistance(0.5, 25, 1);
            else
                drive.gotoRightDistance(0.5, 25, 1);
            drive.makeFieldCentric();
        }

        drive.gotoAngle(1, isRed ? 90 : -90, 2);

        drive.makeFieldCentric();
        // inch up to board to prepare to drop
        approachTag();

        // place pixel
        arm.open();

        sleep(100);

        // drop arm
        while(opModeIsActive() && !arm.goToPickUp()) {
            telemetry.update();
        }

        // if inside, move out of the way, if outside, the time is too close
        if(isNear) {
            drive.makeRobotCentric();
            drive.gotoBackDistance(7);
            drive.makeFieldCentric();
            drive.goToZeroAngle();
            drive.gotoBackDistance(5);
            parkInside();
        }
    }

    public void approachTag(){
        // TODO: Currently this is an infinite loop, make a break case
        while(opModeIsActive()){
            arm.goToDropOff();
            AprilTagDetection target = fetchTarget();

            if (target == null) {
                drive.drive(-0.1, 0.0, 0.0);
                sleep(30);
            } else {
                // fix our delta x value for alignment
                if(Math.abs(target.ftcPose.x) > Constants.DISTANCE_THRESHOLD) {
                    drive.drive(-target.ftcPose.x / 10, 0.0, 0.0);
                } else {
                    stop();
                    break;
                }
                updateTargetTelemetry(target);
            }
        }

        while(opModeIsActive() && !arm.goToDropOff()) {
            stop();
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
        telemetry.update();
        sleep(2);

    }

}

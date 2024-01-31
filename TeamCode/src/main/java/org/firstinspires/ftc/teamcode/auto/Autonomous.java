package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.sensors.Camera;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends LinearOpMode {
    private boolean isLeft = false;
    private boolean isRed = false;
    private boolean goForBoard = false;
    private boolean gotoOpposite = false;

    private MecanumDrive drive;
    private Arm arm;

    private Camera.AprilTagToAlign align;
    private List<AprilTagDetection> detections;

    @Override
    public void runOpMode() throws InterruptedException {
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

        drive.makeFieldCentric();
        // lower claw, prepare to drop
        arm.wristTo((Constants.WRIST_ON_GROUND + Constants.WRIST_MIN) / 2);

        // approach the prop
        drive.gotoBackDistance(0.15, 25, 4);
        arm.toggleRoll();
        align = drive.faceTheProp(0.3, 2);
        arm.wristTo(Constants.WRIST_ON_GROUND);
        stop();
        // drop left pixel
        arm.openLeft();
        sleep(100);
        // lift claw to get out of the way
        arm.travelMode();
        // avoid dropping our second pixel
        arm.close();

        // approach the parking area and board
        if((isLeft && !isRed) || (!isLeft && isRed)) {
            near();
        } else {
            far();
        }

        // go for the extra 20 points or just park?
        if(goForBoard) goForBoard();
        else if((isLeft && !isRed) || (!isLeft && isRed))
            parkInside();
         else parkOutside();
    }

    public void near() {
        // This needs to be in field centric mode since it is going to go backwards regardless of what
        // position the robot goes to in the faceTheProp method.
        drive.autonomouslyDriveByTime(0.3, 0.0, 0.0, 2);

        // straighten out
        drive.goToZeroAngle();

        // back up to the wall the rest of the way
        drive.gotoBackDistance(4);
    }

    public void far() {
        // scoot back
        drive.autonomouslyDriveByTime(0.3, 0.0, 0.0, 1);
        // straighten out
        drive.goToZeroAngle();
        // back up to starting wall
        drive.gotoBackDistance(.6,10,1.5);
        // go along the far wall
        MecanumDrive.HowToMove move = isRed? MecanumDrive.HowToMove.LEFT
                : MecanumDrive.HowToMove.RIGHT;
        drive.autonomouslyMove(.6, 8, move, 1);
        // move away from starting wall, ready to strafe under the flappy bar
        drive.autonomouslyDriveByTime(-.7,0,0,2.5);

        // drive across the field
        double strafe = isRed? .7 : -.7;
        drive.autonomouslyDriveByTime(0,strafe,0,3.5); // move past the prop
        move = isRed ? MecanumDrive.HowToMove.RIGHT : MecanumDrive.HowToMove.LEFT;
        drive.autonomouslyMove(.7, 24, move, 4);

    }

    public void parkInside() {
        // park for 5 points
        drive.autonomouslyMove(0.3, 5,
                //boolean statement ? if true : if false
                isRed ? MecanumDrive.HowToMove.RIGHT : MecanumDrive.HowToMove.LEFT,
                4.75);
    }

    public void parkOutside() {
        if(gotoOpposite)
            drive.autonomouslyDriveByTime(-0.3, 0.0, 0.0, 3.75);
        drive.goToZeroAngle();
        if(isRed)
            drive.gotoRightDistance(4);
        else
            drive.gotoLeftDistance(4);
        drive.autonomouslyDriveByTime(0.1, 0.0, 0.0, 1);
    }

    public void goForBoard() {
        // approach board after running near()
        if((isLeft && !isRed) || (!isLeft && isRed)){
            if(isRed)
                drive.gotoRightDistance(24);
            else
                drive.gotoLeftDistance(24);
            drive.gotoBackDistance(0.12, 25, 4);

            // face the board
            MecanumDrive.HowToMove move = isRed ? MecanumDrive.HowToMove.ROTATE_LEFT
                    : MecanumDrive.HowToMove.ROTATE_RIGHT;
            drive.autonomouslyMove(.4, 90, move, 1.5);
        }
        // approach board after crossing the field
        else {
            drive.gotoAngle(90);
            drive.makeRobotCentric();
            if(isRed)
                drive.gotoLeftDistance(0.5, 25, 1);
            else
                drive.gotoRightDistance(0.5, 25, 1);
            drive.makeFieldCentric();
        }

        // get arm ready to release pixel
        while(opModeIsActive() && !arm.goToDropOff()) {
            telemetry.update();
        }

        // inch up to board to prepare to drop
        approachTag();

        // place pixel
        arm.open();

        // drop arm
        while(opModeIsActive() && !arm.goToPickUp()) {
            telemetry.update();
        }

        // if inside, move out of the way, if outside, the time is too close
    }

    public void approachTag(){
        AprilTagDetection target = fetchTarget();
        // TODO: Handle case if no tag is detected because just giving up is bad
        if (target == null) terminateOpModeNow();
        while(opModeIsActive()){
            // fix our delta x value for alignment
            if(Math.abs(target.ftcPose.x) > Constants.DISTANCE_THRESHOLD) {
                drive.autonomouslyDriveByTime(0.0, -target.ftcPose.x / 40, 0.0, 1);
                drive.goToZeroAngle();
            }
        }
        drive.gotoBackDistance(10, 2);
    }

    public AprilTagDetection fetchTarget() {
        detections = drive.camera.getDetections();

        for(AprilTagDetection april : detections){
            if(april.metadata.name.toLowerCase().indexOf(align.name().toLowerCase()) != -1)
                return april;
        }
        return null;

    }
}

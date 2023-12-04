package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue - RIGHT")
public class RightBlueAuto extends LinearOpMode {
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
        arm = new Arm(this);

        while(opModeInInit())
            drive.getSensorReadout();

        waitForStart();

        drive.gotoBackDistance(22);
        drive.faceTheProp(0.3, MecanumDrive.HowToMove.ROTATE_LEFT, 8);
        drive.makeRobotCentric();
        drive.autoDriveByTime(0.2, 0.0, 0.0, 0.5);
        arm.goToPickUp();
        // Ensure the arm opens
        arm.open();
        sleep(500);
        // Second Pixel Logic?
        arm.travelMode();
        drive.autoDriveByTime(-0.2, 0.0, 0.0, 0.5);
        drive.goToZero();
        drive.makeFieldCentric();

        // go forward to the middle
        drive.gotoBackDistance(4);
        drive.autoGoToPosition(0.2, 5, MecanumDrive.HowToMove.RIGHT, 5);
        drive.gotoBackDistance(50, 5);
        drive.autoGoToPosition(0.7, 5, MecanumDrive.HowToMove.LEFT, 5);

        // DONE: CLEAN UP
        drive.stop();
    }
}

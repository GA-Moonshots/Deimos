package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red - RIGHT")
public class RedRightAuto extends LinearOpMode {
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

        // approach the props
        drive.gotoBackDistance(0.2, 22, 3);
        drive.faceTheProp(0.3, MecanumDrive.HowToMove.ROTATE_LEFT, 8);

        // TODO: support completely robot-centric autonomous
        drive.makeRobotCentric();

        // nudge the piece and hope we get it on the line for 20 points
        // Forward is negative Mr. A...
        drive.autoDriveByTime(-0.2, 0.0, 0.0, 0.5);

        // place the pixel with a couple of redundant commands
        arm.goToPickUp();
        arm.open();
        sleep(500); // Ensure the arm opens
        arm.travelMode();

        // scoot back
        drive.autoDriveByTime(0.2, 0.0, 0.0, 0.5);

        // straighten out
        drive.goToZero();

        // TODO: support completely robot-centric autonomous
        drive.makeFieldCentric();

        // back up to the wall
        drive.gotoBackDistance(4);

        // park for 5 points
        drive.autoGoToPosition(0.3, 5, MecanumDrive.HowToMove.RIGHT, 4.75);
    }
}

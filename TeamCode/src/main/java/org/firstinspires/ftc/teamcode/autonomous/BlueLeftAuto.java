package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue - LEFT")
public class BlueLeftAuto extends LinearOpMode {
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

        // approach the prop
        drive.gotoBackDistance(0.1, 27, 4);
        stop();
        sleep(1000);
        terminateOpModeNow();


        drive.faceTheProp(0.3, MecanumDrive.HowToMove.ROTATE_RIGHT, 8);

        // TODO: support completely robot-centric autonomous
        drive.makeRobotCentric();

        // nudge the pixel and hope it gets onto the line
        // Forward is negative Mr. A...
        drive.autoDriveByTime(-0.2, 0.0, 0.0, 0.5);

        // double open claw and take a half a second to pray
        arm.goToPickUp();
        arm.open();
        sleep(500);

        // lift claw and scoot back
        arm.travelMode();
        drive.autoDriveByTime(0.2, 0.0, 0.0, 0.5);

        // straighten out
        drive.goToZero();

        // TODO: support completely robot-centric autonomous
        drive.makeFieldCentric();

        // back to wall
        drive.gotoBackDistance(4);

        // park for five points
        drive.autoGoToPosition(0.3, 5, MecanumDrive.HowToMove.LEFT, 5.5);

    }
}

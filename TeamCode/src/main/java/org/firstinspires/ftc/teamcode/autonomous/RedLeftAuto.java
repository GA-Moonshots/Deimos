package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red - LEFT")
public class RedLeftAuto extends LinearOpMode {
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
        drive.gotoBackDistance(22);
        drive.faceTheProp(0.3, MecanumDrive.HowToMove.ROTATE_RIGHT, 8);

        // TODO: support completely robot-centric autonomous
        drive.makeRobotCentric();

        // nudge the pixel and hope we get our 20 points
        drive.autoDriveByTime(0.2, 0.0, 0.0, 0.5);

        // when you don't trust your code, do it twice
        arm.goToPickUp();
        arm.open();
        sleep(500); // wait half a second

        // pack up the claw and scoot back
        arm.travelMode();
        drive.autoDriveByTime(-0.2, 0.0, 0.0, 0.5);

        // center out
        drive.goToZero();

        // sprinkle a little extra complication on things for flavor
        drive.makeFieldCentric();

        // JOURNEY AROUND THE DIVIDER TO PARK
        drive.gotoBackDistance(4);
        drive.autoGoToPosition(0.2, 5, MecanumDrive.HowToMove.LEFT, 5);
        drive.gotoBackDistance(50, 5); // move forward
        drive.autoGoToPosition(0.7, 5, MecanumDrive.HowToMove.RIGHT, 5);

    }
}

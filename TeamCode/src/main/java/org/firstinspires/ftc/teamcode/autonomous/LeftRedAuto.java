package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red - LEFT")
public class LeftRedAuto extends LinearOpMode {
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

        drive.fwdFromWall(20);

        drive.faceTheProp(0.3);

        drive.isFieldCentric = false;
        // nudge forward
        drive.nudge(-0.2);
        sleep(100);
        // lowers claw and drops pixel
        arm.goToPickUp();
        sleep(1000);

        arm.travelMode();

        // get back
        drive.nudge(0.3);

        drive.turnToZero();

        drive.isFieldCentric = true;

        drive.backUpToWall(4);

        drive.strafeUntilWall(-0.2);

        drive.fwdFromWall(50, 8);

        drive.strafeUntilWall(0.7);


        // DONE: CLEAN UP
        drive.stop();
        this.terminateOpModeNow();
    }
}

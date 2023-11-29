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

        waitForStart();

        drive.fwdFromWall(20);

        drive.faceTheProp();

        // DONE: CLEAN UP
        drive.stop();
        drive.camera.shutdown();
        this.terminateOpModeNow();
    }
}

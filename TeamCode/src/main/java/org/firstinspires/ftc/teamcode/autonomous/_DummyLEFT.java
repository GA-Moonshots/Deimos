package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = ".Dummy LEFT")
public class _DummyLEFT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(this);

        waitForStart();
        ElapsedTime rt = new ElapsedTime();
        //TODO: Use a drivetrain method
        if(opModeIsActive())
            drive.drive(0.0, -0.3, 0.0);
        while(opModeIsActive() && rt.seconds() <= 5)
            sleep(1);

        drive.stop();
    }
}

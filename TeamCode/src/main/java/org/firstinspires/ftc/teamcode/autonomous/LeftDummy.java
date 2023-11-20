package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Shoulder;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Left Dummy")
public class LeftDummy extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Shoulder shoulder = new Shoulder(hardwareMap, null);

        MecanumDrive drive = new MecanumDrive(hardwareMap, null);

        waitForStart();
        ElapsedTime rt = new ElapsedTime();
        if(opModeIsActive())
            drive.drive(0.0, -0.3, 0.0);
        while(opModeIsActive() && rt.seconds() <= 5)
            sleep(1);

        drive.stop();
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;
import org.firstinspires.ftc.teamcode.systems.Arm;
import org.firstinspires.ftc.teamcode.wrappers.PIDController;

import java.io.IOException;

@TeleOp(name = "Test Mode")
public class TestMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armTest();
    }

    public void pidTest() {

        MecanumDrive drive = new MecanumDrive(this);
        PIDController controller = new PIDController(telemetry, "theta");
        telemetry.addData("IMU Rotation", "(%.2f, %.2f, %.2f)",
                drive.imu.getXAngle(), drive.imu.getYAngle(), drive.imu.getZAngle());
        double turnStrength = controller.getPIDControlledValue(Math.toRadians(drive.imu.getZAngle()), Math.PI / 2);
        telemetry.update();
        waitForStart();

        try {
            controller.resetPID();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        do {
            telemetry.addData("turnStrength", turnStrength);
            telemetry.update();
            drive.drive(0.0d, 0.0d, turnStrength);
            turnStrength = controller.getPIDControlledValue(Math.toRadians(drive.imu.getZAngle()), Math.PI / 2
            );
        } while(turnStrength >= 0.5 && opModeIsActive());
        try {
            controller.resetPID();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        drive.stop();
    }

    private void armTest() {
        Arm arm = new Arm(this);

    }

}

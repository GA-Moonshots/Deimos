/*
 * _DummyLEFT Class - FTC Robot Autonomous OpMode
 *
 * This class represents an autonomous OpMode for a FTC (First Tech Challenge) robot
 * The robot performs a simple movement routine during the autonomous period where the dummy is on the left.
 *
 * Author: Michael
 * Last Modified: 12/8/2023 12:42pm
 * Version: [self explanatory but idk what it is]
 *
 * Class Hierarchy:
 *   - _DummyLEFT
 *       - LinearOpMode
 *       - ElapsedTime
 *       - MecanumDrive
 *
 * Fields:
 *   - MecanumDrive drive: MecanumDrive instance for controlling the robot's mecanum drive system.
 *
 * Methods:
 *   - runOpMode(): Executes the autonomous routine, driving the robot forward for 5 seconds.
 *
 * Note: The specific details of the autonomous routine, such as the duration and direction of movement,
 * are described in the comments within the code. This is unfinished code (I think) so this whole doc may
 * be redone
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.MecanumDrive;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = ".Dummy LEFT")
public class _DummyLEFT extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(this);

        waitForStart();
        ElapsedTime rt = new ElapsedTime();
        drive.autonomouslyDriveByTime(0.0, -0.3, 0.0, 5);

        drive.stop();
    }
}

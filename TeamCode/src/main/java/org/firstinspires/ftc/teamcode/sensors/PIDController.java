/*
 * PIDController Class - FTC Robot Proportional-Integral-Derivative (PID) Controller
 *
 * This class represents a PID controller for use in FTC (First Tech Challenge) robotics. The PID controller
 * calculates the control signal based on the proportional, integral, and derivative components to minimize the
 * error between the desired setpoint and the current system state.
 *
 * Author: [everyone who worked on it]
 * Last Modified: 12:34pm
 * Version: [slef explanatory but idk what it is]
 *
 * Constants:
 *   - double PROPORTIONAL_CONSTANT: Proportional constant for the PID controller.
 *   - double INTEGRAL_CONSTANT: Integral constant for the PID controller.
 *   - double DERIVATIVE_CONSTANT: Derivative constant for the PID controller.
 *
 * Class Hierarchy:
 *   - PIDController
 *       - Telemetry
 *       - ElapsedTime
 *
 * Fields:
 *   - Telemetry telemetry: Telemetry instance for logging information.
 *   - ElapsedTime runtime: ElapsedTime instance for measuring time intervals.
 *   - double integralValue: Accumulated integral value for the PID controller.
 *   - double lastError: Previous error value for calculating the derivative component.
 *   - double finalError: Accumulated error value for telemetry reporting.
 *   - String name: Identifier for telemetry logging purposes.
 *
 * Constructors:
 *   - PIDController(Telemetry telemetry, String name): Initializes the PIDController with Telemetry and a name
 *     for telemetry logging.
 *
 * Methods:
 *   - void resetPID(): Resets the PID controller by clearing accumulated values and resetting runtime.
 *   - double getPIDControlledValue(double current, double target): Calculates the control signal using the PID
 *     algorithm and logs telemetry information.
 *
 * Note: Telemetry messages are added to provide debugging information during development and testing. They can
 * be removed or modified for deployment.
 */

package org.firstinspires.ftc.teamcode.sensors;

import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class PIDController {
    public static final double PROPORTIONAL_CONSTANT = .1;
    public static final double INTEGRAL_CONSTANT = .01;
    public static final double DERIVATIVE_CONSTANT = .1;

    private Telemetry telemetry;

    private ElapsedTime runtime = new ElapsedTime();
    private double integralValue = 0.0d;
    private double lastError = 0.0d;

    private double finalError = 0.0d;
    private String name;

    public PIDController(Telemetry telemetry, String name) {
        this.telemetry = telemetry;
        this.name = name;
    }

    public void resetPID() throws IOException {
        FileWriter writer = null;
        FileReader reader = null;
        String fileName = String.format("%s/FIRST/%s.csv", Environment.getExternalStorageDirectory().getPath(), name);
        try {
            writer = new FileWriter(fileName);
            reader = new FileReader(fileName);
            writer.append(String.format("%f,%f,%f,%f", PROPORTIONAL_CONSTANT, INTEGRAL_CONSTANT, DERIVATIVE_CONSTANT, finalError));
        } catch (IOException e) {
            throw new RuntimeException(e);
        } finally {
            if(writer != null) {
                writer.close();
            }
            if(reader != null) {
                reader.close();
            }
            integralValue = 0.0d;
            lastError = 0.0d;
            runtime.reset();
        }
    }

    public double getPIDControlledValue(double current, double target) {
        double error = target - current;
        // just curious how big these numbers are u know :/ )
        telemetry.addData("trg", target);
        telemetry.addData("crt", current);
        telemetry.addData("x", error);

        // WHY???
        integralValue += error * runtime.seconds();

        double dError = error - lastError;

        double p = PROPORTIONAL_CONSTANT * error;
        // Just calculate it into the constant instead of making it more complex.
        double i = integralValue / INTEGRAL_CONSTANT;
        //double i = integralValue / INTEGRAL_CONSTANT;
        double d = DERIVATIVE_CONSTANT * (dError / runtime.seconds());

        finalError += (Math.pow(p, 2) + Math.pow(i, 2) + Math.pow(d, 2))  / runtime.seconds();

        telemetry.addData("Vp", p);
        telemetry.addData("Vi", i);
        telemetry.addData("Vd", d);
        telemetry.addData("Et", finalError);

        lastError = error;

        runtime.reset();

        return p + i + d;
    }
}

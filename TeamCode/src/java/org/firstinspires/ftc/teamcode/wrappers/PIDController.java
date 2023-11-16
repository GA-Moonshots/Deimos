package org.firstinspires.ftc.teamcode.wrappers;

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

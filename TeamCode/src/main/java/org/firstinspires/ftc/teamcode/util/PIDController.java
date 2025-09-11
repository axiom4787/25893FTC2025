package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;

    private double integral;
    private double previousError;
    private ElapsedTime elapsedTime;

    private double previousTime;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0;
        this.previousError = 0;
        this.previousTime = 0;
        this.elapsedTime = new ElapsedTime();
    }

    public double calculate(double setpoint, double actual) {
        double currentTime = elapsedTime.seconds();
        double deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        double error = setpoint - actual;
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    public void reset() {
        this.integral = 0;
        this.previousError = 0;
        this.previousTime = elapsedTime.seconds();
    }
}

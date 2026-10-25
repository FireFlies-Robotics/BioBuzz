package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class PID {
    private LinearOpMode opMode;
    // Dashboard-tunable PID constants
    private  double KP;
    private  double KI;      // <-- new integral gain (start small!)
    private  double KD;
    public double kS = 0.05
            , kV, kA = 0; // Feedforward

    // Optional tuning parameters
    private static double maxPower;
    private static double INTEGRAL_LIMIT; // anti-windup limit
    private static double DEADZONE; // degrees


    private double lastError = 0;
    private double lastTime = 0;
    private double integralSum = 0;


    public PID(double KP, double KI, double KD, double MAX_POWER, double INTEGRAL_LIMIT, double DEADZONE, LinearOpMode opMode){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.maxPower = MAX_POWER;
        this.INTEGRAL_LIMIT = INTEGRAL_LIMIT;
        this.DEADZONE = DEADZONE;
        this.opMode = opMode;
    }
    public void setKF(double kS,double kV,double kA){
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }
    public PID(double KP, double KI, double KD, LinearOpMode opMode, double maxPower){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.maxPower = maxPower;
        this.opMode = opMode;
    }
    public void setPID(double kp, double ki, double kd){
        KP = kp;
        KI = ki;
        KD = kd;
    }
    public double calculatePIDValue(double position, double target){
        double error = position - target;

        double currentTime = opMode.getRuntime();
        double dt = currentTime - lastTime;
        if (dt <= 0) dt = 1e-3;

        // Deadzone
        if (Math.abs(error) < DEADZONE) error = 0;

        // Integral term (with anti-windup)
        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));

        // Derivative term
        double derivative = (error - lastError) / dt;

        double ff = kS * Math. signum(target)
                + kV * target
                + kA * position;
        // PID Control
        double power = KP * error + KI * integralSum + KD * derivative + ff;

        opMode.telemetry.addData("power", power);
        opMode.telemetry.addData("max power", maxPower);


        // Clamp output
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // Set servo power
        lastError = error;
        lastTime = currentTime;

        opMode.telemetry.addData("error", error);
        opMode.telemetry.addLine("calculatePIDValue");
        return power;


        // Update memory
//    }
//    public void printValues(){
//        pid.calculatePIDValue(lastError, 0);
//        opMode.telemetry.addData("Tag ID", detection.id);
//        opMode.telemetry.addData("Bearing (deg)", "%.2f", error);
//        opMode.telemetry.addData("Servo Power", "%.3f", power);
//        opMode.telemetry.addData("Integral Sum", "%.3f", integralSum);
//        opMode.telemetry.addData("Distance (m)", "%.2f", detection.ftcPose.range);

    }
}

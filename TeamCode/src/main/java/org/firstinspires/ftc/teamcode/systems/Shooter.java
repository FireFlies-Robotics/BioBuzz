package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Utils.PID;
@Config
public class Shooter {

    PID pid;
    public static double kp = 0.002;
    public static double ki = 0.0;
    public static double kd = 0.0001;



            public static double kS = 0.05, kV = 0.0001667 , kA = 0.000001; // Feedforward
    //todo find real KP
    private LinearOpMode opMode;
    public CRServo tunet;
    public CRServo houd;
    public DcMotorEx leftShotingMotor;
    public DcMotorEx rigtShotingMotor;


//    public static double TARGRT_VEL = 5000;

    private double oldShootingPosition;
    private VoltageSensor voltageSensor;
    double lastTime = 0;

    double lastVelocity = 0;



    public Shooter (LinearOpMode opMode) {

        this.opMode = opMode;
        pid = new PID(kp, ki, kd, opMode, 1.0);
        //     tunet = opMode.hardwareMap.get(CRServo.class, "tunet");
        leftShotingMotor = opMode.hardwareMap.get(DcMotorEx.class, "leftShootingMotor");
        rigtShotingMotor = opMode.hardwareMap.get(DcMotorEx.class, "rightShootingMotor");
        rigtShotingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShotingMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        voltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        //   feywhed = opMode.hardwareMap.get(DcMotor.class, "feywhed");
    }
    //    public void
    public void setShotingPower(double power) {
        leftShotingMotor.setPower(power);
        rigtShotingMotor.setPower(power);

    }
    public void shooterPID(double targetVel){
        double voltage = (voltageSensor !=null) ? voltageSensor.getVoltage() : 12.0;
        if (voltage <=0.0){voltage = 12.0;}

        pid.setPID(kp,ki, kd);
        pid.setKF(kS, kV, kA);
        double currentTime = opMode.getRuntime();
//        double dt = currentTixme - lastTime;

        double velocity = leftShotingMotor.getVelocity();


        double shootingPID = pid.calculatePIDValue(targetVel ,velocity);

        double ff =(
                kS * Math.signum(targetVel) +
                        kV * targetVel +
                        kA * (velocity - lastVelocity) / (currentTime - lastTime)) *12/voltage;



        shootingPID = Math.max(-1, Math.min(shootingPID, 1));
        setShotingPower((shootingPID + ff));
//        oldShootingPosition = rigtShotingMotor.getCurrentPosition()/28;
        lastTime = currentTime;
        lastVelocity = velocity;

        opMode.telemetry.addData("velocity", velocity);
        opMode.telemetry.addData("velocity", velocity);
        opMode.telemetry.addData("kp", kp);
        opMode.telemetry.addData("ki", ki);


        opMode.telemetry.addData("PID",shootingPID);
        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("FF", ff);
        packet.put("Velocity", velocity);
        packet.put("TargetVel", targetVel);


        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//        opMode.telemetry.update();
    }

//    private double calculateFF(){
//        double voltage = (voltageSensor != null)
//                ? voltageSensor.getVoltage() : 12.0;
//        if (voltage <= 0){
//            voltage = 12.0;
//            return  ff *12/voltage}
//    }
}
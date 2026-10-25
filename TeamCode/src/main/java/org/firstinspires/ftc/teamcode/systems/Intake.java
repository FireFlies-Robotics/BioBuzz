package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {
    private LinearOpMode opMode;
    public DcMotor intakeMotor;
    public DcMotor transfer;

//    public Servo transferServo;
    public DcMotor transferMotor;
//    public Servo transferServo;

    // Constructor: initialize the motor
    public Intake(LinearOpMode opMode) {
        this.opMode = opMode;
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "intakeMotor");
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//        transferServo = opMode.hardwareMap.get(Servo.class, "transferServo");

    }
    // Activate intake with a given power
    public void activateIntake(double power) {
        intakeMotor.setPower(power);
//        transfer.setPower(power);
    }
    // Stop intake
}

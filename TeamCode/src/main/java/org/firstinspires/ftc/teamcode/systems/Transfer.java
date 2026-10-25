package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Transfer {
    private LinearOpMode opMode;
    public DcMotor transferMotor;
    public Transfer(LinearOpMode opMode) {
        this.opMode = opMode;
        transferMotor = opMode.hardwareMap.get(DcMotor.class, "transferMotor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setTransferPower(double power){
        transferMotor.setPower(power);
    }

    // Stop intake
}

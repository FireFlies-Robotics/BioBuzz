package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Shooter;


@TeleOp(name = "shooter Control", group = "TeleOp")
@Config
@Disabled


public class ShooterCheck extends LinearOpMode {
    public static double targetVel = 2000;

    Shooter shooter;
    public static double hoodPosition;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(this);
        shooter.setShotingPower(0);
        waitForStart();
        while (opModeIsActive()){
            shooter.shooterPID(targetVel);
            if (gamepad1.triangle){targetVel = 2000;}
            if (gamepad1.cross){targetVel = 1000;}
            if (gamepad1.circle){targetVel = 1500;}

            if (gamepad1.square){
                shooter.setShotingPower(0);}





        }
    }
}
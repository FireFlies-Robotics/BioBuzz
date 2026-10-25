package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Transfer;

@TeleOp(name = "Intake Control", group = "TeleOp")
@Disabled
public class IntakeTeleop extends LinearOpMode {

    private Intake intake;

    @Override
    public void runOpMode() {
        // Initialize the intake system
        intake = new Intake(this);

        telemetry.addLine("Initialized — Ready to start");
        telemetry.update();

        // Wait for the start button
        waitForStart();

        while (opModeIsActive()) {
            // If square (X) button is pressed, run intake
            if (gamepad1.circle) {
                intake.activateIntake(1.0); // full power intake
            } else {
                intake.activateIntake(0); // stop when not pressed
            }

            telemetry.addData("Intake Power", intake.intakeMotor.getPower());
            telemetry.update();
        }
    }
}

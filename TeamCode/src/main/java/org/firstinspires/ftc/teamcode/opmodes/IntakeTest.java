package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Intake Test", group = "test")
public class IntakeTest extends LinearOpMode {
    public static double intakePower = 0.0;
    public static double intakePosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo intakeDeploy = hardwareMap.get(Servo.class, "intakeDeploy");

        waitForStart();

        while (!isStopRequested()) {
            intake.setPower(intakePower);
            intakeDeploy.setPosition(intakePosition);
        }
    }
}

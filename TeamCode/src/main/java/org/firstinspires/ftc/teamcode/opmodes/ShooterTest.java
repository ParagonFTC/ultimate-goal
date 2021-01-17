package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Shooter Test", group = "test")
public class ShooterTest extends LinearOpMode {
    public static double shooterPower = 0.0;
    public static double flapPosition = 0.5;
    public static double pusherPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        Servo flap = hardwareMap.get(Servo.class, "flap");
        Servo pusher = hardwareMap.get(Servo.class, "pusher");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Telemetry dashTelemetry = FtcDashboard.getInstance().getTelemetry();

        waitForStart();

        while (!isStopRequested()) {
            shooter.setPower(shooterPower);
            flap.setPosition(flapPosition);
            pusher.setPosition(pusherPosition);
            dashTelemetry.addData("Shooter Velocity", shooter.getVelocity(AngleUnit.RADIANS));
            dashTelemetry.update();
        }
    }
}

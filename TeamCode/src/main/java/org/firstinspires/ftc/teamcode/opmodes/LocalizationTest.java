package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.UltimateGoalLocalizationUtil;

@TeleOp(name = "Localization Test", group = "test")
public class LocalizationTest extends LinearOpMode implements DogeOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DogeCommander commander = new DogeCommander(this);
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Camera camera = new Camera(hardwareMap, drivetrain);

        commander.registerSubsystem(drivetrain);
        commander.registerSubsystem(camera);
        commander.init();

        drivetrain.setPoseEstimate(new Pose2d(-63,33,0));

        waitForStart();

        while (!isStopRequested()) {
            drivetrain.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            Pose2d drivetrainPoseEstimate = drivetrain.getPoseEstimate();
            Pose2d cameraPoseEstimate = camera.getPoseEstimate();
            telemetry.addData("Drivetrain Pose Estimate", drivetrainPoseEstimate);
            if (cameraPoseEstimate != null) telemetry.addData("Camera Pose Estimate", cameraPoseEstimate);
            telemetry.update();
        }
        commander.stop();
    }
}

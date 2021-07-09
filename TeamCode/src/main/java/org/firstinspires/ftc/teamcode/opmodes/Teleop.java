package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.commands.DriveTeleop;
import org.firstinspires.ftc.teamcode.commands.IntakeTeleop;
import org.firstinspires.ftc.teamcode.commands.ShooterTeleop;
import org.firstinspires.ftc.teamcode.commands.TeleopAutomation;
import org.firstinspires.ftc.teamcode.commands.TeleopAutomation2;
import org.firstinspires.ftc.teamcode.commands.WobbleGoalTeleop;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalGrabber;
import org.firstinspires.ftc.teamcode.util.OpmodeTransitionUtil;

@TeleOp
public class Teleop extends LinearOpMode implements DogeOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DogeCommander commander = new DogeCommander(this);
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        WobbleGoalGrabber wobbleGoalGrabber = new WobbleGoalGrabber(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        Camera camera = new Camera(hardwareMap, drivetrain);

        commander.registerSubsystem(drivetrain);
        commander.registerSubsystem(wobbleGoalGrabber);
        commander.registerSubsystem(intake);
        commander.registerSubsystem(shooter);
        commander.registerSubsystem(camera);
        commander.init();

        intake.deploy();
        if (OpmodeTransitionUtil.autoEndPose != null) drivetrain.setPoseEstimate(OpmodeTransitionUtil.autoEndPose);
        else drivetrain.setPoseEstimate(new Pose2d(6,36,0));
        drivetrain.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(12,0.5,6,11.782));

        while (!isStarted()) {
            if (camera.getPoseEstimate() != null) {
                double adjustedX = camera.getPoseEstimate().getX() - 7.67;
                double adjustedY = 1.2 * camera.getPoseEstimate().getY() - 8.97;
                drivetrain.setPoseEstimate(new Pose2d(adjustedX, adjustedY, camera.getPoseEstimate().getHeading()));
                telemetry.addLine("Tracking Target");
            }
            telemetry.update();
        }

        waitForStart();

        telemetry.clearAll();
        //camera.stopTracking();

        commander.runCommandsParallel(
                new WobbleGoalTeleop(wobbleGoalGrabber, gamepad1),
                new IntakeTeleop(intake, gamepad1),
                new TeleopAutomation2(camera, drivetrain, shooter, gamepad1, telemetry)
        );

        commander.stop();
    }
}

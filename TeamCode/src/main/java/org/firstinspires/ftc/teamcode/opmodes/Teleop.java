package org.firstinspires.ftc.teamcode.opmodes;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveTeleop;
import org.firstinspires.ftc.teamcode.commands.IntakeTeleop;
import org.firstinspires.ftc.teamcode.commands.ShooterTeleop;
import org.firstinspires.ftc.teamcode.commands.WobbleGoalTeleop;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalGrabber;

@TeleOp
public class Teleop extends LinearOpMode implements DogeOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DogeCommander commander = new DogeCommander(this);
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        WobbleGoalGrabber wobbleGoalGrabber = new WobbleGoalGrabber(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        commander.registerSubsystem(drivetrain);
        commander.registerSubsystem(wobbleGoalGrabber);
        commander.registerSubsystem(intake);
        commander.registerSubsystem(shooter);
        commander.init();

        waitForStart();

        commander.runCommandsParallel(
                new DriveTeleop(drivetrain, gamepad1),
                new WobbleGoalTeleop(wobbleGoalGrabber, gamepad1),
                new IntakeTeleop(intake, gamepad1),
                new ShooterTeleop(shooter, gamepad1)
        );

        commander.stop();
    }
}

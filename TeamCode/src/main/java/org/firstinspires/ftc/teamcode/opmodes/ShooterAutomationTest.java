package org.firstinspires.ftc.teamcode.opmodes;

import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ShooterTeleop;
import org.firstinspires.ftc.teamcode.commands.TeleopAutomation;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class ShooterAutomationTest extends LinearOpMode implements DogeOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DogeCommander commander = new DogeCommander(this);
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        Camera camera = new Camera(hardwareMap, drivetrain);
        Shooter shooter = new Shooter(hardwareMap);

        commander.registerSubsystem(drivetrain);
        commander.registerSubsystem(camera);
        commander.registerSubsystem(shooter);
        commander.init();

        waitForStart();

        commander.runCommandsParallel(
                new TeleopAutomation(camera, drivetrain, shooter, gamepad1, telemetry)
        );

        commander.stop();
    }
}

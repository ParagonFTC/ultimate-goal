package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.auto.Delay;
import org.firstinspires.ftc.teamcode.commands.DriveTrajectory;
import org.firstinspires.ftc.teamcode.commands.DriveTurn;
import org.firstinspires.ftc.teamcode.commands.auto.WaitForShooter;
import org.firstinspires.ftc.teamcode.subsystems.AutoCamera;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalGrabber;

@Config
@Autonomous
public class BlueAuto extends LinearOpMode implements DogeOpMode {
    public static double FLAP_POSITION = 0.435;
    public static double INITIAL_HEADING = -0.2;
    public static double HEADING_CHANGE = 0.12;

    @Override
    public void runOpMode() throws InterruptedException {
        DogeCommander commander = new DogeCommander(this);
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        final WobbleGoalGrabber wobbleGoalGrabber = new WobbleGoalGrabber(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);
        AutoCamera camera = new AutoCamera(hardwareMap);

        commander.registerSubsystem(drivetrain);
        commander.registerSubsystem(wobbleGoalGrabber);
        commander.registerSubsystem(intake);
        commander.registerSubsystem(shooter);
        commander.registerSubsystem(camera);
        commander.init();

        drivetrain.setPoseEstimate(new Pose2d(-63,33,0));

        while (!isStarted()) {
            telemetry.addData("Stack Size", camera.getStackSize());
            telemetry.update();
        }

        waitForStart();

        commander.runCommand(new DriveTrajectory(drivetrain, drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .forward(5)
                .build()
        ));

        commander.runCommand(new Delay(1));
        AutoCamera.StackSize stackSize = camera.getStackSize();
        telemetry.addData("Stack Size", stackSize);
        telemetry.update();
        shooter.setFlapPosition(FLAP_POSITION);
        shooter.setPower(1);

        commander.runCommand(new DriveTrajectory(drivetrain, drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(-24,12,0),0)
                .splineToSplineHeading(new Pose2d(-10,12,INITIAL_HEADING),INITIAL_HEADING)
                .build()
        ));

        commander.runCommand(new WaitForShooter(shooter));
        shooter.extendPusher();
        commander.runCommand(new Delay(1));
        shooter.retractPusher();
        commander.runCommand(new DriveTurn(drivetrain,HEADING_CHANGE));
        commander.runCommand(new WaitForShooter(shooter));
        shooter.extendPusher();
        commander.runCommand(new Delay(1));
        shooter.retractPusher();
        commander.runCommand(new DriveTurn(drivetrain, HEADING_CHANGE));
        commander.runCommand(new WaitForShooter(shooter));
        shooter.extendPusher();
        commander.runCommand(new Delay(1));
        shooter.setPower(0);
        shooter.setFlapPosition(0.9);

        switch (stackSize) {
            case QUAD:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(36,48,Math.PI/2),Math.PI/2)
                        .build()
                ));
                break;
            case SINGLE:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(12,30,Math.PI/2),Math.PI/2)
                        .build()
                ));
                break;
            case NONE:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(-10,48,Math.PI/2),Math.PI/2)
                        .build()
                ));
                break;
        }
        wobbleGoalGrabber.cycle();
        commander.runCommand(new Delay(1));
        wobbleGoalGrabber.setState(WobbleGoalGrabber.State.DROP);

/*
        intake.deploy();
        intake.setPower(-1);
        commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(),true)
                .lineToSplineHeading(new Pose2d(-24,36,0))
                .build()
        ));
        wobbleGoalGrabber.cycle();
        commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .forward(30)
                .build()
        ));
*/

        commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(),true)
                .lineToLinearHeading(new Pose2d(-29.5,54,-Math.PI/2))
                .build()
        ));
        wobbleGoalGrabber.cycle();
        commander.runCommand(new Delay(1));
        wobbleGoalGrabber.cycle();
        commander.runCommand(new Delay(1));

        switch (stackSize) {
            case QUAD:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(40,48,Math.PI/2))
                        .build()
                ));
                break;
            case SINGLE:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(16,30,Math.PI/2))
                        .build()
                ));
                break;
            case NONE:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-6,48,Math.PI/2))
                        .build()
                ));
                break;
        }
        wobbleGoalGrabber.cycle();

        commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(),true)
                .lineToSplineHeading(new Pose2d(6,36,0))
                .build()
        ));

        camera.shutdown();
        commander.stop();
    }
}

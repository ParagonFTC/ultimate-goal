package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.util.OpmodeTransitionUtil;

@Config
@Autonomous
public class BlueAuto extends LinearOpMode implements DogeOpMode {
    public static double FLAP_POSITION = 0.35;
    public static double INITIAL_HEADING = -0.125;
    public static double HEADING_CHANGE = 0.13;

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

        if (isStopRequested()) commander.stop();

        waitForStart();
/*
        commander.runCommand(new DriveTrajectory(drivetrain, drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .forward(5)
                .build()
        ));
*/
        //commander.runCommand(new Delay(1));
        AutoCamera.StackSize stackSize = camera.getStackSize();
        camera.shutdown();
        telemetry.addData("Stack Size", stackSize);
        telemetry.update();
        shooter.setFlapPosition(FLAP_POSITION);
        shooter.setPower(0.67);

        commander.runCommand(new DriveTrajectory(drivetrain, drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .splineTo(new Vector2d(-36,24),INITIAL_HEADING * 2)
                .splineTo(new Vector2d(-4,14),INITIAL_HEADING)
                .build()
        ));

        commander.runCommand(new DriveTrajectory(drivetrain, drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-4,14,INITIAL_HEADING))
                .build()
        ));

        commander.runCommand(new WaitForShooter(shooter));
        shooter.extendPusher();
        commander.runCommand(new Delay(0.5));
        shooter.retractPusher();
        commander.runCommand(new DriveTurn(drivetrain,HEADING_CHANGE));
        commander.runCommand(new WaitForShooter(shooter));
        shooter.extendPusher();
        commander.runCommand(new Delay(0.5));
        shooter.retractPusher();
        commander.runCommand(new DriveTurn(drivetrain, HEADING_CHANGE));
        commander.runCommand(new WaitForShooter(shooter));
        shooter.extendPusher();
        commander.runCommand(new Delay(0.5));
        shooter.retractPusher();
        shooter.setPower(0);

        switch (stackSize) {
            case QUAD:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(44,48,Math.PI/2),Math.PI/2)
                        .build()
                ));
                break;
            case SINGLE:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(12,24,Math.PI/2),Math.PI/2)
                        .build()
                ));
                break;
            case NONE:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(0,36,3*Math.PI/4),3*Math.PI/4)
                        .build()
                ));
                break;
        }
        wobbleGoalGrabber.autoCycle();
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

        switch (stackSize) {
            case QUAD:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(),true)
                        .addTemporalMarker(2.5, wobbleGoalGrabber::cycle)
                        .lineToLinearHeading(new Pose2d(-29.5,56,-Math.PI/2 + 0.15))
                        .build()
                ));
                break;
            case SINGLE:
                intake.deploy();
                commander.runCommand(new DriveTrajectory(drivetrain, drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                        .lineToSplineHeading(new Pose2d(-9,36,0))
                        .build()
                ));
                intake.setPower(-1);
                commander.runCommand(new DriveTrajectory(drivetrain, drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                        .splineToSplineHeading(new Pose2d(-18,36,0),0)
                        .build()
                ));
                intake.setPower(0);
                wobbleGoalGrabber.cycle();
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                        .lineToLinearHeading(new Pose2d(-29,54,-Math.PI/2))
                        .build()
                ));
                break;
            case NONE:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(-18,54,-Math.PI/4))
                        .build()
                ));
                wobbleGoalGrabber.cycle();
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(),true)
                        .lineToLinearHeading(new Pose2d(-29.5,54,-Math.PI/2))
                        .build()
                ));
                break;
        }
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
                shooter.setPower(0.7);
                shooter.setFlapPosition(0.367);
                commander.runCommand(new DriveTrajectory(drivetrain, drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(-1,33,0.1),0.1)
                        .build()
                ));
                commander.runCommand(new WaitForShooter(shooter));
                shooter.extendPusher();
                commander.runCommand(new Delay(1));
                shooter.retractPusher();
                shooter.setPower(0);
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(16,32,Math.PI/2))
                        .build()
                ));
                break;
            case NONE:
                commander.runCommand(new DriveTrajectory(drivetrain,drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-6,56,Math.PI/2))
                        .build()
                ));
                break;
        }
        wobbleGoalGrabber.autoCycle();
        commander.runCommand(new Delay(1));
        wobbleGoalGrabber.setState(WobbleGoalGrabber.State.DROP);

        if (stackSize == AutoCamera.StackSize.NONE) {
            commander.runCommand(new DriveTrajectory(drivetrain, drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                    .lineToConstantHeading(new Vector2d(-6, 40))
                    .build()
            ));
        }

        commander.runCommand(new DriveTrajectory(drivetrain, drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate(), true)
                .lineToSplineHeading(new Pose2d(6, 33, 0))
                .build()
        ));

        OpmodeTransitionUtil.autoEndPose = drivetrain.getPoseEstimate();

        commander.stop();
    }
}

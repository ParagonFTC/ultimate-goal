package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

@Config
public class TeleopAutomation implements Command {
    private Camera camera;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Gamepad gamepad;
    private StickyGamepad stickyGamepad;
    private Telemetry telemetry;
    private PIDFController TowerPID;
    public static PIDCoefficients TowerPIDCoefficients = new PIDCoefficients(0.6,0,0);
    public static double FlapPosition = 0.597;
    public static double FlapFixedPosition = 0.47908;

    private boolean slowMode = false;
    private boolean endgame = false;
    private boolean endgameStart = false;
    private int endgameRing = 1;

    public TeleopAutomation(Camera camera, Drivetrain drivetrain, Shooter shooter, Gamepad gamepad, Telemetry telemetry) {
        this.camera = camera;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.gamepad = gamepad;
        stickyGamepad = new StickyGamepad(gamepad);
        this.telemetry = telemetry;
        TowerPID = new PIDFController(TowerPIDCoefficients);
    }

    @Override
    public void start() {

    }

    @Override
    public void periodic() {
        boolean lastEndgame = endgame;
        stickyGamepad.update();
        if (endgame) {
            shooter.setPower(1);
            shooter.setFlapPosition(0.42);
            if (!endgameStart && !shooter.isShooting()) {
                endgameStart = true;
                if (endgameRing > 1) shooter.shootRing();
                switch (endgameRing) {
                    case 1:
                        drivetrain.followTrajectoryAsync(drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                                .strafeLeft(13.5)
                                .build()
                        );
                        break;
                    case 2:
                    case 3:
                        drivetrain.followTrajectoryAsync(drivetrain.trajectoryBuilder(drivetrain.getPoseEstimate())
                                .strafeLeft(10)
                                .build()
                        );
                        break;
                    default:
                        endgame = false;
                        break;
                }
            }
            if (endgameStart && drivetrain.isCompleted() && shooter.upToSpeed()) {
                shooter.shootRing();
                endgameStart = false;
                endgameRing++;
            }
        } else {
            /*
            if (!Double.isNaN(camera.getDistance())){
                shooter.setFlapPosition(-0.00176 * camera.getDistance() + FlapPosition);

            } else {
                shooter.setFlapPosition(0.9);
            }
            */
            shooter.setFlapPosition(FlapFixedPosition);
            if (gamepad.a && !Double.isNaN(camera.getTargetHeading())) {
                TowerPID.setTargetPosition(camera.getTargetHeading());
                drivetrain.setWeightedDrivePower(new Pose2d(0,0,TowerPID.update(camera.getCurrentHeading())));
            } else {
                TowerPID.reset();
                double velCoef = 1.0;
                if (slowMode) velCoef = 0.5;
                double xvel = velCoef * Math.pow(-gamepad.left_stick_y,3);
                double yvel = velCoef * Math.pow(-gamepad.left_stick_x,3);
                double thetavel = velCoef * Math.pow(-gamepad.right_stick_x,3);
                drivetrain.setWeightedDrivePower(new Pose2d(xvel, yvel, thetavel));
            }
            if (gamepad.b) {
                shooter.setPower(1);
                shooter.shootRing();
            } else {
                shooter.setPower(0);
            }
        }

        if (stickyGamepad.dpad_down) slowMode = !slowMode;
        if (stickyGamepad.dpad_left) endgame = !endgame;

        telemetry.addData("Current Heading", camera.getCurrentHeading());
        telemetry.addData("Target Heading", camera.getTargetHeading());
        telemetry.addData("Distance", camera.getDistance());
        telemetry.update();
    }

    @Override
    public void stop() {
        camera.stopTracking();
    }

    @Override
    public boolean isCompleted() {
        return false;
    }
}

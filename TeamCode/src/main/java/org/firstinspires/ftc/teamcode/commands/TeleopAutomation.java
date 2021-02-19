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

@Config
public class TeleopAutomation implements Command {
    private Camera camera;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Gamepad gamepad;
    private Telemetry telemetry;
    private PIDFController TowerPID;
    public static PIDCoefficients TowerPIDCoefficients = new PIDCoefficients(0.6,0,0);

    public TeleopAutomation(Camera camera, Drivetrain drivetrain, Shooter shooter, Gamepad gamepad, Telemetry telemetry) {
        this.camera = camera;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        TowerPID = new PIDFController(TowerPIDCoefficients);
    }

    @Override
    public void start() {

    }

    @Override
    public void periodic() {
        if (gamepad.a && camera.isTracking()) {
            TowerPID.setTargetPosition(camera.getTargetHeading());
            drivetrain.setWeightedDrivePower(new Pose2d(0,0,TowerPID.update(camera.getCurrentHeading())));
            shooter.setFlapPosition(-0.00118 * camera.getDistance() + 0.559);
        } else {
            drivetrain.setWeightedDrivePower(new Pose2d(-gamepad.left_stick_y,-gamepad.left_stick_x,-gamepad.right_stick_x));
        }
        double shooterVelocityError = Math.abs(5.35 - shooter.getShooterVelocity()) / 5.35;
        if (gamepad.b && shooterVelocityError < 0.01) {
            shooter.extendPusher();
        } else if (gamepad.b) {
            shooter.setPower(1);
            shooter.retractPusher();
        } else {
            shooter.setPower(0);
            shooter.retractPusher();
        }
        telemetry.addData("Heading Offset", camera.getCurrentHeading() - camera.getTargetHeading());
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

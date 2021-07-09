package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.util.UltimateGoalLocalizationUtil;

/**
 * TeleopAutomation was so good it needed a sequel
 */
@Config
public class TeleopAutomation2 implements Command {
    private Camera camera;
    private Drivetrain drivetrain;
    private Shooter shooter;
    private Gamepad gamepad;
    private StickyGamepad stickyGamepad;
    private Telemetry telemetry;
    private PIDFController TowerPID;
    public static PIDCoefficients TowerPIDCoefficients = new PIDCoefficients(0.6,0,0);

    public enum Mode {
        DRIVE,
        TOWER_GOAL_ALIGN,
        TOWER_GOAL_SHOOT,
        TURN_180,
        ENDGAME
    }
    private Mode mode = Mode.DRIVE;
    public boolean slowMode = false;
    private int ringCount = 3;

    public static double FlapFixedPosition = 0.363;
    public static double shooterSpeed = 0.7;

    public TeleopAutomation2(Camera camera, Drivetrain drivetrain, Shooter shooter, Gamepad gamepad, Telemetry telemetry) {
        this.camera = camera;
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.gamepad = gamepad;
        stickyGamepad = new StickyGamepad(gamepad);
        this.telemetry = telemetry;
        TowerPID = new PIDFController(TowerPIDCoefficients);
        TowerPID.setOutputBounds(-0.2,0.2);
    }

    @Override
    public void start() {

    }

    @Override
    public void periodic() {
        stickyGamepad.update();
        if (!Double.isNaN(camera.getDistance())) telemetry.addData("Distance", camera.getDistance());
        telemetry.update();

        switch (mode) {
            case TOWER_GOAL_SHOOT:
                if (ringCount > 0 && shooter.upToSpeed() && !shooter.isShooting()) {
                    shooter.shootRing();
                    ringCount --;
                } else if (ringCount == 0 && !shooter.isShooting()) {
                    shooter.setPower(0);
                    mode = Mode.DRIVE;
                }
            case DRIVE:
                double velCoef = 1.0;
                if (slowMode) velCoef = 0.5;
                double xvel = velCoef * Math.pow(-gamepad.left_stick_y,3);
                double yvel = velCoef * Math.pow(-gamepad.left_stick_x,3);
                double thetavel = velCoef * Math.pow(-gamepad.right_stick_x,3);
                drivetrain.setWeightedDrivePower(new Pose2d(xvel, yvel, thetavel));
                break;
            case TURN_180:
                if (drivetrain.isCompleted()) mode = Mode.DRIVE;
                break;
            case TOWER_GOAL_ALIGN:
                if (drivetrain.isCompleted()){
                    mode = Mode.TOWER_GOAL_SHOOT;
                }
                break;
        }
        shooter.setFlapPosition(FlapFixedPosition);

        if (stickyGamepad.dpad_down) slowMode = !slowMode;
        /*
        if (stickyGamepad.right_stick_button && mode == Mode.DRIVE) {
            mode = Mode.TURN_180;
            drivetrain.turnAsync(Math.PI);
        }
        */
        if (stickyGamepad.a && mode == Mode.DRIVE) {
            mode = Mode.TOWER_GOAL_ALIGN;
            //double targetHeading = UltimateGoalLocalizationUtil.angleToTowerGoal(drivetrain.getPoseEstimate().getX(), drivetrain.getPoseEstimate().getY()) - 0.12;
            //drivetrain.turnAsync(Angle.normDelta(targetHeading - drivetrain.getPoseEstimate().getHeading()));

            if (!Double.isNaN(camera.getTargetHeading()) && Math.abs(camera.getTargetHeading() - camera.getCurrentHeading()) < Math.PI/4) {
                drivetrain.turnAsync(camera.getTargetHeading() - camera.getCurrentHeading());
            }

            shooter.setPower(shooterSpeed);
            ringCount = 3;
        }
        if (stickyGamepad.b && mode == Mode.DRIVE) {
            mode = Mode.TOWER_GOAL_ALIGN;
            //double targetHeading = UltimateGoalLocalizationUtil.angleToTowerGoal(drivetrain.getPoseEstimate().getX(), drivetrain.getPoseEstimate().getY()) - 0.12;
            //drivetrain.turnAsync(Angle.normDelta(targetHeading - drivetrain.getPoseEstimate().getHeading()));

            if (!Double.isNaN(camera.getTargetHeading()) && Math.abs(camera.getTargetHeading() - camera.getCurrentHeading()) < Math.PI/4) {
                drivetrain.turnAsync(camera.getTargetHeading() - camera.getCurrentHeading());
            }

            shooter.setPower(shooterSpeed);
            ringCount = 1;
        }

        if (stickyGamepad.b && mode == Mode.TOWER_GOAL_SHOOT) ringCount++;
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean isCompleted() {
        return false;
    }
}

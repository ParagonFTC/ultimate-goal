package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.disnodeteam.dogecommander.DogeCommander;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Config
@TeleOp
public class ShooterTimingTest extends LinearOpMode implements DogeOpMode {
    private enum Mode {
        TESTING,
        IDLE
    }
    private Mode mode = Mode.IDLE;
    public static double SHOOTER_POWER;

    @Override
    public void runOpMode() throws InterruptedException {
        double startTimestamp = 0, ringShotTime = 0, shooterRecoverTime = 0;
        boolean lastShooterState = false;

        DogeCommander commander = new DogeCommander(this);
        Shooter shooter = new Shooter(hardwareMap);
        Telemetry combinedTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        NanoClock clock = NanoClock.system();

        commander.registerSubsystem(shooter);
        commander.init();

        shooter.setFlapPosition(0.43);

        waitForStart();

        while (!isStopRequested()) {
            shooter.setPower(SHOOTER_POWER);
            if (gamepad1.b && mode == Mode.IDLE && shooter.upToSpeed()) {
                startTimestamp = clock.seconds();
                shooter.extendPusher();
                mode = Mode.TESTING;
            }

            if (mode == Mode.TESTING) {
                if (!lastShooterState && !shooter.upToSpeed()) {
                    ringShotTime = clock.seconds() - startTimestamp;
                    lastShooterState = true;
                }
                if (lastShooterState && shooter.upToSpeed()) {
                    shooterRecoverTime = clock.seconds() - startTimestamp;
                    shooter.retractPusher();
                    mode = Mode.IDLE;
                    lastShooterState = false;
                }
            }
            combinedTelemetry.addData("Shooter Speed", shooter.getShooterVelocity());
            combinedTelemetry.addData("Shooter Current", shooter.getShooterCurrent());
            combinedTelemetry.addData("Ring Shot Time", ringShotTime);
            combinedTelemetry.addData("Shooter Recover Time", shooterRecoverTime);
            combinedTelemetry.update();
        }
        commander.stop();
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Shooter implements Subsystem {
    private HardwareMap hardwareMap;

    private DcMotorEx shooter;
    private Servo flap;
    private Servo pusher;

    private NanoClock clock;

    private enum State {
        SHOOTING,
        IDLE
    }
    private State shooterState = State.IDLE;

    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(10,0.1,5,11.782);

    public static double PUSHER_RETRACT_POSITION = 0.6; //0.85
    public static double PUSHER_EXTEND_POSITION = 0.8; //0.5
    public static double SHOOTER_MAX_VELO = 5.340707505;
    public static double RING_SHOT_TIME = 0.16; //0.3
    public static double SHOOTER_RECOVERY_TIME = 0.46; //0.9

    private double shooterPower = 0;
    private double flapPosition;
    private double pusherPosition;
    private double startTimestamp;

    public Shooter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void initHardware() {
        clock = NanoClock.system();

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        flap = hardwareMap.get(Servo.class, "flap");
        pusher = hardwareMap.get(Servo.class, "pusher");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTER_PID);
        flapPosition = 0.5;
        retractPusher();
    }

    public void setPower(double power) {
        shooterPower = power;
    }

    public void extendPusher() {
        pusherPosition = PUSHER_EXTEND_POSITION;
    }

    public void retractPusher() {
        pusherPosition = PUSHER_RETRACT_POSITION;
    }

    public void shootRing() {
        if (upToSpeed() && shooterState == State.IDLE) {
            startTimestamp = clock.seconds();
            shooterState = State.SHOOTING;
            extendPusher();
        }
    }

    public void setFlapPosition(double position) {
        flapPosition = position;
    }

    public double getShooterVelocity() {
        return shooter.getVelocity(AngleUnit.RADIANS);
    }

    public double getShooterCurrent() {
        return shooter.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * The shooter motor is configured to be a GoBILDA 5202 series motor, which has a max RPM in code of 60, despite its actual max RPM being 6000
     * From that, the default max achievable RPM fraction is 0.85, from which we get a max achievable velocity of 5.340707505 rad/s
     * Assuming that the power is linearly scaled with the velocity, the target velocity at any power is then 5.340707505 rad/s times the given power
     * @return whether the shooter's current velocity is within a small enough margin to the target velocity
     */
    public boolean upToSpeed() {
        if (shooterPower == 0) return true;
        double targetVelocity = SHOOTER_MAX_VELO * shooterPower;
        boolean shooterVelocityCheck = Math.abs(targetVelocity - getShooterVelocity())/targetVelocity <= 0.01;
        //boolean shooterCurrentCheck = Math.abs(0.57 - getShooterCurrent()) < 0.1;
        return shooterVelocityCheck;
    }

    public boolean isShooting() {
        return shooterState == State.SHOOTING;
    }

    @Override
    public void periodic() {
        pusher.setPosition(pusherPosition);
        shooter.setPower(shooterPower);
        flap.setPosition(flapPosition);

        if (shooterState == State.SHOOTING) {
            if (clock.seconds() - startTimestamp > RING_SHOT_TIME) retractPusher();
            if (clock.seconds() - startTimestamp > SHOOTER_RECOVERY_TIME) shooterState = State.IDLE;
        }
    }
}

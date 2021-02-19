package org.firstinspires.ftc.teamcode.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter implements Subsystem {
    private HardwareMap hardwareMap;

    private DcMotorEx shooter;
    private Servo flap;
    private Servo pusher;

    public static double PUSHER_RETRACT_POSITION = 0.55;
    public static double PUSHER_EXTEND_POSITION = 0.85;
    public static double SHOOTER_MAX_VELO = 5.340707505;

    private double shooterPower = 0;
    private double flapPosition;
    private double pusherPosition;

    public Shooter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void initHardware() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        flap = hardwareMap.get(Servo.class, "flap");
        pusher = hardwareMap.get(Servo.class, "pusher");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flapPosition = 0.9;
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

    public void setFlapPosition(double position) {
        flapPosition = position;
    }

    public double getShooterVelocity() {
        return shooter.getVelocity(AngleUnit.RADIANS);
    }

    /**
     * The shooter motor is configured to be a GoBILDA 5202 series motor, which has a max RPM in code of 60, despite its actual max RPM being 6000
     * From that, the default max achievable RPM fraction is 0.85, from which we get a max achievable velocity of 5.340707505 rad/s
     * Assuming that the power is linearly scaled with the velocity, the target velocity at any power is then 5.340707505 rad/s times the given power
     * @return whether the shooter's current velocity is within a small enough margin to the target velocity
     */
    public boolean upToSpeed() {
        double targetVelocity = SHOOTER_MAX_VELO * shooterPower;
        return Math.abs(targetVelocity - getShooterVelocity())/targetVelocity <= 0.01;
    }

    @Override
    public void periodic() {
        pusher.setPosition(pusherPosition);
        shooter.setPower(shooterPower);
        flap.setPosition(flapPosition);
    }
}

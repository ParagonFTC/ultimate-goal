package org.firstinspires.ftc.teamcode.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter implements Subsystem {
    private HardwareMap hardwareMap;

    private DcMotor shooter;
    private Servo flap;
    private Servo pusher;

    public static double PUSHER_RETRACT_POSITION = 0.55;
    public static double PUSHER_EXTEND_POSITION = 0.85;

    private double shooterPower = 0;
    private double flapPosition;
    private double pusherPosition;

    public Shooter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void initHardware() {
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        flap = hardwareMap.get(Servo.class, "flap");
        pusher = hardwareMap.get(Servo.class, "pusher");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flap.setPosition(0.47);
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

    @Override
    public void periodic() {
        pusher.setPosition(pusherPosition);
        shooter.setPower(shooterPower);
    }
}

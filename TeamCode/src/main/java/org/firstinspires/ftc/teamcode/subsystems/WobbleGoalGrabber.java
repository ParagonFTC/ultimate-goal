package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class WobbleGoalGrabber implements Subsystem {
    private Servo wristLeft, wristRight, grabber;
    private HardwareMap hardwareMap;

    public static final double GRABBER_OPEN_POSITION = 0.15;
    public static final double GRABBER_CLOSE_POSITION = 0.4;

    public static final double WRIST_EXTEND_POSITION = 0.42;
    public static final double WRIST_RETRACT_POSITION = 0.7;

    private double wristPosition = 1;
    private double grabberPosition = GRABBER_CLOSE_POSITION;

    public WobbleGoalGrabber (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void initHardware() {
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        grabber = hardwareMap.get(Servo.class, "grabber");

        wristRight.setDirection(Servo.Direction.REVERSE);
    }

    public void setGrabberPosition(double grabberPosition) {
        this.grabberPosition = grabberPosition;
    }

    public void setWristPosition(double wristPosition) {
        this.wristPosition = wristPosition;
    }

    public void openGrabber() {
        grabberPosition = GRABBER_OPEN_POSITION;
    }

    public void closeGrabber() {
        grabberPosition = GRABBER_CLOSE_POSITION;
    }

    public void extendArm() {
        wristPosition = WRIST_EXTEND_POSITION;
    }

    public void retractArm() {
        wristPosition = WRIST_RETRACT_POSITION;
    }

    @Override
    public void periodic() {
        wristLeft.setPosition(wristPosition);
        wristRight.setPosition(wristPosition);
        grabber.setPosition(grabberPosition);
    }
}

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class WobbleGoalGrabber implements Subsystem {
    private Servo wristLeft, wristRight, grabber;
    private HardwareMap hardwareMap;
    private NanoClock clock;

    public static final double GRABBER_OPEN_POSITION = 0.15;
    public static final double GRABBER_CLOSE_POSITION = 0.4;

    public static final double WRIST_GRAB_POSITION = 0.45;
    public static final double WRIST_TRANSPORT_POSITION = 0.7;
    public static final double WRIST_INIT_POSITION = 1;

    private double wristPosition = WRIST_INIT_POSITION;
    private double grabberPosition = GRABBER_CLOSE_POSITION;

    private double initialTimestamp;
    public static double TRANSITION_DELAY = 0.5;

    public enum State {
        INIT,
        GRAB,
        GRAB_TRANSPORT,
        TRANSPORT,
        DROP
    }

    private State state = State.INIT;

    public WobbleGoalGrabber (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void initHardware() {
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        grabber = hardwareMap.get(Servo.class, "grabber");
        clock = NanoClock.system();

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
        wristPosition = WRIST_GRAB_POSITION;
    }

    public void retractArm() {
        wristPosition = WRIST_TRANSPORT_POSITION;
    }

    public void cycle() {
        switch (state) {
            case INIT:
            case DROP:
                state = State.GRAB;
                break;
            case GRAB:
                initialTimestamp = clock.seconds();
                state = State.GRAB_TRANSPORT;
                break;
            case GRAB_TRANSPORT:
                state = State.TRANSPORT;
                break;
            case TRANSPORT:
                state = State.DROP;
                break;
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        switch (state) {
                case INIT:
                    wristPosition = WRIST_INIT_POSITION;
                    grabberPosition = GRABBER_CLOSE_POSITION;
                    break;
                case GRAB:
                    wristPosition = WRIST_GRAB_POSITION;
                    grabberPosition = GRABBER_OPEN_POSITION;
                    break;
                case GRAB_TRANSPORT:
                    wristPosition = WRIST_GRAB_POSITION;
                    grabberPosition = GRABBER_CLOSE_POSITION;
                    if (clock.seconds() - initialTimestamp > TRANSITION_DELAY) cycle();
                    break;
                case TRANSPORT:
                    wristPosition = WRIST_TRANSPORT_POSITION;
                    grabberPosition = GRABBER_CLOSE_POSITION;
                    break;
                case DROP:
                    wristPosition = WRIST_TRANSPORT_POSITION;
                    grabberPosition = GRABBER_OPEN_POSITION;
                    break;
        }
        wristLeft.setPosition(wristPosition);
        wristRight.setPosition(wristPosition);
        grabber.setPosition(grabberPosition);
    }
}

package org.firstinspires.ftc.teamcode.commands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalGrabber;

public class WobbleGoalTeleop implements Command {
    private WobbleGoalGrabber wobbleGoalGrabber;
    private Gamepad gamepad;

    public WobbleGoalTeleop (WobbleGoalGrabber wobbleGoalGrabber, Gamepad gamepad) {
        this.wobbleGoalGrabber = wobbleGoalGrabber;
        this.gamepad = gamepad;
    }

    @Override
    public void start() {
        wobbleGoalGrabber.closeGrabber();
        wobbleGoalGrabber.retractArm();
    }

    @Override
    public void periodic() {
        if (gamepad.a) {
            wobbleGoalGrabber.closeGrabber();
        } else if (gamepad.b) {
            wobbleGoalGrabber.openGrabber();
        }
        if (gamepad.x) {
            wobbleGoalGrabber.retractArm();
        } else if (gamepad.y) {
            wobbleGoalGrabber.extendArm();
        }
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean isCompleted() {
        return false;
    }
}

package org.firstinspires.ftc.teamcode.commands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalGrabber;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

public class WobbleGoalTeleop implements Command {
    private WobbleGoalGrabber wobbleGoalGrabber;
    private StickyGamepad gamepad;

    public WobbleGoalTeleop (WobbleGoalGrabber wobbleGoalGrabber, Gamepad gamepad) {
        this.wobbleGoalGrabber = wobbleGoalGrabber;
        this.gamepad = new StickyGamepad(gamepad);
    }

    @Override
    public void start() {

    }

    @Override
    public void periodic() {
        gamepad.update();
        if (gamepad.x) wobbleGoalGrabber.cycle();
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean isCompleted() {
        return false;
    }
}

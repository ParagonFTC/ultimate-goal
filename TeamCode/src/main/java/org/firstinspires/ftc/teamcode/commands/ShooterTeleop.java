package org.firstinspires.ftc.teamcode.commands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShooterTeleop implements Command {
    private Shooter shooter;
    private Gamepad gamepad;

    public ShooterTeleop(Shooter shooter, Gamepad gamepad) {
        this.shooter = shooter;
        this.gamepad = gamepad;
    }

    @Override
    public void start() {

    }

    @Override
    public void periodic() {
        if (gamepad.left_trigger > 0.5) {
            shooter.extendPusher();
        } else shooter.retractPusher();
        if (gamepad.left_bumper) shooter.setPower(1);
        else shooter.setPower(0.8);
    }

    @Override
    public void stop() {
        shooter.setPower(0);
    }

    @Override
    public boolean isCompleted() {
        return false;
    }
}

package org.firstinspires.ftc.teamcode.commands.auto;

import com.disnodeteam.dogecommander.Command;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * simple command to wait for the shooter to power up
 */
public class WaitForShooter implements Command {
    private Shooter shooter;

    public WaitForShooter(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void start() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean isCompleted() {
        return shooter.upToSpeed();
    }
}

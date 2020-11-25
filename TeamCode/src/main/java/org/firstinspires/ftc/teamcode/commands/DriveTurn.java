package org.firstinspires.ftc.teamcode.commands;

import com.disnodeteam.dogecommander.Command;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveTurn implements Command {
    private Drivetrain drivetrain;
    private double angle;

    public DriveTurn(Drivetrain drivetrain, double angle) {
        this.drivetrain = drivetrain;
        this.angle = angle;
    }

    @Override
    public void start() {
        drivetrain.turnAsync(angle);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean isCompleted() {
        return drivetrain.isCompleted();
    }
}

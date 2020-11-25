package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.disnodeteam.dogecommander.Command;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveTrajectory implements Command {
    private Drivetrain drivetrain;
    private Trajectory trajectory;

    public DriveTrajectory(Drivetrain drivetrain, Trajectory trajectory) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
    }

    @Override
    public void start() {
        drivetrain.followTrajectoryAsync(trajectory);
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

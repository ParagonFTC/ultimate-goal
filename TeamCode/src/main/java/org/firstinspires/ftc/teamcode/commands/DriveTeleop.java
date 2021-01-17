package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveTeleop implements Command {
    private Drivetrain drivetrain;
    private Gamepad gamepad;

    public DriveTeleop(Drivetrain drivetrain, Gamepad gamepad) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;
    }

    @Override
    public void start() {
        drivetrain.setDrivePower(new Pose2d());
    }

    @Override
    public void periodic() {
        drivetrain.setWeightedDrivePower(new Pose2d(-gamepad.left_stick_y,-gamepad.left_stick_x,-gamepad.right_stick_x));
    }

    @Override
    public void stop() {
        drivetrain.setDrivePower(new Pose2d());
    }

    @Override
    public boolean isCompleted() {
        return false;
    }
}

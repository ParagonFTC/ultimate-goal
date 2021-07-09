package org.firstinspires.ftc.teamcode.commands;

import com.disnodeteam.dogecommander.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeTeleop implements Command {
    private Intake intake;
    private Gamepad gamepad;

    public IntakeTeleop(Intake intake, Gamepad gamepad) {
        this.intake = intake;
        this.gamepad = gamepad;
    }

    @Override
    public void start() {
        intake.deploy();
    }

    @Override
    public void periodic() {
        intake.setPower(-gamepad.right_trigger);
        if (gamepad.right_bumper) {
            intake.setPower(1);
        }
        if (gamepad.left_trigger >= 0.5) {
            intake.halfDeploy();
        } else {
            intake.deploy();
        }
    }

    @Override
    public void stop() {
        intake.setPower(0);
    }

    @Override
    public boolean isCompleted() {
        return false;
    }
}

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.disnodeteam.dogecommander.DogeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalGrabber;

@Config
@TeleOp(name = "Wobble Goal Test", group = "test")
public class WobbleGoalTest extends LinearOpMode implements DogeOpMode {
    public static double wristPosition = 0.5;
    public static double grabberPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        WobbleGoalGrabber wobbleGoalGrabber = new WobbleGoalGrabber(hardwareMap);
        wobbleGoalGrabber.initHardware();

        waitForStart();

        while (!isStopRequested()) {
            wobbleGoalGrabber.setGrabberPosition(grabberPosition);
            wobbleGoalGrabber.setWristPosition(wristPosition);
            wobbleGoalGrabber.periodic();
        }
    }
}

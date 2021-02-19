package org.firstinspires.ftc.teamcode.commands.auto;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.disnodeteam.dogecommander.Command;

/**
 * Tfw DogeCommander doesn't work with the wait function in LinearOpMode
 */
public class Delay implements Command {
    private NanoClock clock;

    private double duration;
    private double startTimestamp;

    public Delay(double duration) {
        this.duration = duration;
        clock = NanoClock.system();
    }

    @Override
    public void start() {
        startTimestamp = clock.seconds();
    }

    @Override
    public void periodic() {

    }

    @Override
    public void stop() {

    }

    @Override
    public boolean isCompleted() {
        return clock.seconds() - startTimestamp > duration;
    }
}

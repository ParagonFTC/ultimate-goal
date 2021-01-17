package org.firstinspires.ftc.teamcode.subsystems;

import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake implements Subsystem {
    public static final double INTAKE_RETRACT_POSITION = 0;
    public static final double INTAKE_DEPLOY_POSITION = 0.5;

    private HardwareMap hardwareMap;

    private DcMotor intake;
    private Servo intakeDeploy;

    private double intakePower = 0;
    private double intakePosition = INTAKE_RETRACT_POSITION;

    public Intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void setPower(double power) {
        intakePower = power;
    }

    public void deploy() {
        intakePosition = INTAKE_DEPLOY_POSITION;
    }

    public void retract() {
        intakePosition = INTAKE_RETRACT_POSITION;
    }

    @Override
    public void initHardware() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeDeploy = hardwareMap.get(Servo.class, "intakeDeploy");
    }

    @Override
    public void periodic() {
        intake.setPower(intakePower);
        intakeDeploy.setPosition(intakePosition);
    }
}

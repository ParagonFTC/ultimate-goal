package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.util.UltimateGoalLocalizationUtil;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Camera implements Subsystem {
    private VuforiaLocalizer vuforia;
    private WebcamName webcamName;
    private HardwareMap hardwareMap;
    private FtcDashboard dashboard;
    private Drivetrain drivetrain;

    public static final String VUFORIA_LICENSE_KEY = "AWiNNND/////AAABmXs1dnUBs0nRthBOBC9Kyho3qpyfTGG2bd6vCAGPVkkqo1CjZrX4bGMzfqbqVBTTC0FbNe4v409zGxLeT35LOz17xAiE9za3L2h9QYfG/HXweWFwImnHP3nRd/4BoM5Sufel1Qj3l9nXNd05ddZrVY5lgyU04m6vsLjMoredoYrHBkdzr5RnN495Hjg+sdw0dM8/7Gnrd6nGDaXFtEHQS2LveGgpGzLDeXXP0bany+tGjV7BopDAMUSJxpp5IPgkigxQZiLYXuL736Xc/rXrbHyDQzDfVfYludH0XgCHoSNxNXg2qXXjrAUmhvApMPRceGTjMkRNyRiqoVA0EBFpvOkxBVFFAyFyXU1JjnNiFRcd";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    private static final float phoneYRotate = -90;
    private static final float phoneXRotate = 0;
    private static final float CAMERA_FORWARD_DISPLACEMENT = 149.83175f; //174.41213f
    private static final float CAMERA_VERTICAL_DISPLACEMENT = 125.89561f; //141.85630f
    private static final float CAMERA_LEFT_DISPLACEMENT = -58.56251f;
    private static final double HEADING_OFFSET = 0.0389808535;

    private static final double ROBOT_CORNER_X = 8.7926122;
    private static final double ROBOT_CORNER_Y = -8.51771655;

    private OpenGLMatrix lastLocation;
    private VuforiaTrackables targetsUltimateGoal;
    private VuforiaTrackable blueTowerGoalTarget;

    private boolean isTracking;

    public Camera(HardwareMap hardwareMap, Drivetrain drivetrain) {
        this.hardwareMap = hardwareMap;
        this.drivetrain = drivetrain;
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void initHardware() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");

        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT,CAMERA_LEFT_DISPLACEMENT,CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, 0, phoneXRotate));

        ((VuforiaTrackableDefaultListener) blueTowerGoalTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        targetsUltimateGoal.activate();
        dashboard.startCameraStream(vuforia, 0);
    }

    @Override
    public void periodic() {
        isTracking = ((VuforiaTrackableDefaultListener)blueTowerGoalTarget.getListener()).isVisible();
        if (isTracking) {
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) blueTowerGoalTarget.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
            /*
            VectorF translation = lastLocation.getTranslation();
            double robotX = translation.get(0) / mmPerInch;
            double robotY = translation.get(1) / mmPerInch;
            double distanceToTower = UltimateGoalLocalizationUtil.distanceToTowerGoal(robotX, robotY);
            double distanceScaling = (distanceToTower + 10) / distanceToTower;
            double correctedX = (distanceScaling * robotX + drivetrain.getPoseEstimate().getX()) / 2;
            double correctedY = (distanceScaling * robotY + drivetrain.getPoseEstimate().getY()) / 2;
            Orientation orientation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS);
            double averageHeading = (Angle.normDelta(drivetrain.getPoseEstimate().getHeading()) + orientation.thirdAngle - Math.PI/2) / 2;
            if (Math.abs(averageHeading) < Math.PI/4) {
                drivetrain.setPoseEstimate(new Pose2d(correctedX, correctedY, Angle.norm(averageHeading)));
            }
            */
        } else {
            lastLocation = null;
        }
    }

    public Pose2d getPoseEstimate() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            Orientation orientation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS);
            return new Pose2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, Angle.norm(orientation.thirdAngle - Math.PI/2));
        }

        else {
            //return new Pose2d(drivetrain.getPoseEstimate().getX(), drivetrain.getPoseEstimate().getY(), Angle.normDelta(drivetrain.getPoseEstimate().getHeading()));
            return null;
        }
    }

    public double getDistance() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return UltimateGoalLocalizationUtil.distanceToTowerGoal(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch);
        } else {
            //Pose2d drivetrainPoseEstimate = drivetrain.getPoseEstimate();
            //return UltimateGoalLocalizationUtil.distanceToTowerGoal(drivetrainPoseEstimate.getX(), drivetrainPoseEstimate.getY()) - 10;
            return Double.NaN;
        }
    }

    public double getCurrentHeading() {
        if (lastLocation != null) {
            Orientation orientation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS);
            return orientation.thirdAngle - Math.PI/2;
        } else {
            //return Angle.normDelta(drivetrain.getPoseEstimate().getHeading());
            return Double.NaN;
        }
    }

    public double getTargetHeading() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return UltimateGoalLocalizationUtil.angleToTowerGoal(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch) + HEADING_OFFSET;
        } else {
            //Pose2d drivetrainPoseEstimate = drivetrain.getPoseEstimate();
            //return UltimateGoalLocalizationUtil.angleToTowerGoal(drivetrainPoseEstimate.getX(), drivetrainPoseEstimate.getY()) + HEADING_OFFSET;
            return Double.NaN;
        }
    }

    public boolean isBehindLine() {
        double robotFrontX = ROBOT_CORNER_X * Math.cos(Math.abs(getTargetHeading())) - ROBOT_CORNER_Y * Math.sin(Math.abs(getTargetHeading()));
        return getPoseEstimate().getX() + robotFrontX < 8;
    }

    public boolean isTracking() {
        return isTracking;
    }

    public void stopTracking() {
        targetsUltimateGoal.deactivate();
    }
}

package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

    public static final String VUFORIA_LICENSE_KEY = "AWiNNND/////AAABmXs1dnUBs0nRthBOBC9Kyho3qpyfTGG2bd6vCAGPVkkqo1CjZrX4bGMzfqbqVBTTC0FbNe4v409zGxLeT35LOz17xAiE9za3L2h9QYfG/HXweWFwImnHP3nRd/4BoM5Sufel1Qj3l9nXNd05ddZrVY5lgyU04m6vsLjMoredoYrHBkdzr5RnN495Hjg+sdw0dM8/7Gnrd6nGDaXFtEHQS2LveGgpGzLDeXXP0bany+tGjV7BopDAMUSJxpp5IPgkigxQZiLYXuL736Xc/rXrbHyDQzDfVfYludH0XgCHoSNxNXg2qXXjrAUmhvApMPRceGTjMkRNyRiqoVA0EBFpvOkxBVFFAyFyXU1JjnNiFRcd";

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private static final float phoneYRotate = -90;
    private static final float phoneXRotate = 0;
    private static final float CAMERA_FORWARD_DISPLACEMENT = 174.41213f;
    private static final float CAMERA_VERTICAL_DISPLACEMENT = 141.85630f;
    private static final float CAMERA_LEFT_DISPLACEMENT = 0;
    private static final double HEADING_OFFSET = -0.120;

    private OpenGLMatrix lastLocation;
    private VuforiaTrackables targetsUltimateGoal;
    private VuforiaTrackable blueTowerGoalTarget;

    private boolean isTracking;

    public Camera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
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
                .translation(0, 0, mmTargetHeight)
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

            TelemetryPacket packet = new TelemetryPacket();

            packet.put("Current Heading", getCurrentHeading());
            packet.put("Target Heading", getTargetHeading());
            packet.put("Distance", getTargetHeading());
            packet.put("Is Tracking", isTracking);

            dashboard.sendTelemetryPacket(packet);
        }
    }

    public double getDistance() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return Math.sqrt(Math.pow(translation.get(0), 2) + Math.pow(translation.get(1), 2)) / mmPerInch;
        } else {
            return Double.NaN;
        }
    }

    public double getCurrentHeading() {
        if (lastLocation != null) {
            Orientation orientation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS);
            return orientation.thirdAngle;
        } else {
            return Double.NaN;
        }
    }

    public double getTargetHeading() {
        if (lastLocation != null) {
            VectorF translation = lastLocation.getTranslation();
            return Math.atan2(-translation.get(0), translation.get(1)) + HEADING_OFFSET;
        } else {
            return Double.NaN;
        }
    }

    public boolean isTracking() {
        return isTracking;
    }

    public void stopTracking() {
        targetsUltimateGoal.deactivate();
    }
}

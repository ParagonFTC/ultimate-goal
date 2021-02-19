package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.disnodeteam.dogecommander.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * Camera for autonomous ring stack detection
 */

public class AutoCamera implements Subsystem {
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private WebcamName webcamName;
    private HardwareMap hardwareMap;
    private FtcDashboard dashboard;

    public static final String VUFORIA_LICENSE_KEY = "AWiNNND/////AAABmXs1dnUBs0nRthBOBC9Kyho3qpyfTGG2bd6vCAGPVkkqo1CjZrX4bGMzfqbqVBTTC0FbNe4v409zGxLeT35LOz17xAiE9za3L2h9QYfG/HXweWFwImnHP3nRd/4BoM5Sufel1Qj3l9nXNd05ddZrVY5lgyU04m6vsLjMoredoYrHBkdzr5RnN495Hjg+sdw0dM8/7Gnrd6nGDaXFtEHQS2LveGgpGzLDeXXP0bany+tGjV7BopDAMUSJxpp5IPgkigxQZiLYXuL736Xc/rXrbHyDQzDfVfYludH0XgCHoSNxNXg2qXXjrAUmhvApMPRceGTjMkRNyRiqoVA0EBFpvOkxBVFFAyFyXU1JjnNiFRcd";

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public StackSize stackSize = StackSize.NONE;

    public AutoCamera(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        dashboard = FtcDashboard.getInstance();
    }

    public enum StackSize {
        SINGLE,
        QUAD,
        NONE
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

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.2, 4.0/3.0);
            //tfod.setClippingMargins(0,275,150,425);
        }

        FtcDashboard.getInstance().startCameraStream(tfod, 0);
    }

    @Override
    public void periodic() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null && recognitions.size() > 0) {
                Recognition recognition = recognitions.get(0);
                if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) stackSize = StackSize.QUAD;
                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) stackSize = StackSize.SINGLE;
            }
        }
    }

    public StackSize getStackSize() {
        return stackSize;
    }

    public void shutdown() {
        if (tfod != null) tfod.shutdown();
    }
}

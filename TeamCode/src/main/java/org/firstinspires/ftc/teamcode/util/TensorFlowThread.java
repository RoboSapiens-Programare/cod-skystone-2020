package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class TensorFlowThread extends Thread {

    public static final String TAG = "TensorThread";

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "SkyStone";
    private static final String LABEL_SECOND_ELEMENT = "SkyStone";

    private static final String VUFORIA_KEY =
            "AYlEu/7/////AAABmXB1kirNm0vlrZa4DCCmkis6ZNJkEkHGNYjIfoKWcK+yxnJOhuC4Lw3B63L+Y5vrSoTsr1mEe6bvGcMR8Hg+v1Z1Cih0IrBRHdIfrrg6lfa723ft/unZOKgck3ftCj8gWuiM89d+A4smkenUI5P/HXMKMGKCk4xxv5of9YNSX8r4KFO8lD+bqYgnP+GVXzD/TwQo7Dqer3bf0HVbOqP6j6HREHAZdP6Idg/JwyRG8LSdC6ekTwogxCWsuWiaUhuC8uAQ4r/ZfJykZpXYCxhdcLwMM4OaUXkUAPuUenzxlL8MXkwOhsDfqiQNEfSB00BodWKq28EC6cc+Vsko8r9PreeU6jCYR4d84VK8uBFLGaJx";

    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    private TensorFlowReturn lastUpdate = new TensorFlowReturn(0,0,0,0,70,false,"");

    public TensorFlowReturn getLastUpdate() {
        return lastUpdate;
    }

    //Cu override
    public TensorFlowThread(HardwareMap hardwareMap) {
        activateTfod(hardwareMap);
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        closeTfod();
    }

    @Override
    public void run() {
        while(this.isAlive()){
            this.lastUpdate = update();
        }
    }

    //Non-generated
    public TensorFlowReturn update(){
        if (tfod != null) {
            double lastTop = lastUpdate.top, lastBottom = lastUpdate.bottom, lastRight = lastUpdate.right, lastLeft = lastUpdate.left, lastAngle = lastUpdate.approxAngle;
            boolean hasRecognised = lastUpdate.hasDetectedObject;
            String lastLabel = lastUpdate.detectedObjectLabel;

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    lastTop = recognition.getTop();
                    lastBottom = recognition.getBottom();
                    lastRight = recognition.getRight();
                    lastLeft = recognition.getLeft();
                    lastLabel = recognition.getLabel();
                    lastAngle = recognition.estimateAngleToObject(AngleUnit.RADIANS);
                    hasRecognised = true;
                }
            }

            return new TensorFlowReturn(lastTop, lastLeft, lastRight, lastBottom, lastAngle, hasRecognised, lastLabel);
        }

        return null;
    }

    private TensorFlowReturn update(TelemetryPacket packet){
        if (tfod != null) {
            double lastTop = 0, lastBottom = 0, lastRight = 0, lastLeft = 0, lastAngle = 0;
            boolean hasRecognised = false;
            String lastLabel = "";

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                packet.put("# Object Detected", updatedRecognitions.size());


                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    packet.put(String.format("label (%d)", i), recognition.getLabel());
                    packet.put("left, top", recognition.getLeft() + ", " + recognition.getTop());
                    packet.put("right, bottom", recognition.getRight() + ", " + recognition.getBottom());

                    lastTop = recognition.getTop();
                    lastBottom = recognition.getBottom();
                    lastRight = recognition.getRight();
                    lastLeft = recognition.getLeft();
                    lastLabel = recognition.getLabel();
                    lastAngle = recognition.estimateAngleToObject(AngleUnit.RADIANS);
                    hasRecognised = true;
                }
            }

            return new TensorFlowReturn(lastTop, lastLeft, lastRight, lastBottom, lastAngle, hasRecognised, lastLabel);
        }

        return null;
    }

    private List<SkyStone> FindSkystone(boolean isBlueSide){
        if (tfod != null) {

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions.size() != 3) {

                float PosSkystone = 0;
                int LeftOfSkystone = 0;

                for (Recognition recognition : updatedRecognitions) {
                   if (recognition.getLabel().equals("SkyStone")){
                        PosSkystone = recognition.getRight();
                   }
                }
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals("SkyStone")){
                        if (recognition.getRight() < PosSkystone){
                            LeftOfSkystone++;
                        }
                    }
                }

                List<SkyStone> stones = new ArrayList<>();

                stones.add(new SkyStone(LeftOfSkystone, isBlueSide));
                stones.add(new SkyStone(LeftOfSkystone + 3, isBlueSide));

                return stones;
            }

        }

        return new ArrayList<>();
    }


    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void activateTfod(HardwareMap hardwareMap){
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(hardwareMap);
        }

        if (tfod != null) {
            tfod.activate();
        }
    }

    private void closeTfod(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }



}

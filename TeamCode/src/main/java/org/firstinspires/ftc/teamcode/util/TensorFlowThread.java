package org.firstinspires.ftc.teamcode.util;

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
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "SkyStone";

    private static final String VUFORIA_KEY =
            "AYlEu/7/////AAABmXB1kirNm0vlrZa4DCCmkis6ZNJkEkHGNYjIfoKWcK+yxnJOhuC4Lw3B63L+Y5vrSoTsr1mEe6bvGcMR8Hg+v1Z1Cih0IrBRHdIfrrg6lfa723ft/unZOKgck3ftCj8gWuiM89d+A4smkenUI5P/HXMKMGKCk4xxv5of9YNSX8r4KFO8lD+bqYgnP+GVXzD/TwQo7Dqer3bf0HVbOqP6j6HREHAZdP6Idg/JwyRG8LSdC6ekTwogxCWsuWiaUhuC8uAQ4r/ZfJykZpXYCxhdcLwMM4OaUXkUAPuUenzxlL8MXkwOhsDfqiQNEfSB00BodWKq28EC6cc+Vsko8r9PreeU6jCYR4d84VK8uBFLGaJx";

    public TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    /*
    *   Not quite good practice, but these are the variables that modify
    */
    private TensorFlowReturn lastTfodData = new TensorFlowReturn(0,0,0,0,0,false,"");
    private List<SkyStone> detectedSkyStones = new ArrayList<>();
    /*
    *   Getters follow
    */


    public TensorFlowReturn getLastTfodData() {
        return lastTfodData;
    }
    public List<SkyStone> getDetectedSkyStones(){
        return detectedSkyStones;
    }



    //Cu override
    //Constructor
    public TensorFlowThread(HardwareMap hardwareMap) {
        activateTfod(hardwareMap);
    }

    //Destructor
    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        closeTfod();
    }

    //Dis the RUN method
    @Override
    public void run() {
        while(this.isAlive()){
            this.lastTfodData = rawTfodData();
            this.detectedSkyStones = findSkystone(false); //TODO: trebuie sa gasim o varianta non-case based
        }

        closeTfod();
    }

    //Non-generated
    public TensorFlowReturn rawTfodData(){
        if (tfod != null) {
            double lastTop = lastTfodData.top, lastBottom = lastTfodData.bottom, lastRight = lastTfodData.right, lastLeft = lastTfodData.left, lastAngle = lastTfodData.approxAngle;
            boolean hasRecognised = lastTfodData.hasDetectedObject;
            String lastLabel = lastTfodData.detectedObjectLabel;

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

    private List<SkyStone> findSkystone(boolean isBlueSide){
        if (tfod != null) {

            //get tfod recognitions
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            //we need 3 recognition to determine where the skystone is
            if (updatedRecognitions != null && updatedRecognitions.size() >= 2) {

                int PosSkystone = 0;
                float skystoneRight = 0;
                float stoneRight = 0;
                boolean skystoneExists = false;

                for (Recognition recognition : updatedRecognitions) {
                   if (recognition.getLabel().equals("SkyStone")){
                        skystoneExists = true;
                        skystoneRight = recognition.getRight();
                   }
                    if (recognition.getLabel().equals("Stone")){
                        stoneRight = recognition.getRight();
                    }
                }

                if(!skystoneExists){
                    PosSkystone = 3;
                }
                else {
                    PosSkystone = isBlueSide? stoneRight < skystoneRight? 2 : 1 : stoneRight < skystoneRight? 1 : 2;
                }

                List<SkyStone> stones = new ArrayList<>();

                stones.add(new SkyStone(PosSkystone, isBlueSide));
                stones.add(new SkyStone(PosSkystone + 3, isBlueSide));

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

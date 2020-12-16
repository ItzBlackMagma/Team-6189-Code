package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class RingDetector {

    public Camera camera;
    public Telemetry telemetry;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public TFObjectDetector tfod;

    public RingDetector(Camera camera, Telemetry telemetry) {
        this.camera = camera;
        this.telemetry = telemetry;
    }

    public void initTfod(HardwareMap hardwareMap) {
        telemetry.addData("/> ", "Initializing TensorFlow Object Detection...");

        //int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, camera.vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        telemetry.addData("/> TFOD", " Loaded Assets...");
        telemetry.addData("/> TFOD", " Creating Instance...");

        if (tfod != null)
            tfod.activate();

        telemetry.addData("> ROBOT", "TensorFlow Has Been Initialized");
        telemetry.update();
    }

    public int detect(){
        int stackSize = 0;
        // getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                if(recognition.getLabel() == "Single")
                    stackSize = 1;
                else if (recognition.getLabel() == "Quad")
                    stackSize = 4;
            }
            telemetry.update();
        }
        telemetry.update();

        return stackSize;
    }

    public void shutdown(){
        telemetry.addData("> ROBOT", "Shutting Down TFOD...");
        tfod.shutdown();
        telemetry.addData("> ROBOT", "TFOD Has Been Shut Down");
        telemetry.update();
    }


}

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Camera {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    private OpenGLMatrix lastLocation = new OpenGLMatrix();

    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    // We use millimeters for accuracy
    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    float halfField = 72 * mmPerInch;
    float quadField  = 36 * mmPerInch;

    // Our instance of Vuforia
    VuforiaLocalizer vuforia;

    //targets
    VuforiaTrackables targets;
    List<VuforiaTrackable> allTrackables;


    public Camera(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void activate(HardwareMap hardwareMap){
        telemetry.addData("/> ", "Beginning Camera Activation Process...");

        // Camera preview
        telemetry.addData("/> CAMERA", " Checking For Camera Monitor View...");
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(); // uses no camera preview

        telemetry.addData("/> CAMERA", " Camera Monitor View Is Disabled...");

        // Vuforia parameters
        parameters.vuforiaLicenseKey = "AU6DGO3/////AAABmfbaGbX2lU7yobzEFgj/TC95dmC+wGBKUjoXoXSYSiz92D3Y5XU2YY4TlNcgnQLdXr8Pz3zstBN9KHBPTMczwa4QWR0rqGKqC5L3rdvyZM/bFd2v9/YkKpd54Uyl0tX1CyEB9XSW2HKhFjcofvkud19pT1nqEuQBU+Q8zKCJXc8gSycUPELKVARHhsMPOoJMH4wlS7QmwWde4q/nolTJIjolaLvSemiql29GodpyuXfxCyjRKlCLvEZ1GbwhfdDwrPsZM1QBbOJgdnAIGZ00FNf+059bdvUv3SkcfacMRVua/Jp1BWPgkocF3y2PZrBN28s0AGIlbFBMkYSDZ8stGOWDI/a9nM1EXutODEZUGOUd";
        parameters.useExtendedTracking = false;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "robo eye");

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Ensures the bitmap file type is available
        vuforia.enableConvertFrameToBitmap();

        // @see #captureFrameToFile()
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        // Load the data sets for the tracking pictures
        VuforiaTrackables targets = vuforia.loadTrackablesFromAsset("UltimateGoal");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackable redWall = targets.get(2); // image 3
        redWall.setName("RedWall");

        VuforiaTrackable blueWall = targets.get(1); // image 2
        blueWall.setName("BlueWall");

        VuforiaTrackable frontWall = targets.get(0); // image 1
        frontWall.setName("FrontWall");

        VuforiaTrackable redTower = targets.get(4); // image 5
        redTower.setName("RedTower");

        VuforiaTrackable blueTower = targets.get(3); // image 4
        blueTower.setName("BlueTower");

        // For convenience, gather together all the trackable objects in one easily-iterable collection
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);


        //Set the position of the perimeter targets with relation to origin (center of field)
        redWall.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueWall.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWall.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTower.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTower.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Now we put the location of the camera on the robot
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(9, -6.5f,3.5f)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 90, 90, 0));


        // Tell the target listeners where the camera is
        ((VuforiaTrackableDefaultListener)redWall.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener)blueWall.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener)frontWall.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener)redTower.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);
        ((VuforiaTrackableDefaultListener)blueTower.getListener()).setCameraLocationOnRobot(parameters.cameraName, robotFromCamera);

        this.targets = targets;
        this.allTrackables = allTrackables;

        targets.activate();

        telemetry.addData("/> CAMERA", " Vuforia Is Now Activated...");
        telemetry.addData("/> ROBOT", "Camera Activation Complete");
    }

    public void track(){
        telemetry.addData("/> VUFORIA", " Tracking Targets...");
        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            //telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
        /**
         * Provide feedback as to where the robot was last located (if we know).
         */
        if (lastLocation != null) {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            telemetry.addData("/> VUFORIA: Robot Transform (Orientation & Location)", format(lastLocation));
            telemetry.addData("/> VUFORIA: Robot Location: X, Y, R", getX() + ", " + getY() + ", " + getAngle()); // returns inches

        } else {
            telemetry.addData("/> VUFOIRA ", "Target Not Found: Location Unknown");
        }
        //telemetry.update();
    }

    public void deactivate(){
        targets.deactivate();
        telemetry.addData("/> CAMERA", " Vuforia Has Been Deactivated");
    }

    public String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public void captureFrameToFile() {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
                    try {
                        FileOutputStream outputStream = new FileOutputStream(file);
                        try {
                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                        } finally {
                            outputStream.close();
                            telemetry.log().add("captured %s", file.getName());
                        }
                    } catch (IOException e) {
                        RobotLog.ee("TAG", e, "exception in captureFrameToFile()");
                    }
                }
            }
        }));
    }

    public double getAngle(){ // returns the rotation around just the Z axis
        return (double) Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }

    public Orientation getOrientation(){
        return Orientation.getOrientation(lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    public float[] getLocation(){
        if(lastLocation != null) {
            float[] location = lastLocation.getTranslation().getData();
            return location;
        } else {
            return null;
        }
    }

    public double getX(){
        return getLocation()[0];
    }

    public double getY(){
        return getLocation()[1];
    }

}

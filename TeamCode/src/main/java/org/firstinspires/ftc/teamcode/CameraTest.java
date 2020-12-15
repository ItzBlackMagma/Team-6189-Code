package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Camera Test")
public class CameraTest extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);
    Camera camera = new Camera(robot);
    RingDetector detector = new RingDetector(camera, telemetry);

    @Override
    public void init() {
        robot.init(hardwareMap);
        camera.activate(hardwareMap);
        detector.initTfod(hardwareMap);
    }

    @Override
    public void loop() {
        //camera.track();
        detector.detect();
    }

    @Override
    public void stop() {
        //camera.deactivate();
        detector.shutdown();
    }
}

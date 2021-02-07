package org.firstinspires.ftc.teamcode.infomodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(group = "Info OpModes")
public class CameraTest extends OpMode {
    Robot robot = new Robot(telemetry);
    double x,y,r,p;
    boolean takePic = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        p = gamepad1.right_trigger;
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        r = gamepad1.right_stick_x;

        robot.move(x,y,r,p);

        if(gamepad1.x && !takePic){
            robot.camera.captureFrameToFile();
        } takePic = gamepad1.x;

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}

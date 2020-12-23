package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Driver Controlled", group="Official")
public class DriverControlled extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);
    Camera camera = new Camera(robot);
    RingDetector detector = new RingDetector(camera, telemetry);

    int inv = 1;
    double[] target;
    double x, y, r, speed;
    boolean manualRotate = false;


    @Override
    public void start() {

    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        inv = robot.getInvFromTeam(robot.getTeam(camera));
    }

    @Override
    public void loop() {

        // set speed
        if (gamepad1.right_trigger > 0.25)
            speed = gamepad1.right_trigger;
        else
            speed = 0.25;

        // set direction
        x = gamepad1.left_stick_x * 2 - robot.robotX;
        y = -(gamepad1.left_stick_y * 2 - robot.robotY);

        // figure out which target to aim at
        if (gamepad2.a){
            target = Locations.POWER_SHOT_A;
        } else if(gamepad2.b){
            target = Locations.POWER_SHOT_B;
        } else if(gamepad2.x){
            target = Locations.POWER_SHOT_C;
        } else if(gamepad2.y) {
            target = Locations.HIGH_GOAL;
        }

        // figure out whether to rotate freely or aim at target
        if (gamepad1.a){
            manualRotate = true;
        } else if (!gamepad1.a){
            manualRotate = false;
        }

        // rotates and aims the robot
        if (manualRotate){
            r = gamepad1.right_stick_x;
            robot.aim(target, inv, false);
        } else {
            robot.aim(target, inv, true);
        }

        // moves the robot
        robot.moveToPoint(x, y, r, speed, 1);
    }

    @Override
    public void stop() {
        robot.stop();
        detector.shutdown();
        camera.deactivate();
    }
}

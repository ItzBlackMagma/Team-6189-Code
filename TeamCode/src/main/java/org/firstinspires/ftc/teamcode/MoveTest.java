package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Move test")
public class MoveTest extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);
    Camera camera = new Camera(robot);
    RingDetector detector = new RingDetector(camera, telemetry);

    int inv = 1, stackSize = 0;
    double speed = 0, spinPower = 0;
    boolean translate = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        camera.activate(hardwareMap);
        detector.initTfod(hardwareMap);
    }

    @Override
    public void loop() {

        camera.track();
        stackSize = detector.detect();

        // Speed control
        if (gamepad1.right_trigger > .25) {
            speed = gamepad1.right_trigger;
        } else {
            speed = .25;
        }

        // Movement mode control
        if(gamepad1.right_bumper)
            translate = false;
        if(gamepad1.left_bumper)
            translate = true;

        // game pad 1 controls (movement)
        if(translate) {
            robot.moveToPoint(robot.robotX + (gamepad1.left_stick_x * 2),
                    robot.robotY + (-gamepad1.left_stick_y * 2),
                    gamepad1.right_stick_x, speed, 0.1);
        } else {
            robot.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
        }

        // game pad 2 controls (scoring)
        spinPower = gamepad2.left_trigger;
        robot.setLaunchPower(spinPower);
        robot.setLoadPower(gamepad2.right_stick_y);

        robot.liftWobble(gamepad2.left_stick_y);
        robot.linear.setPower(gamepad2.left_stick_x);
        // robot.lock.setPower();

        // Output data
        telemetry.addData("/> WOBBLE_LIFT_POS", robot.wobbleLift.getCurrentPosition());
        telemetry.addData("/> ROBOT_POS", camera.getLocation().toString());
        telemetry.addData("/> IMU", robot.getRotation("Z"));
        telemetry.addData("/> SPIN POWER", spinPower);
        telemetry.update();
    }

    @Override
    public void stop() {  robot.stop(); camera.deactivate(); detector.shutdown();  }
}

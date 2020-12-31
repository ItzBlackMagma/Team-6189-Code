package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name="Move test")
public class MoveTest extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);
    Camera camera = new Camera(robot);
    RingDetector detector = new RingDetector(camera, telemetry);


    double speed;
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
        detector.detect();

        if (gamepad1.right_trigger > .25) {
            speed = gamepad1.right_trigger;
        } else {
            speed = .25;
        }

        if(gamepad1.right_bumper)
            translate = false;
        if(gamepad1.left_bumper)
            translate = true;

        if(translate) {
            robot.moveToPoint(robot.robotX + (gamepad1.left_stick_x * 2),
                    robot.robotY + (-gamepad1.left_stick_y * 2),
                    gamepad1.right_stick_x, speed, 0.1);
        } else {
            robot.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
        }

        robot.setLaunchPower(gamepad2.left_trigger);
        robot.setLoadPower(gamepad2.right_stick_y);

        robot.liftWobble(gamepad2.left_stick_y);

        telemetry.addData("/> WOBBLE POS", robot.wobbleLift.getCurrentPosition());

        robot.lock.setPower(gamepad2.left_stick_x);

        telemetry.addData("/> ROBOT_POS", robot.robotX + "       " + robot.robotY);

    }
}

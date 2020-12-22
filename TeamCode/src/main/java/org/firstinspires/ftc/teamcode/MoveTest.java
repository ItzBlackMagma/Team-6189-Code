package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name="Move test")
public class MoveTest extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);

    double speed;
    boolean translate = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger > .25) {
            speed = gamepad1.right_trigger;
        } else {
            speed = .25;
        }

        if(gamepad1.right_bumper)
            translate = false;
        if(gamepad1.left_bumper)
            translate = true;

        if(translate)
            robot.moveToPoint( robot.robotX + (gamepad1.left_stick_x * 2),
                               robot.robotY + (-gamepad1.left_stick_y * 2),
                                  gamepad1.right_stick_x, speed, 1);
        else
            robot.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);

        robot.setLaunchPower(gamepad1.left_trigger);

    }
}

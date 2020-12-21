package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name="Move test")
public class MoveTest extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);

    DcMotor leftSpin, rightSpin;

    double speed;
    boolean translate = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        leftSpin = hardwareMap.dcMotor.get("left spin");
        rightSpin = hardwareMap.dcMotor.get("right spin");

        rightSpin.setDirection(DcMotorSimple.Direction.REVERSE);
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
                    robot.robotY + (-gamepad1.left_stick_y * 2), speed, 1);
        else
            robot.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);


        if(gamepad1.left_trigger > 0.1){
            leftSpin.setPower(gamepad1.left_trigger);
            rightSpin.setPower(gamepad1.left_trigger);
        } else {
            leftSpin.setPower(0);
            rightSpin.setPower(0);
        }

    }
}

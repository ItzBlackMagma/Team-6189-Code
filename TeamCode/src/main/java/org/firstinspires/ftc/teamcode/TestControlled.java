package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Test Opmode", group="test")
public class TestControlled extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        robot.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, (gamepad1.right_trigger / 2));
        telemetry.addData("/> X", gamepad1.left_stick_x);
        telemetry.addData("/> Y", gamepad1.left_stick_y);
        telemetry.addData("/> R", gamepad1.right_stick_x);
        telemetry.addData("/> POWER", gamepad1.right_trigger / 2);
        telemetry.update();
    }
}

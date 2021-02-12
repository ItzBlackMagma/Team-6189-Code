package org.firstinspires.ftc.teamcode.infomodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(group = "Info OpModes")
public class MoveEncoders extends OpMode {
    Robot robot = new Robot(telemetry);
    double x,y,r,p;
    boolean reset = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.resetEncoders();
        robot.useEncoders();
    }

    @Override
    public void loop() {
        p = gamepad1.right_trigger;
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        r = gamepad1.right_stick_x;

        robot.move(x,y,r,p);

        if(gamepad1.x && !reset){
            robot.resetEncoders();
            robot.useEncoders();
        } reset = gamepad1.x;

        telemetry.addData("/> MOTOR 1", robot.motor1.getCurrentPosition());
        telemetry.addData("/> MOTOR 2", robot.motor2.getCurrentPosition());
        telemetry.addData("/> MOTOR 3", robot.motor3.getCurrentPosition());
        telemetry.addData("/> MOTOR 4", robot.motor4.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}

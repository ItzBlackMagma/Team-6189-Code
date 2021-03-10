package org.firstinspires.ftc.teamcode.infomodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.odometry.Kinematics;

@TeleOp(group = "Info OpModes")
public class KinematicsTest extends OpMode {
    Robot robot = new Robot(telemetry);
    Kinematics kinematics = new Kinematics(robot);

    double x, y, r, p;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.noEncoders();

        kinematics.start();
    }

    @Override
    public void loop() {
        controlMovement();
        telemetry.addData("/> GLOBAL X", kinematics.getGLOBAL_X());
        telemetry.addData("/> GLOBAL Y", kinematics.getGLOBAL_Y());
        telemetry.addData("/> POSE R", kinematics.getPose().getR());
        telemetry.addData("/> POSE X", kinematics.getPose().getX());
        telemetry.addData("/> POSE Y", kinematics.getPose().getY());
        telemetry.addData("/> ROTATION", robot.getRotation());
        telemetry.update();
    }

    void controlMovement() {
        p = gamepad1.right_trigger;
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        r = gamepad1.right_stick_x;
        if (p < 0.25)
            p = 0.25;
        robot.move(x, y, r, p);
    }

    @Override
    public void stop() {
        kinematics.isRunning = false;
        robot.shutdown();
    }
}
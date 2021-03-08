package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odometry.Kinematics;

@TeleOp
public class KinematicsTest extends OpMode {
    Robot robot = new Robot(telemetry);
    Kinematics kinematics = new Kinematics(robot);

    double x, y, r, p, spinPower, fireAngle = 0, wobblePower = 0;
    boolean isWobbleGrabbed = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.noEncoders();

        kinematics.start();
    }

    @Override
    public void loop() {
        controlMovement();
        telemetry.addData("/> POSE X", kinematics.pose.getX());
        telemetry.addData("/> POSE Y", kinematics.pose.getY());
        telemetry.addData("/> POSE R", kinematics.pose.getR());
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
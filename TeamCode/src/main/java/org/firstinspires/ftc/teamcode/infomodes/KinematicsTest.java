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
        kinematics.getPose().printPose(telemetry); // x,y,r
        telemetry.addData("/> GLOBAL X", kinematics.getGLOBAL_X());
        telemetry.addData("/> GLOBAL Y", kinematics.getGLOBAL_Y());
        telemetry.addData("/> ROTATION", robot.getRotation());
        telemetry.addData("/> ROTATION DEG", Math.toDegrees(robot.getRotation()));
        telemetry.addData("/> GLOBAL VELOCITY X", kinematics.gVelocityX);
        telemetry.addData("/> GLOBAL VELOCITY Y", kinematics.gVelocityY);
        telemetry.addData("/> AT POINT", kinematics.atPoint);
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
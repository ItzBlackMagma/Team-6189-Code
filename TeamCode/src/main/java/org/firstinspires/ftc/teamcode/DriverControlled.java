package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriverControlled extends OpMode {
    Robot robot = new Robot(telemetry);

    double x, y, r, p, spinPower, fireAngle = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.noEncoders();
    }

    @Override
    public void loop() {
        controlMovement();
        controlWobble();
        controlLauncher();
        telemetry.addData("/> WOBBLE POS", robot.wobble.lifter.getCurrentPosition());
        telemetry.addData("/> FIRE POS", robot.launcher.angle.getCurrentPosition());
        telemetry.addData("/> FIRE ANGLE", robot.launcher.getAngle());
        telemetry.addData("/> DESIRED FIRE ANGLE", fireAngle);
        telemetry.addData("/> SPIN POWER", spinPower);
        telemetry.update();
    }

    void controlLauncher() {
        if (gamepad2.x) { //--------------------------------spin speed
            spinPower = .62;
        } else if (gamepad2.b) {
            spinPower = .565;
        } else {
            spinPower = gamepad2.right_trigger;
        }
        robot.launcher.launch(spinPower);

        //---------------------------------push the ring into the spinner
        if (gamepad2.a) {
            robot.launcher.fire();
        } else {
            robot.launcher.reload();
        }

        //-----------------------------------control the angle
        if (gamepad2.right_bumper) {
            robot.launcher.setAnglePower(0.1);
            fireAngle = Math.atan((Locations.HighGoalHeight - Locations.robotLaunchHeight) / (Locations.GOAL_TO_STACK));
        } else if (gamepad2.left_bumper) {
            robot.launcher.setAnglePower(-0.1);
            fireAngle = Math.atan((Locations.PowerShotHeight - Locations.robotLaunchHeight) / (Locations.GOAL_TO_STACK));
        } else {
            robot.launcher.setAnglePower(0);
        }
        robot.launcher.rotateToAngle(fireAngle, 1);

        robot.launcher.load(gamepad2.right_stick_y); // loader speed
    }

    void controlWobble() {
        if (gamepad1.right_bumper) { // extend wobble arm
            robot.wobble.lifter.setPower(.1);
            robot.wobble.extend(0.1);
        } else if (gamepad1.left_bumper) { // fold wobble arm
            robot.wobble.lifter.setPower(-.1);
            robot.wobble.fold(0.1);
        } else {robot.wobble.lifter.setPower(0);}

        if (gamepad1.x) { // grip wobble goal
            robot.wobble.grip(1);
        } else if (gamepad1.b) { // release wobble goal
            robot.wobble.grip(0);
        }
    }

    void controlMovement() {
        p = gamepad1.right_trigger;
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        r = gamepad1.right_stick_x;
        if (p < 0.25) {p = 0.25;}
        robot.move(x, y, r, p);
    }

    @Override
    public void stop() {robot.shutdown();}
}

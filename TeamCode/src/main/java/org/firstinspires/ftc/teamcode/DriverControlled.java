package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Thread.sleep;

@TeleOp
public class DriverControlled extends OpMode {
    Robot robot = new Robot(telemetry);

    double x, y, r, p, spinPower, fireAngle = 0, wobblePower = 0;
    boolean isWobbleGrabbed = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.noEncoders();

        // startup sequence
        robot.wobble.grip(0);
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
        telemetry.addData("/> SPIN POWER", robot.launcher.spin.getPower());
        // telemetry.addData("/> VOLTAGE", ControlHub_VoltageSensor.getVoltage());
        telemetry.update();
    }

    void controlLauncher() {
        if (gamepad2.x) { //--------------------------------spin speed
            spinPower = .62;
        } else if (gamepad2.b) {
            spinPower = .565;
        } else if(gamepad2.y){
            spinPower = .1;
        }else {
            spinPower = gamepad2.right_trigger;
        }
        robot.launcher.spinPower(spinPower);

        //---------------------------------push the ring into the spinner
        if (gamepad2.a) {
            robot.launcher.fire();
        } else {
            robot.launcher.reload();
        }
        telemetry.addData("/> SERVO POS", robot.launcher.push.getPosition());

        //-----------------------------------control the angle
        if (gamepad2.right_bumper) {
            robot.launcher.angle.setPower(0.1); // setAnglePower(0.1);
            fireAngle = Math.toDegrees(Math.atan((Locations.HighGoalHeight - Locations.robotLaunchHeight) / (Locations.GOAL_TO_STACK)));
        } else if (gamepad2.left_bumper) {
            robot.launcher.angle.setPower(-0.1);// .setAnglePower(-0.1);
            fireAngle = Math.toDegrees(Math.atan((Locations.PowerShotHeight - Locations.robotLaunchHeight) / (Locations.GOAL_TO_STACK)));
        } else {
            robot.launcher.angle.setPower(0); //.setAnglePower(0);
        }
     //   robot.launcher.rotateToAngle(fireAngle, 1);

        robot.launcher.loadPower(gamepad2.right_stick_y); // loader speed
    }

    void controlWobble() {
        if (gamepad2.dpad_down) { // grip wobble goal
            robot.wobble.grip(0);
            isWobbleGrabbed = true;
        } else if (gamepad2.dpad_up) { // release wobble goal
            robot.wobble.grip(1);
            isWobbleGrabbed = false;
        }
        if(isWobbleGrabbed){
            wobblePower = -gamepad2.left_stick_y * 0.8;
        } else if(!isWobbleGrabbed) {
            wobblePower = -gamepad2.left_stick_y * 0.3;
        }
        robot.wobble.lifter.setPower(wobblePower);
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

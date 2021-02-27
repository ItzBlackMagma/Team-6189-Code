package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriverControlled extends OpMode {

    Robot robot = new Robot(telemetry);
    Presets presets = new Presets(robot);

    double x,y,r,p,spinPower,wobblePower,extenderPower,loadPower,wobblePos=0,wobbleInc=1000;
    boolean isDpad_up = false, isDpad_down = false, isX = false, isA = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.noEncoders();
        robot.wobble.resetEncoders();
        robot.wobble.useEncoders();
    }

    @Override
    public void loop() {
        // toggles
        if(gamepad2.dpad_up && !isDpad_up){  // dpad up
            robot.wobble.toPosition();
            wobblePos += wobbleInc;
        }
        isDpad_up = gamepad2.dpad_up;
        if(gamepad2.dpad_down && !isDpad_down){ // dpad down
            wobblePos -= wobbleInc;
        }
        isDpad_down = gamepad2.dpad_down;
        if(gamepad1.x && !isX){  // x pressed
            presets.runNextPreset();
        }
        isX = gamepad1.x;

        // movement
        p = gamepad1.right_trigger;
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        r = gamepad1.right_stick_x;
        if(p < 0.25)
            p = 0.25;

        // secondary game pad controls
        if (gamepad2.x) {
            spinPower = .62;
        } else if (gamepad2.y) {
            spinPower = .75;
        } else if(gamepad2.b) {
            spinPower = .565;
        } else{
            spinPower = gamepad2.right_trigger;
        }

        if(gamepad2.a && !isA){
            robot.launcher.fire();
        } else{
            if(!robot.launcher.isReady)
                robot.launcher.reload();
        }
        isA = gamepad2.a;

        wobblePower = -gamepad2.left_stick_y;
        extenderPower = -gamepad2.left_stick_x;
        loadPower = gamepad2.right_stick_y;

        // set everything
        robot.move(x,y,r,p);
        robot.launcher.launch(spinPower);
        robot.launcher.load(loadPower);
        // robot.wobble.lift(wobblePower);
        robot.wobble.raiseToPos(wobblePos, wobblePower);
        robot.wobble.extend(extenderPower);
        robot.wobble.grip(gamepad2.right_stick_x);
        telemetry.addData("/> WOBBLE POS", robot.wobble.lifter.getCurrentPosition());
    }



    @Override
    public void stop() {
        robot.shutdown();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class DriverControlled extends OpMode {

    Robot robot = new Robot(telemetry);
    double x,y,r,p,spinPower,wobblePower,extenderPower,loadPower,wobblePos=0,wobbleInc=10;
    boolean isDpad_up, dpad_up_toggle, isDpad_down, isDpad_down_toggle;

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
        if (gamepad2.dpad_up){ // dpad up
            if(dpad_up_toggle){
                isDpad_up = false;
            } else {
                isDpad_up = true;
                dpad_up_toggle = true;
            }
        } else {
            isDpad_up = false;
            dpad_up_toggle = false;
        }

        if (gamepad2.dpad_down){ // dpad down
            if(isDpad_down_toggle){
                isDpad_down = false;
            } else {
                isDpad_down = true;
                isDpad_down_toggle = true;
            }
        } else {
            isDpad_down = false;
            isDpad_down_toggle = false;
        }

        // movement
        p = gamepad1.right_trigger;
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        r = gamepad1.right_stick_x;
        if(p < 0.25)
            p = 0.25;

        // secondary
        spinPower = gamepad2.right_trigger;
        wobblePower = gamepad2.left_stick_y;
        extenderPower = gamepad2.left_stick_x;
        loadPower = gamepad2.right_stick_y;

        // wobble pos
        if(isDpad_up)
            wobblePos += wobbleInc;
        if(isDpad_down)
            wobblePos -= wobbleInc;

        // set everything
        robot.move(x,y,r,p);
        robot.launcher.launch(spinPower);
        robot.launcher.load(loadPower);
        // robot.wobble.lift(wobblePower);
        robot.wobble.raiseToPos(wobblePos, wobblePower);
    }



    @Override
    public void stop() {
        robot.shutdown();
    }
}

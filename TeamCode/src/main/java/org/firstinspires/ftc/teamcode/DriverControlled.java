package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DriverControlled extends OpMode {

    Robot robot = new Robot(telemetry);

    double x,y,r,p,spinPower,wobblePower,extenderPower,loadPower,wobblePos=0,wobbleInc=1000;
    final double robotLaunchHeight = 5, HighGoalHeight = 36, PowerShotHeight = 32;
    boolean isDpad_up = false, isDpad_down = false, isX = false, isA = false, highGoal = true;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.noEncoders();
        robot.wobble.resetEncoders();
        robot.wobble.useEncoders();
    }

    @Override
    public void loop() {
        controlMovement();
        controlWobble();
        controlLauncher();
        telemetry.addData("/> WOBBLE POS", robot.wobble.lifter.getCurrentPosition());
    }

    void controlLauncher(){
        if (gamepad2.x) {
            spinPower = .62;
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

        if(gamepad2.right_bumper){
            highGoal = true;
        } else if(gamepad2.left_bumper){
            highGoal = false;
        }

        if(highGoal){
            robot.launcher.rotateToAngle(Math.atan((HighGoalHeight - robotLaunchHeight) / (Locations.GOAL_TO_STACK)), .5);
        } else if(!highGoal){
            robot.launcher.rotateToAngle(Math.atan((PowerShotHeight - robotLaunchHeight) / (Locations.GOAL_TO_STACK)), .5);
        }

        loadPower = gamepad2.right_stick_y;
        robot.launcher.launch(spinPower);
        robot.launcher.load(loadPower);
    }

    void controlWobble(){
        if(gamepad1.dpad_up && !isDpad_up){  // dpad up
            robot.wobble.toPosition();
            wobblePos += wobbleInc;
        }
        isDpad_up = gamepad1.dpad_up;
        if(gamepad1.dpad_down && !isDpad_down){ // dpad down
            wobblePos -= wobbleInc;
        }
        isDpad_down = gamepad1.dpad_down;

        if(gamepad1.right_bumper){ // extend wobble arm
            robot.wobble.raiseToPos(wobblePos, .5);
        } else if(gamepad1.left_bumper){ // fold wobble arm
            robot.wobble.raiseToPos(0,0.5);
        }
        if(gamepad1.x){ // grip wobble goal
            robot.wobble.grip(1);
        } else if(gamepad1.b){ // release wobble goal
            robot.wobble.grip(0);
        }
    }

    void controlMovement(){
        // movement
        p = gamepad1.right_trigger;
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        r = gamepad1.right_stick_x;
        if(p < 0.25)
            p = 0.25;

        robot.move(x,y,r,p);
    }

    @Override
    public void stop() {
        robot.shutdown();
    }
}

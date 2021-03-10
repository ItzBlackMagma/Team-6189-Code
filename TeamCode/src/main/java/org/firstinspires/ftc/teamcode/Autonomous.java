package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.presets.Presets;

public class Autonomous extends LinearOpMode {
    Robot robot = new Robot(telemetry);
    Presets presets = new Presets(robot);

    int stackSize;

    @Override
    public void runOpMode() {
        // init phase
        robot.init(hardwareMap);
        sleep(1000);
        stackSize = robot.camera.stackSize();
        telemetry.addData("/> INIT_STACK_SIZE", stackSize);
        telemetry.update();

        waitForStart();
        stackSize = robot.camera.stackSize();
        telemetry.addData("/> STACK_SIZE", stackSize);
        telemetry.update();

        // run phase
        presets.runWobbleSpecificPreset(stackSize);
        sleep(2000);
        presets.runNextPreset();
        sleep(2000);

        // stop phase
        robot.shutdown();
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Autonomous extends LinearOpMode {
    Robot robot = new Robot(telemetry);
    Presets presets = new Presets(robot);

    int stackSize;

    @Override
    public void runOpMode() throws InterruptedException {
        // init phase
        robot.init(hardwareMap);
        sleep(1000);
        stackSize = robot.camera.stackSize();

        waitForStart();
        stackSize = robot.camera.stackSize();

        // run phase
        presets.runWobbleSpecificPreset(stackSize);
        presets.runNextPreset();


        // stop phase
        robot.shutdown();
    }

}

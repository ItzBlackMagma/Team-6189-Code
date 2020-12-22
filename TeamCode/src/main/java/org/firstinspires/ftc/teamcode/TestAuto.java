package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware(telemetry);
    Camera camera = new Camera(robot);
    Crossbow bow = new Crossbow(robot, camera);
    RingDetector detector = new RingDetector(camera, telemetry);

    Team team;

    int stackSize;
    int inv = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        // init phase
        robot.init(hardwareMap);
        camera.activate(hardwareMap);
        detector.initTfod(hardwareMap);

        for (int i = 0; i < 20; i ++) {
            camera.track();
            stackSize = detector.detect();
        }

        team = robot.getTeam(camera);

        switch (team){
            case RED:
                inv = -1;
                break;
            case BLUE:
                inv = 1;
                break;
        }

        int stackSize = detector.detect();

        waitForStart();
        // run phase
        robot.autoToPoint(camera.quadField, camera.quadField / 2, .5, 1, opModeIsActive());

        switch (stackSize){
            case 0:
                robot.autoToPoint(Locations.TARGET_ZONE_A[0], Locations.TARGET_ZONE_A[1], 0.75, 1, opModeIsActive());
                break;
            case 1:
                robot.autoToPoint(Locations.TARGET_ZONE_B[0], Locations.TARGET_ZONE_A[1], 0.75, 1, opModeIsActive());
                break;
            case 4:
                robot.autoToPoint(Locations.TARGET_ZONE_C[0], Locations.TARGET_ZONE_A[1], 0.75, 1, opModeIsActive());
                break;
        }



        robot.stop();
    }

}

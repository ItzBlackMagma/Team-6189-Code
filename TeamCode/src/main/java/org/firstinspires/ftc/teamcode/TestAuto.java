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

        inv = robot.getInvFromTeam(team);

        waitForStart();
        // run phase

        // moves to the correct wobble goal zone
        switch (stackSize){
            case 0:
                robot.autoToPoint(Locations.TARGET_ZONE_A[0], Locations.TARGET_ZONE_A[1] * inv, 0.75, 1, opModeIsActive());
                break;
            case 1:
                robot.autoToPoint(Locations.TARGET_ZONE_B[0], Locations.TARGET_ZONE_A[1] * inv, 0.75, 1, opModeIsActive());
                break;
            case 4:
                robot.autoToPoint(Locations.TARGET_ZONE_C[0], Locations.TARGET_ZONE_A[1] * inv, 0.75, 1, opModeIsActive());
                break;
        }

        robot.autoToPoint(Locations.LAUNCH_LINE - 20, (Locations.inFTCHalfField / 2) * inv, 0.75, 1, opModeIsActive());

        for(int i = 1; i < Locations.TARGETS.length; i++){
            robot.setLaunchPower(0);
            robot.aim(Locations.TARGETS[i], inv, true);
            sleep(1000);
            robot.setLaunchPower(1);
            sleep(1000);
        }

        robot.autoToPoint(Locations.LAUNCH_LINE, (Locations.inFTCHalfField / 2) * inv, 0.75, 1, opModeIsActive());

        robot.stop();
    }

}

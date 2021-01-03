package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class FirstAuto extends LinearOpMode {

    private RobotHardware robot = new RobotHardware(telemetry);
    private Camera camera = new Camera(robot);
    private RingDetector detector = new RingDetector(camera, telemetry);

    private int stackSize = 0, inv = 1;
    private double aimIncCount = 100;
    private long aimIncTime = 25;

    @Override
    public void runOpMode() throws InterruptedException {

        // init phase
        robot.init(hardwareMap);
        camera.activate(hardwareMap);
        detector.initTfod(hardwareMap);
        telemetry.addData("/> INIT", "Activation complete");

        sleep(1000);

        while ((!isStarted() && opModeIsActive()) || camera.getLocation() == null){
            camera.track();
            stackSize = detector.detect();
            inv = robot.getInvFromTeam(robot.getTeam(camera));
            telemetry.addData("/> LOCATION", camera.getLocation());
            telemetry.addData("/> STACK_SIZE", stackSize);
            telemetry.addData("/> TEAM", robot.getTeam(camera));
            telemetry.addData("/> INV", inv);
            telemetry.update();
        }
        waitForStart();

        /*
         *      ******  RUN PHASE ******
         */

        while (!robot.atAngle) {
            robot.move(0,0, robot.getPowerToAngle(robot.getAngleToPoint(Locations.START_STACK[0], Locations.START_STACK[1])), 0.75);
            camera.track();
            stackSize = detector.detect();
            telemetry.addData("/> LOCATION", camera.getLocation());
            telemetry.addData("/> STACK_SIZE", stackSize);
            telemetry.addData("/> IMU", robot.getRotation("Z"));
            telemetry.update();
        }

        sleep(100);

        // aim and shoot at power shot targets
        robot.setLaunchPower(1); // insert power shot power
        for(int i=1; i<Locations.TARGETS.length; i++){
            robot.setLoadPower(0);
            for(int t = 0; t< aimIncCount; t++) {
                robot.aim(Locations.TARGETS[i], inv, true);
                sleep(aimIncTime);
            }
            robot.setLoadPower(1);
            sleep(100);
        }

        // moves to the correct wobble goal zone
        switch (stackSize){
            case 0:
                robot.autoToPoint(Locations.TARGET_ZONE_A[0], Locations.TARGET_ZONE_A[1] * inv, 0.75, 1, opModeIsActive(), camera);
                break;
            case 1:
                robot.autoToPoint(Locations.TARGET_ZONE_B[0], Locations.TARGET_ZONE_A[1] * inv, 0.75, 1, opModeIsActive(), camera);
                break;
            case 4:
                robot.autoToPoint(Locations.TARGET_ZONE_C[0], Locations.TARGET_ZONE_A[1] * inv, 0.75, 1, opModeIsActive(), camera);
                break;
        }

        sleep(100);

        robot.autoToPoint(Locations.LAUNCH_LINE, robot.robotY, .75, .5, opModeIsActive(), camera);

    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class FirstAuto extends LinearOpMode {

    private RobotHardware robot = new RobotHardware(telemetry);
    private Camera camera = new Camera(robot);
    private RingDetector detector = new RingDetector(camera, telemetry);

    public int stackSize = 0, inv = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        // init phase
        robot.init(hardwareMap);
        camera.activate(hardwareMap);
        detector.initTfod(hardwareMap);

        while (!isStarted() && opModeIsActive()){
            camera.track();
            stackSize = detector.detect();
            inv = robot.getInvFromTeam(robot.getTeam(camera));
        }
        waitForStart();

        /*
         *      ******  RUN PHASE ******
         */

        while (!robot.atAngle) {
            robot.move(0,0, robot.getPowerToAngle(robot.getAngleToPoint(Locations.START_STACK[0], Locations.START_STACK[1])), 0.75);
            camera.track();
            stackSize = detector.detect();
        }

        sleep(100);

        // aim and shoot at power shot targets
        robot.setLaunchPower(1); // insert power shot power
        for(int i=1; i<Locations.TARGETS.length; i++){
            for(int t=0; t<50; t++) {
                robot.aim(Locations.TARGETS[i], inv, true);
                sleep(50);
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

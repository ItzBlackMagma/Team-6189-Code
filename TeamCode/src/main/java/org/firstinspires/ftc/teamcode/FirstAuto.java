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

    private boolean atPoint = false;

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

        robot.stop();
        waitForStart();

        /*
         *      ******  RUN PHASE ******
         */

        /*
        while (!robot.atAngle) {
            robot.move(0,0, robot.getPowerToAngle(robot.getAngleToPoint(Locations.START_STACK[0], Locations.START_STACK[1])), 0.75);
            camera.track();
            stackSize = detector.detect();
            telemetry.addData("/> LOCATION", camera.getLocation());
            telemetry.addData("/> STACK_SIZE", stackSize);
            telemetry.addData("/> IMU", robot.getRotation("Z"));
            telemetry.update();
        }
        */
        telemetry.addData("/> LOCATION", camera.getLocation());
        telemetry.addData("/> STACK_SIZE", stackSize);
        telemetry.addData("/> IMU", robot.getRotation("Z"));
        sleep(100);

        // moves to the correct wobble goal zone
        switch (stackSize){
            case 0: // Zone A
                robot.move(0.1, 1, 0, .75);
                sleep(600);
                robot.stop();
                sleep(100);
                robot.move(-0.1, -1, 0, .75);
                sleep(400);
                robot.stop();
                break;
            case 1: // Zone B
                robot.move(0, 1, 0, .75);
                sleep(1000);
                robot.stop();
                sleep(100);
                robot.move(0, -1, 0, .75);
                sleep(800);
                robot.stop();
                break;
            case 4: // Zone C
                robot.move(0.1, 1, 0, .75);
                sleep(1000);
                robot.stop();
                sleep(100);
                robot.move(-0.1, -1, 0, .75);
                sleep(800);
                robot.stop();
                break;
        }

        robot.stop();
        sleep(1000);

        // fire rings
        robot.setLaunchPower(1);
        sleep(2000);
        setLoaderPower(1, 400);
        sleep(2000);
        setLoaderPower(1, 400);
        robot.setLaunchPower(0);

        // park on launch line
        robot.move(0,1, 0, .75);
        sleep(400);
        robot.stop();
    }

    public void setLoaderPower(double power, long ms){
        robot.setLoadPower(-power);
        sleep(ms);
        robot.setLoadPower(0);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class FirstAuto extends LinearOpMode {
    Robot robot = new Robot(telemetry);
    int stackSize = 0;

    @Override
    public void runOpMode() {

        // init phase
        robot.init(hardwareMap);
        telemetry.addData("/> INIT", "Activation complete");

        stackSize = robot.camera.stackSize();
        sleep(500);

        telemetry.addData("/> IsStarted ?", isStarted());
//        while (opModeIsActive() && isStarted()) {
//            camera.track();
//            stackSize = detector.detect();
//            inv = robot.getInvFromTeam(robot.getTeam(camera));
//            telemetry.addData("/> LOCATION", camera.getLocation());
//            telemetry.addData("/> STACK_SIZE", stackSize);
//            telemetry.addData("/> TEAM", robot.getTeam(camera));
//            telemetry.addData("/> INV", inv);
//            telemetry.update();
//        }

        robot.stop();
        waitForStart();
        /*
         *      ******  RUN PHASE ******
         */


        telemetry.addData("/> STACK_SIZE", stackSize);
        telemetry.addData("/> STATUS", "FINISHED");
        telemetry.update();
        sleep(3000);
        // Selects the most likely stack size
//        if(Singles > Os || Quads > Os){
//            if(Quads > Singles){
//                stackSize = 4;
//            } else {
//                stackSize = 1;
//            }
//        } else {
//            stackSize = 0;
//        }

        stackSize = robot.camera.stackSize();

        telemetry.addData("/> LOCATION", robot.camera.getLocation());
        telemetry.addData("/> STACK_SIZE", stackSize);
        telemetry.addData("/> IMU", robot.getRotation());
        telemetry.update();
        sleep(3000);

        // moves to the correct wobble goal zone
        switch (stackSize){
            case 0: // Zone A
                robot.move(0, 1, 0, .75);
                sleep(1400);
                robot.move(1, 0, 0, .75);
                sleep(1100);
                robot.move(0,0,1,0.75);
                sleep(300);
                robot.move(0,0,-1,0.75);
                sleep(300);
                robot.stop();
                sleep(100);
                robot.move(-0.2, -1, 0, .75);
                sleep(1200);
                robot.stop();
                break;
            case 1: // Zone B
                robot.move(0, 1, 0, .75);
                sleep(2100);
                robot.move(1, 0, 0, .75);
                sleep(500);
                robot.move(0,-1,0,.5);
                sleep(500);
                robot.move(1, 0, 0, .75);
                sleep(400);
                robot.stop();
                sleep(100);
                robot.move(-0.2, -1, 0, .75);
                sleep(1200);
                robot.stop();
                break;
            case 4: // Zone C
                robot.move(0, 1, 0, .75);
                sleep(2450);
                robot.move(1, 0, 0, .75);
                sleep(1000);
                robot.stop();
                robot.move(0,0,1,0.75);
                sleep(300);
                robot.move(0,0,-1,0.75);
                sleep(300);
                robot.stop();
                sleep(100);
                robot.move(-0.1, -1, 0, .75);
                sleep(2150);
                robot.stop();
                break;
        }

        robot.stop();
        sleep(1000);

        robot.move(0,0,1,0.5);
        sleep(100);
        robot.stop();

//        while(robot.getRotation() > 0.5 || robot.getRotation() < -0.5){
//            telemetry.addData("/> IMU", robot.getRotation());
//            double deltaRot = 0 - robot.getRotation();
//            if(deltaRot > 0.5){
//                robot.move(0,0,0.5, 1);
//            } else if( deltaRot < -0.5) {
//                robot.move(0,0,-0.5, 1);
//            } else {
//                robot.stop();
//            }
//        }

        sleep(500);

        robot.move(-1, 0, 0, .75);
        sleep(1100);
        robot.stop();
        sleep(500);

        // fire rings
        robot.launcher.spinPower(0.62);
        sleep(2000);
        setLoaderPower(1, 400);
        robot.move(-1,0,0,.5);
        sleep(200);
        robot.stop();
        sleep(2000);
        setLoaderPower(1, 1000);
        robot.launcher.spinPower(0);

        // park on launch line
        robot.move(0,1, 0, .75);
        sleep(1300);
        robot.stop();
    }

    public void setLoaderPower(double power, long ms){
        robot.launcher.loadPower(-power);
        sleep(ms);
        robot.launcher.loadPower(0);
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class NewAutonomous extends LinearOpMode {
    Robot robot = new Robot(telemetry);
    final double DEFAULT_SPEED = 0.8;
    int stackSize = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.resetEncoders();
        robot.toPosition();
        waitForStart();

        stackSize = robot.camera.stackSize();
        telemetry.addData("/> STACK SIZE", stackSize);
        telemetry.update();
        sleep(100);

        switch (stackSize){  // move the first wobble goal
            case 0:
                moveToPoint(Locations.WOBBLE_ZONE_A - 18, 4000);
                moveToPoint(-Locations.WOBBLE_ZONE_A + 18, 4000);
                break;
            case 1:
                moveToPoint(Locations.WOBBLE_ZONE_B - 18, 6000);
                moveToPoint(-Locations.WOBBLE_ZONE_B + 18, 6000);
                break;
            case 4:
                moveToPoint( Locations.WOBBLE_ZONE_C - 18, 7000);
                moveToPoint(-Locations.WOBBLE_ZONE_C + 18, 7000);
                break;
        }
        moveToPoint(0, Locations.LINE_2 * .75, .08, 0, 2000); // navigate to the second wobble goal

        switch (stackSize) { // move the second wobble goal
            case 0:
                moveToPoint(Locations.WOBBLE_ZONE_A - 18, 4000);
                moveToPoint(Locations.distanceBetweenPoints(Locations.WOBBLE_ZONE_A, Locations.WOBBLE_ZONE_A - 18), 4000);
                break;
            case 1:
                moveToPoint(Locations.WOBBLE_ZONE_B - 18, 6000);
                moveToPoint(Locations.distanceBetweenPoints(Locations.WOBBLE_ZONE_B, Locations.WOBBLE_ZONE_A - 18), 6000);
                break;
            case 4:
                moveToPoint(Locations.WOBBLE_ZONE_C - 18, 7000);
                moveToPoint(Locations.distanceBetweenPoints(Locations.WOBBLE_ZONE_C, Locations.WOBBLE_ZONE_A - 18), 7000);
                break;
        }
        moveToPoint(-50, 0.8, 4000);
        robot.launcher.launch(.7);
        for(int i=0;i<1000;i++) {
            robot.launcher.rotateToAngle(Math.atan(Locations.PowerShotHeight - Locations.robotLaunchHeight) /
                    (Locations.GOAL_TO_STACK - Locations.distanceBetweenPoints(Locations.WOBBLE_ZONE_A - 18, Locations.STARTER_STACK_Y)), 1);
        }
        for(int i=0;i<3;i++) {
            shoot(200);
            moveToPoint(-5, 0.25, 1000);
        }
        moveToPoint(18,1000);
    }

    void shoot(long sleep){
        robot.launcher.fire();
        sleep(sleep);
        robot.launcher.reload();
    }

    void moveToPoint(double pos, double offset, double xPower, double yPower, long sleep){
        robot.move(xPower, yPower,0,1);
        robot.setPos(pos - offset,pos + offset,pos - offset,pos + offset);
        robot.toPosition();
        sleep(sleep);
        robot.resetEncoders();
    }

    void moveToPoint(double pos, long sleep){moveToPoint(pos, 0, 0, DEFAULT_SPEED, sleep);} // forward

    void moveToPoint(double offset, double power, long sleep){moveToPoint(0, offset, power, 0, sleep);} // strafe
}

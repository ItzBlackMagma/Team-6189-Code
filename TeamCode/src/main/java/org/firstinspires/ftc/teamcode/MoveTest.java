package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Move test")
public class MoveTest extends OpMode {

    RobotHardware robot = new RobotHardware(telemetry);
    Camera camera = new Camera(robot);
    RingDetector detector = new RingDetector(camera, telemetry);

    int inv = 1, stackSize = 0;
    double speed = 0, spinPower = 0;
    boolean translate = false, atPoint = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        camera.activate(hardwareMap);
        detector.initTfod(hardwareMap);
    }

    @Override
    public void loop() {

        camera.track();
        stackSize = detector.detect();

        // Speed control
        if (gamepad1.right_trigger > .25) {
            speed = gamepad1.right_trigger;
        } else {
            speed = .25;
        }

        // Movement mode control
        if (gamepad1.right_bumper)
            translate = false;
        if (gamepad1.left_bumper)
            translate = true;

        // game pad 1 controls (movement)
        if (translate) {
            /*
            robot.moveToPoint(robot.robotX + (gamepad1.left_stick_x * 4),
                    robot.robotY + (-gamepad1.left_stick_y * 4),
                    gamepad1.right_stick_x, speed, 0.1);
             */
            goToPosition(gamepad1.left_stick_x, gamepad1.left_stick_y, speed, gamepad1.right_stick_x, 0.5);
        } else {
            robot.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, speed);
        }

        // game pad 2 controls (scoring)
        if (gamepad2.x) {
            spinPower = .65;
        } else if (gamepad2.y) {
            spinPower = .75;
        } else if(gamepad2.b) {
            spinPower = .55;
        } else{
            spinPower = gamepad2.left_trigger;
        }

        robot.setLaunchPower(spinPower);
        robot.setLoadPower(gamepad2.right_stick_y);

        robot.liftWobble(gamepad2.left_stick_y / 2);
        robot.linear.setPower(gamepad2.left_stick_x);
        robot.lock.setPower(gamepad2.right_stick_x);

        // Output data
        telemetry.addData("/> WOBBLE_LIFT_POS", robot.wobbleLift.getCurrentPosition());
        telemetry.addData("/> ROBOT_POS X, Y", camera.getX() + ", " + camera.getY());
        telemetry.addData("/> IMU", robot.getRotation("Z"));
        telemetry.addData("/> SPIN POWER", spinPower);
        telemetry.addData("/> TRANSLATE", translate);
    }

    @Override
    public void stop() {  robot.stop(); camera.deactivate(); detector.shutdown();  }


    public void goToPosition(double targetXposition, double targetYposition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        camera.track();
        detector.detect();

        double distanceToXTarget = targetXposition - camera.getX();
        double distanceToYTarget = targetYposition - camera.getY();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        distanceToXTarget = targetXposition - camera.getX();
        distanceToYTarget = targetYposition - camera.getX();

        double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

        double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
        double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);
        double pivotCorrection = desiredRobotOrientation - robot.getRotation("Z");

        robot.move(robotMovementXComponent, robotMovementYComponent, desiredRobotOrientation, robotPower);

        if (distance < allowableDistanceError){
            atPoint = true;
        } else {
            atPoint = false;
        }
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}

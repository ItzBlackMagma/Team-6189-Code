package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotHardware {

    Telemetry telemetry;
    //public OdometryGlobalCoordinatePosition globalPositionUpdate;

    //Crossbow module
    public DcMotor sideways, vertical, drawback, loader, rightSpin, wobbleLift;
    public DcMotor motor1,motor2,motor3,motor4; //starts with left front and moves clockwise
    //public DcMotor verticalLeft, verticalRight, horizontal; //Odometry motors

    public double robotX, robotY, robotAngle;
    public double maxWobbleHeight = 9000, minWobbleHeight = 0, currentWobbleHeight;

    public boolean atAngle = false;
    public boolean atPoint = false;

    public CRServo linear, lock;

    //Chassis module
    public WebcamName webcamName;

    public BNO055IMU imu;

    // We use millimeters for accuracy
    public final double COUNTS_PER_INCH = Locations.mmPerInch * 5; // calculate this with a real number
    public final double COUNTS_PER_REV = COUNTS_PER_INCH;
    public double launchHeight = Locations.mmPerInch * 18;

    /**
     * Creates a RobotHardware object and takes in a telemetry object
     * @param telemetry Used to add data to the screen
     */
    public RobotHardware(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Takes in x, y, r, and power to make the robot move in any direction
     * @param x The power of the robot forward(+) and back(-)
     * @param y The power of the robot right(+) and left(-)
     * @param r The power of the robot in rotating to the right(+) or to the left(-)
     * @param power The speed control to easily adjust how fast the robot will move
     */
    public void move(double x, double y, double r, double power){

        double power1 = y + x + r;
        double power2 = y - x - r;
        double power3 = y + x - r;
        double power4 = y - x + r;

        motor1.setPower(power1 * power);
        motor2.setPower(power2 * power);
        motor3.setPower(power3 * power);
        motor4.setPower(power4 * power);
    }

    public void encoderMove(double x, double y, int pos, double power){
        if (motor1.getCurrentPosition() ==  pos) {
            stop();
            return;
        }
        motor1.setTargetPosition(pos);
        move(x, y,0, power);
    }

    /**
     * Sets all movement power to zero to prevent the robot from moving
     */
    public void stop(){
        move(0,0,0,0);
    }

    /**
     * Sets the robots power in a way that moves the robot closer to the specified point.
     * @param x The desired x coordinate to reach
     * @param y The desired y coordinate to reach
     * @param r The power of the robot in rotating to the right(+) or to the left(-)
     * @param power The speed control to easily adjust how fast the robot will move
     * @param error The radius of error. How accurate the robot needs to be when moving to the specified point.
     */
    public void moveToPoint(double x, double y, double r, double power, double error){ // takes in inches
        double xPow = 0, yPow = 0;

        double deltaX = x - robotX;
        double deltaY = y - robotY;

        boolean atX = false;
        boolean atY = false;

        if (deltaX >= error) {
            xPow = 1;
        } else if (deltaX <= -error) {
            xPow = -1;
        } else {
            xPow = 0;
            atX = true;
        }

        if (deltaY >= error) {
            yPow = 1;
        } else if (deltaY <= -error) {
            yPow = -1;
        } else {
            yPow = 0;
            atY = true;
        }


        if (atX && atY) {
            atPoint = true;
        }

        move(xPow, yPow, r, power);
        telemetry.addData("/> AtPoint  XY (" + x + ", " + y + ")", atPoint);
    }
    /**
     * Sets the robots power in a way that moves the robot closer to the specified point without putting power into the robot.
     * @param x The desired x coordinate to reach
     * @param y The desired y coordinate to reach
     * @param power The speed control to easily adjust how fast the robot will move
     * @param error The radius of error. How accurate the robot needs to be when moving to the specified point.
     */
    public void moveToPoint(double x, double y, double power, double error) { // takes in inches
        moveToPoint(x, y, 0, power, error);
    }

    /**
     * Uses the move to point function and a while loop to move the robot towards the specified point.
     * This function is made for the autonomous period so the robot will continue to move toward the specified point until it reaches this point
     * This function has no rotation power
     * @param x The desired x coordinate to reach
     * @param y The desired y coordinate to reach
     * @param power The speed control to easily adjust how fast the robot will move
     * @param error The radius of error. How accurate the robot needs to be when moving to the specified point.
     * @param opModeIsActive Checks to see if the opmode was deactivated so the robot doesn't continue running if we don't want it to.
     * @param cam Uses the camera object to update the position of the robot
     */
    public void autoToPoint(double x, double y, double power, double error, boolean opModeIsActive, Camera cam){
        atPoint = false;
        while (!atPoint && opModeIsActive){
            cam.track();
            moveToPoint(x, y, power, error);
        }
        stop();
    }

    /**
     * Will return the correct direction of power for turning to a specified angle
     * @param angle The desired angle
     * @return The power needed to move towards that angle
     */
    public double getPowerToAngle(double angle){
        double currentAngle = getRotation("Z");
        double deltaAngle = angle - currentAngle;

        if (deltaAngle > 0.1) {
            atAngle = false;
            return 1.0;
        } else if (deltaAngle < -0.1) {
            atAngle = false;
            return -1.0;
        } else {
            atAngle = true;
            return 0;
        }

    }

    /**
     * This method will return what the angle of the robot needs to be at in order to face a particular point
     * @param pointX X coordinate to face
     * @param pointY y coordinate to face
     * @return The angle needed to face the coordinates
     */
    public double getAngleToPoint(double pointX, double pointY){ // takes in inches
        return Math.atan2(pointX - robotX, pointY - robotY);
    }

    /**
     * This method uses the IMU to figure out the X, Y, and Z rotations of the robot.
     * Z is the most commonly used angle.
     * @param axis Uses "X", "Y", or "Z" to return the desired angle
     * @return returns the angle specified
     */
    public double getRotation(String axis){
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        switch(axis) {
            case "Z":
                return angles.thirdAngle;
            case "Y":
                return angles.secondAngle;
            case "X":
                return angles.firstAngle;
            default:
                return 0;
        }
    }

    /**
     * Uses the camera and the location of the robot to find what team the robot is on.
     * @param camera Specifies the camera for the robot to use
     * @return returns the team as BLUE or RED using the Team enum
     */
    public Team getTeam(Camera camera){
        if (robotY > 0){
            return Team.BLUE;
        } else {
            return Team.RED;
        }
    }

    public void getSpinSpeed(int pos1, int pos2){

    }

    public void setSpinSpeed(double rpm){
        rightSpin.setPower(rpm);
    }

    public void setLaunchPower(double power){
        rightSpin.setPower(power);
    }

    public void setLoadPower(double power){
        loader.setPower(power);
    }

    public double calculateUp(double[] pos, int inv){
        double x = pos[0];
        double y = pos[1] * inv;
        double z = pos[2];

        double distance = Math.hypot(x - robotX, y - robotY);
        return Math.atan2(z, distance); // up angle
    }

    public void aim(double[] pos, int inv, boolean rotate){
        double upAngle = calculateUp(pos, inv);
        double sideAngle = getAngleToPoint(pos[0], pos[1] * inv);

        if(rotate)
            move(0, 0, getPowerToAngle(sideAngle), 0.75);

        // vertical.setPower();

        telemetry.addData("/> SYSTEM", "AIMBOT IS RUNNING");

    }

    public int getInvFromTeam(Team team){
        switch (team){
            case RED:
                return -1;
            case BLUE:
                return 1;
            default:
                return 0;
        }
    }



    public void liftWobble(double power){
        currentWobbleHeight = wobbleLift.getCurrentPosition();
//        if(currentWobbleHeight >= maxWobbleHeight) {
//            wobbleLift.setPower(-Math.abs(power));
//        } else if(currentWobbleHeight <= minWobbleHeight){
//            wobbleLift.setPower(Math.abs(power));
//        } else {
//            wobbleLift.setPower(0);
//        }

        wobbleLift.setPower(-power);
    }

    //Set everything up

    /**
     * Initializes the hardware on the robot all in one place
     * @param hardwareMap Uses the hardware map given to an OpMode
     */
    public void init(HardwareMap hardwareMap){
        telemetry.addData("/> SYSTEM", "Beginning Initialization Process...");

        initIMU(hardwareMap);
        initChassis(hardwareMap);
        initLauncher(hardwareMap);
        initWobbleLift(hardwareMap);

        webcamName = hardwareMap.get(WebcamName.class, "robo eye");

        telemetry.addData("/> INIT", "Webcam Initialized...");
        telemetry.addData("/> SYSTEM", "Initialization Complete");
        telemetry.update();
    }

    private void initIMU(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("/> INIT", "Imu Initialized");

    }

    private void initWobbleLift(HardwareMap hardwareMap){
        wobbleLift = hardwareMap.dcMotor.get("wobble");
        wobbleLift.setDirection(DcMotorSimple.Direction.FORWARD);
        wobbleLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linear = hardwareMap.crservo.get("linear");
        linear.setDirection(DcMotorSimple.Direction.FORWARD);

        lock = hardwareMap.crservo.get("lock");
        lock.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initLauncher(HardwareMap hardwareMap){
        loader = hardwareMap.dcMotor.get("loader");
        rightSpin = hardwareMap.dcMotor.get("right spin");

        rightSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSpin.setDirection(DcMotorSimple.Direction.REVERSE);
        loader.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initChassis(HardwareMap hardwareMap) {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor4 = hardwareMap.dcMotor.get("motor4");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("/> INIT", "Chassis Motors Initialized");
    }

    private void initBow(HardwareMap hardwareMap){
        sideways = hardwareMap.dcMotor.get("vertical");
        vertical = hardwareMap.dcMotor.get("horizontal");
        drawback = hardwareMap.dcMotor.get("drawback");

        sideways.setDirection(DcMotor.Direction.FORWARD);
        vertical.setDirection(DcMotor.Direction.FORWARD);
        drawback.setDirection(DcMotor.Direction.FORWARD);

        sideways.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drawback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encoders
        sideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drawback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("/> INIT", "Motors Initialized...");
        telemetry.update();
    }

    //public void stopGPS(){ globalPositionUpdate.stop(); }
    
}

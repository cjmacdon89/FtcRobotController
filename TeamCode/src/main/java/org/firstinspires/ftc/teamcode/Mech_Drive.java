package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Mech_Drive", group="TeleOp")
public class Mech_Drive extends LinearOpMode {

    @Override
    public void runOpMode(){

        // Setting Variables
        ElapsedTime runtime = new ElapsedTime();
        DcMotor LB_Drive;
        DcMotor RB_Drive;
        DcMotor LF_Drive;
        DcMotor RF_Drive;
        DcMotor Elevator;
        Servo Claw;
        Servo Arm;
        BNO055IMU imu;
        Orientation lastAngles = new Orientation();
        double drive = 0;
        double turn=  0;
        double mech = 0;
        double RF_Mech = 0;
        double LF_Mech = 0;
        double RB_Mech = 0;
        double LB_Mech = 0;
        double maxPower = 0.38;
        double dead_zone = 0.38;
        double elevator_power = 0;
        double current_position = 0;
        double Elevator_Position = 0;
        double elevator_mode = 2;
        double Toggle_Arm = 0;

        ElapsedTime Spinner_Timer = new ElapsedTime(4000);

        //Motor Mapping
        LB_Drive  = hardwareMap.get(DcMotor.class, "LB_Drive");
        RB_Drive = hardwareMap.get(DcMotor.class, "RB_Drive");
        LF_Drive  = hardwareMap.get(DcMotor.class, "LF_Drive");
        RF_Drive = hardwareMap.get(DcMotor.class, "RF_Drive");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Claw = hardwareMap.get(Servo.class, "Claw");
        Arm = hardwareMap.get(Servo.class, "Arm");

        //setting the direction of motors
        LB_Drive.setDirection(DcMotor.Direction.FORWARD);
        RB_Drive.setDirection(DcMotor.Direction.REVERSE);
        LF_Drive.setDirection(DcMotor.Direction.FORWARD);
        RF_Drive.setDirection(DcMotor.Direction.REVERSE);
        Claw.setPosition(0.15);
        Arm.setPosition(1);

        // Display Status of Code (At this point Robot is ready to Start)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Setup Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();
        runtime.reset();

        // run until driver presses STOP
        while (opModeIsActive()) {
            // Read Joystick Values for robot movement
            drive = gamepad1.left_stick_y;
            mech = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            //speed boost
            if (gamepad1.right_trigger > 0.5) {
                maxPower = 0.95;
            }
            // smaller speed boost
            else if (gamepad1.right_bumper) {
                maxPower = 0.65;
            }
            // slow speed mode
            else if(gamepad1.left_bumper){
                maxPower = 0.2;
            }
            // no speed boost
            else{
                maxPower = 0.38;
            }

            // Setting Deadzones

            if ((drive <= dead_zone && drive > 0.0) || (drive >= -dead_zone && drive < 0.0)){
                drive = 0;
            }
            if ((mech <= dead_zone && mech > 0.0) || (mech >= -dead_zone && mech < 0.0)){
                mech = 0;
            }
            if ((turn <= dead_zone && turn > 0.0) || (turn >= -dead_zone && turn < 0.0)){
                turn = 0;
            }

            // Setting Claw Positions
            if(gamepad2.right_bumper){
                Claw.setPosition(0.15);
            }
            if(gamepad2.left_bumper){
                Claw.setPosition(0.38);
            }

            // Setting Arm Positions
            if(gamepad2.dpad_up){
                Arm.setPosition(0);
            }
            if(gamepad2.dpad_down){
                Arm.setPosition(1);
            }

            // Driving commands
            LF_Mech = Range.clip(drive - mech - turn, -1.0 * maxPower, maxPower);
            RF_Mech = Range.clip(drive + mech + turn, -1.0 * maxPower, maxPower);
            RB_Mech = Range.clip(drive - mech + turn, -1.0 * maxPower, maxPower);
            LB_Mech = Range.clip(drive + mech - turn, -1.0 * maxPower, maxPower);

            // Setting Power to motors
            LB_Drive.setPower(LB_Mech);
            RB_Drive.setPower(RB_Mech);
            LF_Drive.setPower(LF_Mech);
            RF_Drive.setPower(RF_Mech);

            //Setting Elevator Encoder Mode

            // Automatic Encoder Mode
            if(gamepad2.dpad_left){
                elevator_mode = 2;
            }
            // Manual Encoder Mode
            if(gamepad2.dpad_right){
                elevator_mode = 0;
            }

            // Elevator Control
            if(elevator_mode > 1) {
                // Setting Elevator To Small Position
                if (gamepad2.x) {
                    Elevator_Position = 1500;
                    Elevator.setTargetPosition((int) Elevator_Position);
                    Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    current_position = Elevator.getCurrentPosition();
                    Elevator.setPower(1.0);
                }
                if (gamepad2.b) {
                    Elevator_Position = 1900;
                    Elevator.setTargetPosition((int) Elevator_Position);
                    Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    current_position = Elevator.getCurrentPosition();
                    Elevator.setPower(1.0);
                }
                if (gamepad2.y) {
                    Elevator_Position = 2800;
                    Elevator.setTargetPosition((int) Elevator_Position);
                    Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    current_position = Elevator.getCurrentPosition();
                    Elevator.setPower(1.0);
                }

                // When "A" Button Pressed Set Elevator Position To 0
                if (gamepad2.a) {
                    Elevator_Position = 0;
                    Elevator.setTargetPosition((int) Elevator_Position);
                    Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    current_position = Elevator.getCurrentPosition();
                    Elevator.setPower(1.0);
                }
            }

            if(elevator_mode < 1) {
                elevator_power = gamepad2.left_stick_y;
                if ((elevator_power <= dead_zone && elevator_power > 0.0) || (elevator_power >= -dead_zone && elevator_power < 0.0)) {
                    elevator_power = 0;
                }

                if(gamepad2.left_trigger < 0.2) {
                    Elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Elevator.setPower(elevator_power);
                }
                else{
                    Elevator_Position = Elevator.getCurrentPosition();
                    Elevator.setTargetPosition((int) Elevator_Position);
                    Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Elevator.setPower(1.0);
                }

            }

            // Show the elapsed game time, wheel and intake system power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Elevator Encoder", "left (%.2f)", current_position);
            telemetry.addData("Front Motors", "left (%.2f), right (%.2f)", LF_Mech, RF_Mech);
            telemetry.addData("Back Motors", "left (%.2f), right (%.2f)", LB_Mech, RB_Mech);
            telemetry.update();
        }
        // Set all motor Power to Zero
        LB_Drive.setPower(0);
        RB_Drive.setPower(0);
        LF_Drive.setPower(0);
        RF_Drive.setPower(0);
        Elevator.setPower(0);
    }
}

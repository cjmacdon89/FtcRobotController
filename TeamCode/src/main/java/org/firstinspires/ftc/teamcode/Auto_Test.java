package org.firstinspires.ftc.teamcode;// Java Deployment Package same for all codes

// Import Libraries used in code
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Declaring the type of OpMode
@Autonomous(name="Auto_Test",preselectTeleOp = "TeleOP_Test")

// Start of program
public class Auto_Test extends LinearOpMode {

    // LinearOpMode function to start running code on Robot
    @Override // Override to validate runOpmode is operating off current code
    public void runOpMode(){

        // Declare DcMotor Objects
        DcMotor LB_Drive;
        DcMotor RB_Drive;
        DcMotor LF_Drive;
        DcMotor RF_Drive;

        //Motor Mapping
        LB_Drive  = hardwareMap.get(DcMotor.class, "LB_Drive");
        RB_Drive = hardwareMap.get(DcMotor.class, "RB_Drive");
        LF_Drive  = hardwareMap.get(DcMotor.class, "LF_Drive");
        RF_Drive = hardwareMap.get(DcMotor.class, "RF_Drive");

        //setting the direction of motors
        LB_Drive.setDirection(DcMotor.Direction.FORWARD);
        RB_Drive.setDirection(DcMotor.Direction.REVERSE);
        LF_Drive.setDirection(DcMotor.Direction.FORWARD);
        RF_Drive.setDirection(DcMotor.Direction.REVERSE);

        // Declare Servo Object
        Servo Claw;
        // Servo Mapping
        Claw = hardwareMap.get(Servo.class, "Claw");
        // setting inital position of servo
        Claw.setPosition(0.5);

        // Declare Distance Sensor Object
        DistanceSensor sensorRange;
        // Sensor Mapping
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        // Collect Distance Information
        double distance = 0;
        distance = sensorRange.getDistance(DistanceUnit.MM);

        // Declare object for colour sensor
        NormalizedColorSensor colorSensor;
        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        // Collect Colour Information
        double colour_alpha = 0;
        double colour_red = 0;
        double colour_blue = 0;
        double colour_green = 0;
        colour_alpha = colors.alpha;
        colour_red = colors.red;
        colour_blue = colors.blue;
        colour_green = colors.green;

        waitForStart();// Wait for Play button to start code

        // Basic Telemetry to push to Driver Station Information
        telemetry.addData("Status","Started");
        telemetry.update();

        // Telemetry of variables
        telemetry.addLine()
                .addData("Red", "%.3f", colour_red)
                .addData("Green", "%.3f", colour_blue)
                .addData("Blue", "%.3f", colour_green);
        telemetry.update();

        double speed = 0.2;
        LB_Drive.setPower(speed);

        int encoder_count_LF = 0;
        encoder_count_LF =  LF_Drive.getCurrentPosition();


        sensorRange.getDistance(DistanceUnit.MM);

        // Location for robot code

    }

}

// Add functions to use in code
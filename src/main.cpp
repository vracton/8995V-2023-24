//©2024 Team 8995B

/*
TODO
(in order of importance)

lock intake on double press
bespoke arcade for judge awards

mabe

linear pid (requires horiz and vert tracking wheel, need to check if bot has space, especially vert)
*/

#include "main.h"
#include "lemlib/api.hpp"

//i hate using undocumented code 😭

//motor setup

//drive motors
pros::Motor rfm(18, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rmm(2, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rbm(8, pros::E_MOTOR_GEARSET_06, false);
pros::Motor lfm(20, pros::E_MOTOR_GEARSET_06, true);
pros::Motor lmm(17, pros::E_MOTOR_GEARSET_06, true);
pros::Motor lbm(10, pros::E_MOTOR_GEARSET_06, true);
//intake motor
pros::Motor intake(9, pros::E_MOTOR_GEARSET_18, true);
//puncher motors here
pros::Motor cata1(7, pros::E_MOTOR_GEARSET_36, false);

//pneumatics
pros::ADIDigitalOut pneum(1); //1-8 = "A"-"H"

//controller
pros::Controller master (CONTROLLER_MASTER);

//inertial
pros::Imu im(19);

//motor group setup
pros::MotorGroup rsm({rbm, rmm, rfm});
pros::MotorGroup lsm({lbm, lmm,lfm});

//drivetrain setup
//why do you make me use doubles 😢
lemlib::Drivetrain drivetrain {
	&lsm, //left motors
	&rsm, //right motors
	11.5, //track width
	3.25, //wheel diam
	360.0, //rpm (36-60 gear ratio & 600 rpm motor)
	0.0 //chase power: idk what this does, but it doesn't work wihout it.
};

//odometry sensor setup
lemlib::OdomSensors odomSensors {
	nullptr, //vertical tracking wheel 1
	nullptr, //vertical tracking wheel 2
	nullptr, //horizontal tracking wheel 1
	nullptr, //horizontal tracking wheel 2
	&im //intertial sensor
};

//linear controller (odom)
lemlib::ControllerSettings linearController { //(-0.1,0]
	15, //kP
	0, //kI ???
	30, //kD
	3, //anti-windup (idk what this is or what to make it)
	0.5, //small error
	100, //small error timeout
	1.5, //large error
	500, //large error timeout
	20 //slew rate ???
};

//angular controller (odom)
lemlib::ControllerSettings angularController {
	1.9, //kP
	0, //kI ???
	10, //kD
	3, //anti-windup (idk what this is or what to make it)
	0.5, //small error
	100, //small error timeout
	1.5, //large error
	500, //large error timeout
	0 //slew rate ???
};

//chassis setup (finally)
//bruh it doesnt let me name it "chassis" even though thats what it says in docs
lemlib::Chassis chassi(drivetrain, linearController, angularController, odomSensors);

//brain screen
void screen(){
	while (true){
		//print current position info to brain
		//x and y don't work because inertial only works for turning
		lemlib::Pose pose = chassi.getPose();
		pros::lcd::print(0, "x: %f", pose.x);
		pros::lcd::print(1, "y: %f", pose.y);
		pros::lcd::print(2, "heading: %f", pose.theta);
		pros::lcd::print(3, "you're a bit skibidi");
		// master.print(1, 0, "%.2f", master.get_analog(ANALOG_LEFT_Y));
		// master.print(2, 0, "%.2f", master.get_analog(ANALOG_RIGHT_Y));
		pros::delay(10);
	}
}

//initialize
void initialize(){
	pros::lcd::initialize();
	chassi.calibrate();
	chassi.setPose(-43.033,-64.494,180);
	pros::Task screenTask(screen);
}

//auton
//load pure pursuit path
ASSET(crossover_txt);
void autonomous() {
	// SKILLZ
	//go to matchload spot
	chassi.moveToPoint(-59.732,-49.971,2000,false,127.0f,false);
	//turn to face cata
	chassi.turnTo(37.403,-1.933,1000,false,127.0f,false);
	//move back to touch bar
	chassi.moveToPoint(-58.732,-49.971,2000,true,127.0f,false);
	//turn on cata
	pneum.set_value(true);
	cata1.move_velocity(300);
	//matchload
	for (double i=0;i<=35;i+=0.01){
		master.print(1, 0, "%.2fs",35-i);
		pros::delay(10);
	}
	cata1.brake();
	pneum.set_value(false);
	//push in ball(s)
	chassi.turnTo(-62.863,-68,2000);
	chassi.moveToPoint(-62.863,-34.75,2000);
	//crossover to other side
	chassi.follow(crossover_txt, 15.0f,7000,false,false);
	//left
	chassi.turnTo(45,-10.05,1000);
	chassi.moveToPoint(45,-10.05,1500,false);
	pneum.set_value(true);
	//before -3 and -11 (10 and 3 now)
	pros::delay(500);
	chassi.moveToPoint(20,-11.05,1500,true);
	chassi.moveToPoint(45,-11.05,1500,false);
	pros::delay(500);
	chassi.moveToPoint(20,-3.05,3000, true);
	//right
	chassi.moveToPoint(45,3.05,1500,false);
	pros::delay(500);
	chassi.moveToPoint(20,3.05,1500,true);
	chassi.moveToPoint(45,3.05,1500,false);
	pros::delay(500);
	chassi.moveToPoint(20,3.05,3000, true);
	pneum.set_value(false);
}

//driver control
bool pneumOut = false;
bool cataOn = false;

void opcontrol(){
	// //go to matchload spot
  // chassi.moveToPoint(-59.732,-49.971,2000,false,127.0f,false);
	// //turn to face cata
	// chassi.turnTo(37.403,-1.933,1000,false,127.0f,false);
	// //move back to touch bar
	// chassi.moveToPoint(-58.732,-49.971,2000,true,127.0f,false);
	// //turn on cata
	// cataOn = true;
	// cata1.move_velocity(100);
	// //cata2.move_velocity(100);
	while (true){
		//drive (mabe change later)
		chassi.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y), 2.7);
		//intake
		if (master.get_digital(DIGITAL_L1)){
			intake.move_velocity(300);
		} else if (master.get_digital(DIGITAL_L2)) {
			intake.move_velocity(-300);
		} else {
			intake.brake();
		}
		//puncher
		if (master.get_digital_new_press(DIGITAL_R1)){
			if (!cataOn){
				cataOn = true;
				cata1.move_velocity(300);
				//cata2.move_velocity(100);
			}
		}
		if (master.get_digital_new_press(DIGITAL_R2)){
			if (cataOn){
				cataOn = false;
				cata1.brake();
				//cata2.brake();
			}
		}
		//pneumatics
		if (master.get_digital_new_press(DIGITAL_X)){
			pneumOut = !pneumOut;
			pneum.set_value(pneumOut);
			master.rumble(".");
		}
		pros::delay(10);
	}
}
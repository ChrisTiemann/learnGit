// custom function includes
#include "Navio/ScaleVars.h" // functions for re-scaling a value within a specified output range
// log file includes
#include <fstream>
#include <string>
// standard includes
#include <cstdlib>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <iomanip> // used to force GPS data to output with correct precision
// Navio2 includes
// barometer includes
#include "Navio/MS5611.h"
#include <cmath>
// RC Input includes
#include <Navio/RCInput.h>
// IMU Includes
#include "Navio/MPU9250.h"
#include "Navio/LSM9DS1.h"
// PWM Output Includes
#include "Navio/PWM.h"
#include "Navio/Util.h"
// AHRS Includes
#include "AHRS2.hpp"
// ADC Includes
#include <Navio/ADC.h>
// GPS Includes
#include "Navio/Ublox.h"

using namespace std;

// switch case parameters
char steer_type = 'r'; // 's' = step, 'm' = sine, 'r' = rc control, 'p' = prescribed, 'c' = controller (PID)
char throttle_type = 'r' ; // 's' = step, 'm' = sine, 'r' = rc control 'p' = prescribed, 'c' = controller (PID)

float motor_cmd = 0;
float blade_pitch_cmd = 0;
#define MOTOR 13

string pitch_cmd_msg;
//string pitch_left_cmd_msg;
// step parameters
int step_counter_steer;
int step_counter_throttle;
float steer_step_freq = .5; //hz
float throttle_step_freq = 1; //hz
// sine parameters
float omega_steer = .25; //hz
float omega_throttle = 2; //hz
float amplitude_steer = .5; //rc pwm
float amplitude_throttle = .5; //rc pwm
// step parameters (Truck Bullshit)
#define NUM_STEPS_STEER 5
#define NUM_STEPS_THROTTLE 2
#define STEP_SIZE_STEER 0.20
#define STEP_SIZE_THROTTLE 0.05
#define INITIAL_DEFLECTION_STEER 0.40
#define INITIAL_DEFLECTION_THROTTLE 0.00

//Roll
float roll_desired = 0, roll_actual, roll_cmd, roll_rate;
float roll_error, roll_error_old;
float trap_roll, trap_roll_old;

float roll_trim = 0;
float roll_offset = 0; //degrees

float K_roll = 0;
float D_roll = 0; 
float I_roll = 0;

//Pitch
float K_pitch = 0;
float D_pitch = 0; 
float I_pitch = 0;

float pitch_desired = 0, pitch_actual, pitch_cmd, pitch_rate;
float pitch_error, pitch_error_old;
float trap_pitch, trap_pitch_old;

//Yaw
float yaw_cmd, yaw_rate_desired, yaw_rate;
float yaw_error, yaw_error_old;
float trap_yaw, trap_yaw_old;

//PID Gains
float K = 0.0015;
float D = 0.0005;
float I = 0.0;

//PID Gains - Yaw
float K_yaw = .00012;
float D_yaw = 0.0001; 
float I_yaw = 0;

//Other PID Vars
float blade_pitch_cmd_1, blade_pitch_cmd_2, blade_pitch_cmd_3, blade_pitch_cmd_4;


//---------------------------------------------------------------------------------------------------User Configurable Parameters
const bool dbmsg_global = false; // set flag to display all debug messages
bool dbmsg_local  = false; // change in the code at a specific location to view local messages only
char control_type = 'x'; // valid options:  p=PID , n=NDI , g=glide (no spin), m=multisine , s=input sweep, c=rc control
char heading_type = 'd'; // valid otpoins 1=N, 2=E, 3=S, 4=W, u=user, c=rc control, n=navigation algorithm, d=dumb navigation
float user_heading = 115; //degress, only used if us is the heading type
bool live_gains = false;


//------------------------------------------------------------------------------------------------------------System ID Variables
float A           = -0.7391;
float B           = -43.362;
float C           = 17.417;
float wn_start    = .5;
float zeta_start  = 2;
float wn          = 0;
float zeta        = 0;
float ki_ndi_start = .005;

//-------------------------------------------------------------------------------------------------------Common Control Variables

string heading_type_message = "null";
float yaw_error_sum  = 0;
float yaw_error_rate = 0;
int num_wraps = 0;


//---------------------------------------------------------------------------------------------------------------Step Input Timer

//-------------------------------------------------------------------------------------------Loop timing (scheduler) Declarations

#define NUM_LOOPS  9
bool main_loop_active = true; // will bind this to an RC channel later for online tuning
struct timeval time_obj;
unsigned long long tse; // time since epoch (us)
unsigned long long time_start; // time from the beginning of current program executing (us)
unsigned long long time_now; // current time since the program starting executing (us)
unsigned long long tsd; //time since deployment (us)
unsigned long long time_of_activation; //time from the biginning of the activation loop

bool set_time_flag = true; //Tells the program to grab the current time on its first itteration

//---------------------------------------------------------------------------------------------------------Barometer Declarations

int baro_step = 0; // declare the step counter (used to delay time from read to caclulate)
MS5611 barometer; // declare a new barometer object
// using the barometric equation in region 1 (valid for elevations up to 36,000ft)
// see wikipedia.org/wiki/Barometric_formula for more information
// using imperial units (some values use strange mixed units, Kelvin with imperial length?), has been tested and output looks ok
// CONSTANTS
const float Pb =  29.92126;    // static reference pressure for barometric equation (inHg)
const float Tb = 288.815;      // reference temperature for barometric equation (K) !!!not currently using this in the calculation
const float Lb =   -.0019812;  // standard temperature lapse rate (K/ft)
const float hb =   0.0;        // refernce height (ft), could be omitted but kept for generality
const float  R =   8.9494596e4;// universal gas constant (lbft^2)/(lbmolKs^2)
const float g0 =  32.17405;    // acceleration due to earth's gravity near the surface (ft/s^2)
const float  M =  28.9644;     // molar mass of earth's air (lb/lbmol)
// VARIABLES
float Tc  = 0.0; // temperature (C) <--read directly from sensor
float Tf  = 0.0; // temperature (F) <--this is not used for any calcs
float Tk  = 0.0; // temperature (K) <--converted for use in barometric equation
float Pm  = 0.0; // pressure (mbar) <--read directly from sensor
float Phg = 0.0; // pressure (inHg) <--converted for use in barometric equation
float msl = 0.0; // mean sea level altitue (ft) [should be close to 920ft for UMKC Flarsheim]

//----------------------------------------------------------------------------------------------------------RC Input Declarations
RCInput rcinput{}; const float input_range[2] = {1088,1940}; // range is the same for all channels
// for PID tuning
const float output_range[6][2] = {{-.5,.5},{-.5,.5},{.9,1.9},{-.5,.5},{-.5,.5},{-.5,.5}};
float coefficients[6][2];

//---------------------------------------------------------------------------------------------------------------IMU Declarations
//#define DECLINATION 1.70 //magnetic declination for KC
#define DECLINATION 12.71 //magnetic declination for camp roberts
#define WRAP_THRESHOLD 160.00 // wrap threshold for wrap counter (yaw is +/-180, need to make it continuous
// vars to hold mpu values
float a_mpu[3] , a_mpu_ahrs[3];
float g_mpu[3] , g_mpu_ahrs[3];
float m_mpu[3] ;
// vars to hold lsm values
float a_lsm[3] , a_lsm_ahrs[3];
float g_lsm[3] , g_lsm_ahrs[3];
float m_lsm[3] ;
//create simple gyro integration for yaw as a fallback
float yaw_mpu_integrated = 0 , yaw_lsm_integrated = 0;
float yaw_mpu_integrated_degrees = 0 , yaw_lsm_integrated_degrees = 0;
float yaw_mpu_integrated_previous = 0 , yaw_lsm_integrated_previous = 0;
//also need a simple que to store old data points
float gyro_z_lsm_old[3] = {0,0,0};
float gyro_z_mpu_old[3] = {0,0,0};

//-------------------------------------------------------------------------------------------------------------Servo Declarations
#define PITCH_1 0
#define PITCH_2 1
#define PITCH_3 2
#define PITCH_4 3

#define SERVO_NEUTRAL 1.5
#define SATURATION .2

float pitch_setpoint = 0; // 0.15 This is the amount of pitch the rotor blades should have at 'neutral pitch' 
float servo_neutral_odd;
float servo_neutral_even;

float t = 0;

//--------------------------------------------------------------------------------------------------------------AHRS Declarations
#define PI 3.14159
#define G_SI 9.81
AHRS ahrs_mpu_mahony; // create a new object of type AHRS for the Mahony filter
AHRS ahrs_lsm_mahony;
AHRS ahrs_mpu_madgwick; // create a new object of type AHRS for the Madgwick filter
AHRS ahrs_lsm_madgwick;
float roll_mpu_mahony   = 0 , pitch_mpu_mahony   = 0 , yaw_mpu_mahony   = 0; // euler angles for mpu mahony
float roll_lsm_mahony   = 0 , pitch_lsm_mahony   = 0 , yaw_lsm_mahony   = 0; // euler angles for lsm mahony
float roll_mpu_madgwick = 0 , pitch_mpu_madgwick = 0 , yaw_mpu_madgwick = 0; // euler angles for mpu madgwick
float roll_lsm_madgwick = 0 , pitch_lsm_madgwick = 0 , yaw_lsm_madgwick = 0; // euler angles for lsm mahony
float offset_mpu[3];
float offset_lsm[3];
float dt, maxdt;
float mindt = 0.01;
float dtsumm = 0;
int isFirst = 1;

//Campus Quad Offsets
float mag_offset_mpu[3] = {19.236,7.985,59.101};
float mag_rotation_mpu[3][3] = {{.769,-.166,.063},{-.166,.725,.075},{.063,.075,.976}};
float mag_offset_lsm[3] = {-5.388,1.707,72.647};
float mag_rotation_lsm[3][3] = {{.843,-.192,.088},{-.192,.677,.112},{.088,.112,.949}};

//---------------------------------------------------------------------------------------------------------------GPS Declarations double
double time_gps = 0;
double lat = 0;
double lng = 0;
double alt_ellipsoid = 0;
double msl_gps = 10000;
double horz_accuracy = 0;
double vert_accuracy = 0;
int status_gps = 0x00; // default condition - no fix
string status_gps_string = "no fix";

//----------------------------------------------------------------------------------------------------------Waypoint Declarations
float prescribed_input[3001][2]; // waypoint array is 3000x2
//double target[2] = {35.7178528,-120.76411}; // from step input payload drop
double target[2] = {39.016998,-94.585846}; // intersection of 61st street and Morningside

//-----------------------------------------------------------------------------------------------------------Logfile Declarations
// these are used to format the filename string
#define FILENAME_LENGTH 12
#define FILENAME_OFFSET 4
// leave extra room for '.csv' and potential '-1', '-2', etc.
char filename[FILENAME_LENGTH+6];
// log file location (relative path)
string file_location = "/home/pi/Navio2/C++/Examples/NewCode_AutoRotate/LogFiles/";
// function to check if file exists so that the filename can be dynamically changed
bool file_exists(string name_of_file){
	ifstream checkfile(name_of_file); // try to read from the file
	return bool(checkfile);} // cast the result to bool, if file opens, this returns true (it exists)

int main( int argc , char *argv[])
{
	//----------------------------------------------------------------------------------------------------------------File Title Prefix
	int multisine_counter = 0;
	int parameter; // parameter is the argument passed in with the program call, create it here
	char *logfile_prefix; // pointer to a character array which will contain the log_file prefix when using the -d parameter
	string logfile_prefix_string; // previous variable will be converted to a string before using it to make a file
	logfile_prefix_string = file_location; // used to store the path
	string logfile_prefix_fromfile; // used to store the prefix when using the -f parameter
	ifstream ifs; // create a new file stream object, this will be reused for each file we need to read
	while((parameter = getopt(argc,argv, "hfd:")) != -1){ // read in the program options
		switch(parameter){ // switch on the character after the dash example:  sudo ./NewCode -d <filename>"
			case 'h' : // h parameter calls the help message
				cout << "Use option \"-d <FilePrefix>\" to insert a prefix into the filename" << endl;
				cout << "Use option \"-f\" to read configuration from file (configuration.csv)" << endl;
				return EXIT_FAILURE; // do not continue executing after displaying the help message
				break;
			case 'f' : // f parameter reads important variables from a configutation.txt file
				cout << "Reading user configuration from file" << endl;
				ifs.open("configuration.txt"); // open the configuration file
				ifs >> control_type; // read in the control type (character)
				cout << "Control type: " << control_type << endl; // echo to console
				ifs >> heading_type; // read in the heading type (character)
				cout << "Heading type: " << heading_type << endl; // echo to console
				ifs >> user_heading; // read in the heading (number), only used for user heading
				cout << "User heading: " << user_heading << " degrees" << endl; // echo to the console
				getline(ifs,logfile_prefix_fromfile); // this is a hack to get to the next line
				getline(ifs,logfile_prefix_fromfile); // read in the prefix from the configuration file
				logfile_prefix_string = file_location+logfile_prefix_fromfile+'_'; // stick an underscore after the file prefix
				cout << "Logfile prefix: " << logfile_prefix_fromfile << endl; // echo to the console
				ifs.close(); // close the file
				break;
			// this option reads in a prefix from the program call and uses the defaults for everything else
			case 'd' : logfile_prefix = optarg; logfile_prefix_string = file_location+string(logfile_prefix)+'_'; break;
			// stop execution if option flag is invalid
			case '?' : cout << "Invalid option.  Use -h for help" << endl; return EXIT_FAILURE;}}


	cout << endl;
	cout << "=======================================" << endl;
	cout << "         Begin Initialiation           " << endl;
	cout << "=======================================" << endl;
	cout << endl;
	cout << "Reading mag calibration from file......" << endl;
	ifs.open ("/home/pi/Navio2/C++/Examples/NewCodeTruck/mpu_mag_cal.csv"); // name of the mpu mag calibration file
	if(ifs){ // if successful, read the values and print a message
		cout << "MPU9250 offsets:" << endl;
	for(int i = 0 ; i < 3 ; i++ ){ // colum iterator, offsets are the first row (1x3) inside the 4x3 data file
		if(i != 0){ // on the first and last iteration there is no comma to catch
			cout << ", "; // print a comma to make output readable
			char delim; // create a character inside this scope, it is just a dummy variable
			ifs >> delim;} // stream the comma to the dummy character so it can be discarded
		ifs >> mag_offset_mpu[i]; // stream the offsets into the correct position in the offset array
		cout << mag_offset_mpu[i];} // output the offset array to the console
	cout << endl; // end line, prepare to read/write the rotation matrix
	cout << "MPU9250 rotation matrix:" << endl;
	for(int i = 0 ; i < 3 ; i++ ){ // row iterator, matrix is 3x3
		for(int j = 0 ; j < 3 ; j++ ){ // column iterator
			if(j != 0){ // on the first and last iteration there is no comma to catch
				cout << ", "; // print a comma to make the output readable
				char delim; // create a character inside this scope, it is just a dummy variable
				ifs >> delim;} // stream the comma to the dummy character so it can be discarded
			ifs >> mag_rotation_mpu[i][j]; // stream the rotation matrix values to the correct positions in the array
			cout << mag_rotation_mpu[i][j];} // output the rotation matrix to the console
		cout << endl;}}
	else{ // if the file open operation is not successful
	cout << "Cannot read MPU offsets, using defaults" << endl;} // print that the file could not be read, this is not a fatal error
	ifs.close(); // close the current file to prepare for the next file read operation

	// propogate the prescribed_input matrix with 0's, the value 0 will be used to mark the end of valid data
	for(int i = 0 ; i < 3000 ; i++ ){
		for(int j = 0 ; j < 2 ; j++){
			//cout << "we made it to here" << endl;
			prescribed_input[i][j] = 0;}} // fill with zeros, this will mark the EOF

	// read in the prescribed_input file
	ifs.open("/home/pi/Navio2/C++/Examples/NewCode_AutoRotate/prescribed_input.csv");
	if(ifs){
		cout << "Reading prescribed_input from file............" << endl;
		cout << "steer\tthrottle" << endl;
		for(int i = 0 ; i < 3000 ; i++ ){ // row iterator
			for(int j = 0 ; j < 2 ; j++){ // column iterator
				if(j!=0){
					if(prescribed_input[i][j] != 0){
						cout << ",";}
					char delim;
					ifs >> delim;}
				ifs >> prescribed_input[i][j];
				}

			}
		prescribed_input[3001][0] = 0;
		prescribed_input[3001][1] = 0;
		for(int i = 0 ; i < 3000 ; i++){
			for(int j = 0 ; j < 2 ; j++){
				if(prescribed_input[i][0]!=0){
					cout << prescribed_input[i][j];
					if(j!=1){
						cout << ",";}}}
				if(prescribed_input[i+1][0] != 0){
					cout << endl;}}
		}
	else{
		cout << "Could not read prescribed input" << endl;
		if(throttle_type == 'p' || steer_type == 'p'){
			cout << "Halting execution, you have specified a prescribed" << endl;
			cout << "input is to be used but have not provided a file." << endl;
			cout << "Please re-run with a file in the same directory as" << endl;
			cout << "this code called 'prescribed_input.csv'." << endl;
			// end execution if a navigation heading is being used but there are not prescribed_input
			return EXIT_FAILURE;}
	}



	//-------------------------------------------------------------------------------------------Loop timing (scheduler) Initialization
	// initialize the time variables
	gettimeofday(&time_obj, NULL); // standard unix function to get the current time since epoch
	tse          = time_obj.tv_sec*1000000LL + time_obj.tv_usec; // time since epoch (converted to microseconds)
	time_start = tse; // time since epoch (ms), will be used to calculate time since start (beginning at 0us)

	// declare the scheduler frequencies
	cout << endl;
	cout << "Establishing loop frequencies.........." << endl;
	// 8-4-2017 changed all timer related variables to long long, timer variable (int) was overflowing and causing erratic timing
	// if code had been running for ~35 minutes, tested after fixing and overflow is no longer occuring after 35 minutes
	const float frequency [NUM_LOOPS] = {300,1000,1,100,50,20,steer_step_freq,throttle_step_freq,1}; //Hz
	unsigned long long duration [NUM_LOOPS]; // stores the expected time since last execution for a given loop
	unsigned long long timer [NUM_LOOPS]; // stores the time since the last execution in a given loop
	unsigned long long watcher [NUM_LOOPS]; // used for monitoring actual timer loop durations

	// calculate the scheduler us delay durations and populate the timer array with zeros
	cout << "Calculating loop delay durations......." << endl;
	cout << "Populating loop timers................." << endl;
	for(int i = 0 ; i < NUM_LOOPS ; i++){ // populate the durations using the passed values for frequency, echo everything to the console
		duration[i] = 1000000/frequency[i];
		if(dbmsg_global || dbmsg_local){cout << "Frequency is " << frequency[i] << "(Hz), duration is " << duration[i] << "(us)"<< endl;}
		timer[i] = 0;
		if(dbmsg_global || dbmsg_local){cout << "Timer is set to " << timer[i] << "(us), should all be zero" << endl;}
		watcher[i] = 0;}
	//---------------------------------------------------------------------------------------------------------Barometer Initialization
	cout << "Initializing barometer................." << endl;
	barometer.initialize();
	cout << " --Barometer successfully initalized-- " << endl;
	//----------------------------------------------------------------------------------------------------------RC Input Initialization
	cout << "Initilizing RC Input..................." << endl;
	rcinput.init();
	int rc_array [rcinput.channel_count]; // array for holding the values which are read from the controller
	float rc_array_scaled [rcinput.channel_count]; // array for holding the scaled values
	for(int i = 0 ; i < rcinput.channel_count ; i++ ){
		rc_array[i] = rcinput.read(i);} // read in each value using the private class function (converts to int automatically)
	cout << "Currently using " << rcinput.channel_count << " channels............." << endl;
	cout << " --RC Input successfully initialized-- " << endl;
	cout << "Creating RC Input range coefficients   " << endl;
	for(int i = 0 ; i < rcinput.channel_count ; i++){ // using a custom class for converting bounded values to values in a new range
	// in the Python version of this function, both parameters are returned by one function
	// since multiple values may not be returned by a single function this version uses a function for each scaling parameter
		coefficients[i][0] = calculate_slope(input_range,output_range[i]); // first call the slope calculator
		coefficients[i][1] = calculate_intercept(input_range,output_range[i],coefficients[i][0]); // next calculate the intercept
	}
	cout << "Successfully created range coefficients" << endl;

	//---------------------------------------------------------------------------------------------------------------IMU Initialization
	cout << "Initializing IMUs......................" << endl;
	InertialSensor *mpu;
	mpu = new MPU9250();
	mpu->initialize();
	cout << " --MPU9250 successfully initialized--" << endl;
	InertialSensor *lsm;
	lsm = new LSM9DS1();
	lsm->initialize();
	cout << " --LSM9DS1 successfully initialized--" << endl;
	cout << "Populating IMU sensor arrays..........." << endl;
	for(int i = 0; i < 3 ; i++){ // seed everything with a value of 0
		a_mpu[i]=g_mpu[i]=m_mpu[i]=a_lsm[i]=g_lsm[i]=m_lsm[i] = 0.0;}
	//-------------------------------------------------------------------------------------------------------------Servo Initialization
	cout << "Initializing PWM Output................" << endl;
	PWM pwm_out;
	// create pwm output object and initialize right and left winch servos, stop execution if servos cannot be initialized
	// NOTE!!! if the code is erroring out here, the first thing to check is to make sure that you are are running the code with sudo

	if(!pwm_out.init(PITCH_1)){ 
		cout << "Cannot Initialize Pitch Servo Number 1" << endl;
		cout << "Make sure you are root" << endl;
		return EXIT_FAILURE;}
	if(!pwm_out.init(PITCH_2)){ 
		cout << "Cannot Initialize Pitch Servo Number 2" << endl;
		cout << "Make sure you are root" << endl;
		return EXIT_FAILURE;}
	if(!pwm_out.init(PITCH_3)){
		cout << "Cannot Initialize Pitch Servo Number 3" << endl;
		cout << "Make sure you are root" << endl;
		return EXIT_FAILURE;}
	if(!pwm_out.init(PITCH_4)){ 
		cout << "Cannot Initialize Pitch Servo Number 4" << endl;
		cout << "Make sure you are root" << endl;
		return EXIT_FAILURE;}
	if(!pwm_out.init(MOTOR)){ 
		cout << "Cannot Initialize the motor" << endl;
		cout << "Make sure you are root" << endl;
		return EXIT_FAILURE;}
		
	cout << "Enabling PWM output channels..........." << endl;
	pwm_out.enable(PITCH_1);
	pwm_out.enable(PITCH_2);
	pwm_out.enable(PITCH_3);
	pwm_out.enable(PITCH_4);
	pwm_out.enable(MOTOR);
	cout << "Setting PWM period for 50Hz............" << endl;
	pwm_out.set_period(PITCH_1, 50);
	pwm_out.set_period(PITCH_2, 50);
	pwm_out.set_period(PITCH_3, 50);
	pwm_out.set_period(PITCH_4, 50);
	pwm_out.set_period(MOTOR, 50);
	cout << "  --PWM Output successfully enabled-- " << endl;
	
	pwm_out.set_duty_cycle(MOTOR, 1);
	usleep(2000);
	pwm_out.set_duty_cycle(MOTOR, 2);
	usleep(2000);
	pwm_out.set_duty_cycle(MOTOR, 1);
	
	cout << "Setting Servo Neutral for Even and Odd Pitch Mechanisms" << endl;
	servo_neutral_even = SERVO_NEUTRAL + pitch_setpoint;
	servo_neutral_odd  = SERVO_NEUTRAL - pitch_setpoint;
	cout << " --Pitch Mechanisms successfully initialized--" << endl;

	//--------------------------------------------------------------------------------------------------------------ADC Initialization
	cout << "Initializing ADC......................." << endl;
	ADC adc{};
	adc.init();
	float adc_array[adc.get_channel_count()] = {0.0f};
	for(int i = 0 ; i < ARRAY_SIZE(adc_array) ; i++){
		adc_array[i] = adc.read(i);}
	cout << "     --ADC successfully enabled--     " << endl;

	//-------------------------------------------------------------------------------------------------------------AHRS Initialization
	cout << "Initializing gyroscope................." << endl;
	cout << "Reading gyroscope offsets.............." << endl;
	// gyroscope offsets generated by sampling the gyroscope 100 times when the program executes, averaging the 100 samples
	// and then subtracting off the newly acquired offset values evertime the gyro is read later on in the code
	for(int i = 0 ; i < 100 ; i++){
		// read both sensors, we are only concerned with gyroscope information right now
		mpu->update(); // update the mpu sensor
		mpu->read_gyroscope(&g_mpu[0],&g_mpu[1],&g_mpu[2]); // read the updated info
		lsm->update(); // update the lsm sensor
		lsm->read_gyroscope(&g_lsm[0],&g_lsm[1],&g_lsm[2]); // read the updated info
		//read 100 samples from each gyroscope axis on each sensor
		g_mpu[0] *= 180/PI;
		g_mpu[1] *= 180/PI;
		g_mpu[2] *= 180/PI;
		g_lsm[0] *= 180/PI;
		g_lsm[1] *= 180/PI;
		g_lsm[2] *= 180/PI;
		//populate the offset arrays
		offset_mpu[0] += (-g_mpu[0]*0.0175);
		offset_mpu[1] += (-g_mpu[1]*0.0175);
		offset_mpu[2] += (-g_mpu[2]*0.0175);
		offset_lsm[0] += (-g_lsm[0]*0.0175);
		offset_lsm[1] += (-g_lsm[1]*0.0175);
		offset_lsm[2] += (-g_lsm[2]*0.0175);
		//wait a bit before reading gyro again
		usleep(10000);}
	cout << "Calculating gyroscope offsets.........." << endl;
	// average the offsets for the mpu
	offset_mpu[0]/=100.0;
	offset_mpu[1]/=100.0;
	offset_mpu[2]/=100.0;
	// average the offsets for the lsm
	offset_lsm[0]/=100.0;
	offset_lsm[1]/=100.0;
	offset_lsm[2]/=100.0;
	cout << "Setting gyroscope offsets.............." << endl;
	// finally write the acquired offsets to the ahrs objects which have a member function which automatically handles
	// the application of offsets at each update/read cycle
	ahrs_mpu_mahony.setGyroOffset(offset_mpu[0],offset_mpu[1],offset_mpu[2]);
	ahrs_lsm_mahony.setGyroOffset(offset_lsm[0],offset_lsm[1],offset_lsm[2]);
	ahrs_mpu_madgwick.setGyroOffset(offset_mpu[0],offset_mpu[1],offset_mpu[2]);
	ahrs_lsm_madgwick.setGyroOffset(offset_lsm[0],offset_lsm[1],offset_lsm[2]);
	cout << " --Gyroscope offsets stored in AHRS--  " << endl;
	cout << "Setting magnetometer calibration......." << endl;
	ahrs_mpu_mahony.setMagCalibration(mag_offset_mpu,mag_rotation_mpu);
	ahrs_lsm_mahony.setMagCalibration(mag_offset_lsm,mag_rotation_lsm);
	ahrs_mpu_madgwick.setMagCalibration(mag_offset_mpu,mag_rotation_mpu);
	ahrs_lsm_madgwick.setMagCalibration(mag_offset_lsm,mag_rotation_lsm);
	cout << "--Magnetometer offsets stored in AHRS--" << endl;
	cout << "-Magnetometer rotations stored in AHRS-" << endl;

	//---------------------------------------------------------------------------------------------------------------GPS Initialization
	vector<double> pos_data; // this vector will contain undecoded gps information
	int prescribed_input_index = 0; // this is the counter for the current altitude level used in the navigation algorithm
	Ublox gps;
	cout << "Initializing GPS......................." << endl;
	if(!gps.testConnection()){ // check if the gps is working
		cout << "    --ERROR, GPS not initialized--    " << endl;
		if(heading_type == 'n'){
			// stop program execution if gps won't initialize and navigation heading type is selected
			cout << "Fatal exception, navigation impossible" << endl;
			cout << "without GPS, try restarting..........." << endl;
			return EXIT_FAILURE;}}
	else{ // gps is good!
		cout << "  --GPS successfully initialized--    " << endl;}

	//------------------------------------------------------------------------------------------------------------------Welcome Message
	usleep(500000);
	cout << endl;
	cout << "=======================================" << endl;
	cout << "Initialization completed..............." << endl;
	cout << "=======================================" << endl << endl;
	usleep(500000);
	cout << "=======================================" << endl;
	cout << "Main Loop starting now................." << endl;
	cout << "=======================================" << endl << endl;
	usleep(500000);

	//----------------------------------------------------------------------------------------------------------Log File Initialization
	while(true)
	{
		int standby_message_timer = 0; // used to limit the frequency of the standby message
		//while(!(adc_array[4] > 4000)){
		while(!((rc_array[5]<1500))){
//bool temp_flag = false; // uncomment for testing with no transmitter
//while(!temp_flag){ // uncomment for testing with no transmitter
			if(standby_message_timer > 250){
				cout << endl << "---------------------------------------" << endl << "           Autopilot Inactive         " << endl;
				cout << "         Waiting for Killswitch       " << endl << "--------------------------------------" << endl;
				standby_message_timer = 0;}
			// when the autopilot is inactive, set both winches to the neutral deflection
			pwm_out.set_duty_cycle(PITCH_1, SERVO_NEUTRAL); // These are flipped so the pitch is positive for powered flight
			pwm_out.set_duty_cycle(PITCH_2, SERVO_NEUTRAL);
			pwm_out.set_duty_cycle(PITCH_3, SERVO_NEUTRAL);
			pwm_out.set_duty_cycle(PITCH_4, SERVO_NEUTRAL);

			// since this loop executes based on an rc and an adc condition, we have to poll these devices for new status
			rc_array[5] = rcinput.read(5);
			adc_array[4] = adc.read(4);
			// step counter needs to be set to zero also, this is a hack because the numbering is messed up in the code
			// we start at negative one here and presumably the counter is incremented before 1st execution
			step_counter_steer = -1;
			step_counter_throttle = -1;
			usleep(5000);
			standby_message_timer++; // increment the message delay timer
			// everything that needs to be set to zero by the killswitch goes here
			set_time_flag = true;
			multisine_counter = 0; // time counter for the multisine input
			yaw_mpu_integrated = 0; // integrated yaw
			yaw_mpu_integrated_previous = 0;
			yaw_lsm_integrated = 0;
			yaw_lsm_integrated_previous = 0;
			prescribed_input_index = 0; // wind level for navigation, can only be incremented by the main loop to avoid "waypoint indecision"
			for(int i = 0 ; i < 3 ; i++){
				gyro_z_lsm_old[i] = 0;
				gyro_z_mpu_old[i] = 0;
			}
			trap_yaw       		= 0; // prevent integral wind up
			yaw_error           = 0;
			yaw_error_old       = 0;
//temp_flag = true; // uncomment for testing with no transmitter

		}
		time_t result = time(NULL);
		char *today = asctime(localtime(&result)); // establish char array to write today's date
		today[strlen(today) - 1] = '\0'; // remove end line from the end of the character array
		// remove spaces, remove colons, only keep the month/day/hour/minute
		for(int i = 0 ; i < FILENAME_LENGTH ; i++){
			if(today[i+FILENAME_OFFSET] == ' ' || today[i+FILENAME_OFFSET] == ':'){
				if(i == 8 - FILENAME_OFFSET) {
					filename[i] = '0';} // add leading zero in front of 1 digit day
				else{
					filename[i] = '_';}}
			else{
				filename[i] = today[i+FILENAME_OFFSET];}}
		// add .csv to the end of the file
		filename[FILENAME_LENGTH+0] = '-';
		filename[FILENAME_LENGTH+1] = '0';
		filename[FILENAME_LENGTH+2] = '.';
		filename[FILENAME_LENGTH+3] = 'c';
		filename[FILENAME_LENGTH+4] = 's';
		filename[FILENAME_LENGTH+5] = 'v';
		// for adding the relative path to the file, it will be converted to a string
		string filename_str(filename);
		//cout << "filename_str" << filename_str << endl;
		filename_str = logfile_prefix_string+filename_str;
		// used to put a number on the end of the file if it is a duplicate
		int filename_index = 1;
		// loop while the file name already exists or until we are out of indices to write to
		cout << endl << "Setting up log file...................." << endl;
		cout << "Checking for unique file name.........." << endl;
		while(file_exists(filename_str))
		{
		cout << "Filename already exists................" << endl;
			filename[FILENAME_LENGTH+0] = '-';
			filename[FILENAME_LENGTH+1] = filename_index + '0';
			filename[FILENAME_LENGTH+2] = '.';
			filename[FILENAME_LENGTH+3] = 'c';
			filename[FILENAME_LENGTH+4] = 's';
			filename[FILENAME_LENGTH+5] = 'v';
			string temp(filename);
			filename_str = logfile_prefix_string+temp;
			filename_index++;
			usleep(50000);} // don't try to read the files at warp speed
		// create the file output stream object
		ofstream fout;
		// open the file
		cout << "Log file created at: " << endl << filename_str << endl << endl;
		fout.open(filename_str,ios::out);
		// header string
		fout << "today,microseconds_since_start,tsd,msl,"
			"rc_array[0],rc_array[1],rc_array[2],rc_array[3],rc_array[4],rc_array[5],"
			"rc_array_scaled[0],rc_array_scaled[1],rc_array_scaled[2],rc_array_scaled[3],rc_array_scaled[4],rc_array_scaled[5],"
			"adc_array[0],adc_array[1],adc_array[2],adc_array[3],adc_array[4],adc_array[5],"
			"roll_mpu_mahony,pitch_mpu_mahony,yaw_mpu_mahony,"
			"roll_lsm_mahony,pitch_lsm_mahony,yaw_lsm_mahony,"
			"roll_mpu_madgwick,pitch_mpu_madgwick,yaw_mpu_madgwick,"
			"roll_lsm_madgwick,pitch_lsm_madgwick,yaw_lsm_madgwick,"
			"a_mpu[0],a_mpu[1],a_mpu[2],"
			"g_mpu[0],g_mpu[1],g_mpu[2],"
			"m_mpu[0],m_mpu[1],m_mpu[2],"
			"roll_cmd,pitch_cmd,yaw_cmd,"
			"pitch_desired,roll_desired,yaw_rate_desired,"
			"blade_pitch_cmd_1,blade_pitch_cmd_2,blade_pitch_cmd_3,blade_pitch_cmd_4,"
			"K,D,I,K_yaw,D_yaw,SERVO_NEUTRAL,"
			"time_gps,lat,lng,alt_ellipsoid,"
			"msl_gps,horz_accuracy,vert_accuracy,status_gps,motor_cmd," << endl;
		usleep(20000);
		//everything that needs to be set to zero by the killswitch goes here
		multisine_counter = 0;
		yaw_mpu_integrated = 0;
		yaw_mpu_integrated_previous = 0;
		yaw_lsm_integrated = 0;
		yaw_lsm_integrated_previous = 0;
		prescribed_input_index = 0;
		for(int i = 0 ; i < 3 ; i++){
			gyro_z_lsm_old[i] = 0;
			gyro_z_mpu_old[i] = 0;
		}
		yaw_error_sum       = 0; //prevent integral wind up
		yaw_error           = 0;
		yaw_error_old  = 0;
		num_wraps = 0;
		

//while(adc_array[4] > 4000) 
while((rc_array[5]<1500))
//while(true)
	{
		// refresh time now to prepare for another loop execution
		gettimeofday(&time_obj, NULL); // must first update the time_obj
		tse        = time_obj.tv_sec*1000000LL + time_obj.tv_usec; // update tse (us)
		//tse        = tse + 2000000000; // uncomment to test integer overflow fix
		time_now   = tse - time_start; // calculate the time since execution start by subtracting off tse
		if(set_time_flag == true)
		{
			time_of_activation = time_now; //Time when the drop occurs
			set_time_flag = false;
		}
		tsd = time_now - time_of_activation; //tsd = time since deployment
		if( (time_now-timer[0]) > duration[0])
		{
			//----------------------------------------------------------------------------------------------------------------AHRS Update
			dt = time_now-timer[0];
			dt = dt/1000000.0; // convert from useconds

			//if(prescribed_input[prescribed_input_index][0] != 0){
			//	if(msl_gps < prescribed_input[prescribed_input_index][2]){
			//		prescribed_input_index++;}}

			// Tested sampling rate for IMUs, with both IMUs execution of the following block was taking
			// approximate 1300us (~750Hz), slowed this loop down ot 500Hz so that there is a little
			// headroom but the sensor is being polled as quickly as possible
			// Read both IMUs at 500Hz
			// start with the MPU9250
			mpu->update();
			mpu->read_accelerometer(&a_mpu[0],&a_mpu[1],&a_mpu[2]);
			mpu->read_gyroscope(&g_mpu[0],&g_mpu[1],&g_mpu[2]);
			mpu->read_magnetometer(&m_mpu[0],&m_mpu[1],&m_mpu[2]);

			// now read in the LSM9DS1
			lsm->update();
			lsm->read_accelerometer(&a_lsm[0],&a_lsm[1],&a_lsm[2]);
			lsm->read_gyroscope(&g_lsm[0],&g_lsm[1],&g_lsm[2]);
			lsm->read_magnetometer(&m_lsm[0],&m_lsm[1],&m_lsm[2]);

			for(int i = 0 ; i < 3 ; i++ ){
				a_mpu_ahrs[i] = a_mpu[i]/G_SI;
				g_mpu_ahrs[i] = g_mpu[i]*(180/PI);
				a_lsm_ahrs[i] = a_lsm[i]/G_SI;
				g_lsm_ahrs[i] = g_lsm[i]*(180/PI);}

			ahrs_mpu_mahony.updateMahony(a_mpu_ahrs[0],a_mpu_ahrs[1],a_mpu_ahrs[2],g_mpu_ahrs[0]*0.0175,g_mpu_ahrs[1]*0.0175,g_mpu_ahrs[2]*0.0175,m_mpu[0],m_mpu[1],m_mpu[2],dt);
			ahrs_lsm_mahony.updateMahony(a_lsm_ahrs[0],a_lsm_ahrs[1],a_lsm_ahrs[2],g_lsm_ahrs[0]*0.0175,g_lsm_ahrs[1]*0.0175,g_lsm_ahrs[2]*0.0175,m_lsm[0],m_lsm[1],m_lsm[2],dt);
			ahrs_mpu_madgwick.updateMadgwick(a_mpu_ahrs[0],a_mpu_ahrs[1],a_mpu_ahrs[2],g_mpu_ahrs[0]*0.0175,g_mpu_ahrs[1]*0.0175,g_mpu_ahrs[2]*0.0175,m_mpu[0],m_mpu[1],m_mpu[2],dt);
			ahrs_lsm_madgwick.updateMadgwick(a_lsm_ahrs[0],a_lsm_ahrs[1],a_lsm_ahrs[2],g_lsm_ahrs[0]*0.0175,g_lsm_ahrs[1]*0.0175,g_lsm_ahrs[2]*0.0175,m_lsm[0],m_lsm[1],m_lsm[2],dt);

			// update the que holding the old gyro values with the new gyro information
			gyro_z_mpu_old[2] = gyro_z_mpu_old[1];
			gyro_z_mpu_old[1] = gyro_z_mpu_old[0];
			gyro_z_mpu_old[0] = g_mpu_ahrs[2]-offset_mpu[2];
			gyro_z_lsm_old[2] = gyro_z_lsm_old[1];
			gyro_z_lsm_old[1] = gyro_z_lsm_old[0];
			gyro_z_lsm_old[0] = g_lsm_ahrs[2]-offset_lsm[2];
			// update previous integrated yaw variables
			yaw_mpu_integrated_previous = yaw_mpu_integrated;
			yaw_lsm_integrated_previous = yaw_lsm_integrated;

			//update the integrated gyro yaw (this is a fallback in case the heading is bad)
			yaw_mpu_integrated = (((gyro_z_mpu_old[0]+gyro_z_mpu_old[1])/2)*dt) + yaw_mpu_integrated_previous;
			yaw_mpu_integrated_degrees = yaw_mpu_integrated * (180/PI);
			yaw_lsm_integrated = (((gyro_z_lsm_old[0]+gyro_z_lsm_old[1])/2)*dt) + yaw_lsm_integrated_previous;
			yaw_lsm_integrated_degrees = yaw_lsm_integrated * (180/PI);

			watcher[0] = time_now - timer[0]; // used to check loop frequency
			timer[0] = time_now;
		}

		if( (time_now-timer[1]) > duration[1])
		{
			if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1)
			{
				// examples provided with Navio2 have gps code which is designed to run in a threaded program, the gps update
				// functions block execution, for this code, the functions have been modified, the gps is polled here at 1000hz
				// but valid new data only comes through occasionally, the NAV_STATUS message has been disabled entireley because
				// it was causing .75 second interruptions in the timer loop, in the future threading will be utilized for the
				// gps (then the imu and ahrs may be added to separate threads as well)
	                	time_gps = pos_data[0]/1000.00000;
		       	        lng = pos_data[1]/10000000.00000;
        	       		lat = pos_data[2]/10000000.00000;
	        	        alt_ellipsoid = (pos_data[3]/1000.00000)*3.28;
		               	msl_gps = (pos_data[4]/1000.00000)*3.28;
				//msl_gps = 2500-step_counter*160; //uncomment to test navigation algorithm
        		        horz_accuracy = (pos_data[5]/1000.00000)*3.28;
	                	vert_accuracy = (pos_data[6]/1000.00000)*3.28;
			}
			watcher[1] = time_now - timer[1]; // used to check loop frequency
			timer[1] = time_now;
			}

		if( (time_now-timer[2]) > duration[2])
		{
			watcher[2] = time_now - timer[2]; // used to check loop frequency
			timer[2] = time_now;
		}

		if( (time_now-timer[3]) > duration[3])
		{
			//-------------------------------------------------------------------------------------------------------------Barometer Read
			if(baro_step == 0){
				barometer.refreshPressure();}
			else if(baro_step == 1){
				barometer.readPressure();}
			else if(baro_step == 2){
				barometer.refreshTemperature();}
			else if(baro_step == 3){
				barometer.readTemperature();}
			else if(baro_step == 4){
				barometer.calculatePressureAndTemperature();
				baro_step = -1;}
			else{
				cout << "improper barometer step, pressure may be incorrect" << endl;}
			baro_step++; // increment the step

			//--------------------------------------------------------------------------------------------------------------RC Input Read
			for(int i = 0 ; i < rcinput.channel_count ; i++ )
			{
				rc_array[i] = rcinput.read(i); // read in each value using the private class function (converts to int automatically)
				if(dbmsg_global || dbmsg_local) {cout << "Channel Number: " << i << " Channel Value: "  << rc_array[i] << endl;}
			}
			for(int i = 0 ; i < rcinput.channel_count ; i++)
			{
				rc_array_scaled[i] = scale_output(coefficients[i],float(rc_array[i]));
			}

			//-----------------------------------------------------------------------------------------------------------------ADC Update
			for(int i = 0 ; i < ARRAY_SIZE(adc_array) ; i++){
				adc_array[i] = adc.read(i);}

			//------------------------------------------------------------------------------------------------------Euler Angle Conversion
			ahrs_mpu_mahony.getEuler(&roll_mpu_mahony,&pitch_mpu_mahony,&yaw_mpu_mahony);
			ahrs_lsm_mahony.getEuler(&roll_lsm_mahony,&pitch_lsm_mahony,&yaw_lsm_mahony);
			ahrs_mpu_madgwick.getEuler(&roll_mpu_madgwick,&pitch_mpu_madgwick,&yaw_mpu_madgwick);
			ahrs_lsm_madgwick.getEuler(&roll_lsm_madgwick,&pitch_lsm_madgwick,&yaw_lsm_madgwick);

			//----------------------------------------------------------------------------------------------------------------Controllers
			float dt_control = time_now-timer[3]; // dt for this loop
			dt_control = dt_control/1000000.0; // convert from useconds

			// make yaw continuous instead of +/-180 using a wrap counter
/* 			if(yaw_mpu_madgwick > WRAP_THRESHOLD && yaw_prev < - WRAP_THRESHOLD){
				num_wraps--;
			}
			if(yaw_mpu_madgwick < - WRAP_THRESHOLD && yaw_prev >  WRAP_THRESHOLD){
				num_wraps++;
			} */

			// account for magnetic declination
			yaw_mpu_mahony = yaw_mpu_mahony + DECLINATION;
			yaw_lsm_mahony = yaw_lsm_mahony + DECLINATION;
			yaw_mpu_madgwick = yaw_mpu_madgwick + DECLINATION;
			yaw_lsm_madgwick = yaw_lsm_madgwick + DECLINATION;
			
// PID /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
			
			//Roll
			roll_desired = 0; // Degrees
						
			roll_rate = g_mpu[0]; //Degrees/sec
			roll_actual = roll_mpu_madgwick; //Degrees
			roll_error_old = roll_error; 
			roll_error = (roll_desired) - (roll_actual);
			trap_roll = trap_roll_old; 
			trap_roll = (.01/2)*(roll_error + roll_error_old) + trap_roll_old;
						
			K_roll = K * (roll_error);
			D_roll = D * (-roll_rate);
			I_roll = I * (trap_roll);
			
			if (I_roll >= .05 ) { // Cap Integrator so it won't diverge
				trap_roll = 10;}
			
			roll_cmd = K_roll + D_roll + I_roll; //Build Roll Cmd
			
			
			//Pitch
			//pitch_desired = 0 + rc_array_scaled[1]; // Degrees
						
			pitch_rate = g_mpu[1]; //Degrees/sec
			pitch_actual = pitch_mpu_madgwick; //Degrees, Encoder Position minus bias in degrees
			pitch_error_old = pitch_error; 
			pitch_error = (pitch_desired) - (pitch_actual);
			trap_pitch = trap_pitch_old; 
			trap_pitch = (.01/2)*(pitch_error + pitch_error_old) + trap_pitch_old;
						
			K_pitch = K * (pitch_error);
			D_pitch = D * (-pitch_rate);
			I_pitch = I * (trap_pitch);
			
			if (I_pitch >= .05 ) { // Cap Integrator so it won't diverge
				trap_pitch = 10;}
			
			pitch_cmd = K_pitch + D_pitch + I_pitch; //Build Pitch Cmd
			
			
			//Yaw
			yaw_rate_desired = rc_array_scaled[3];
			
			yaw_rate = g_mpu[2]*(180/PI); // Degrees/sec
			yaw_error_old = yaw_error; 
			yaw_error = (yaw_rate_desired) - (yaw_rate);
			trap_yaw_old = trap_yaw;
			trap_yaw = (.01/2)*(yaw_error + yaw_error_old) + trap_yaw_old;
			
			if (I_yaw * trap_yaw >= 0 && I_yaw * trap_yaw > .06) { //Cap Integrator so it won't diverge
				trap_yaw = 30;}
			else if (I_yaw *trap_yaw < 0 && I_yaw * trap_yaw < -.06) {
				trap_yaw = -30;}
						
			yaw_cmd = K_yaw*(yaw_error) + D_yaw*(-yaw_rate) + I_yaw*(trap_yaw); // Build Yaw Cmd
			
				
// end of PID/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

			t = float(time_now)/1000000.0;

switch(steer_type){
				case 's' : {// step input
					pitch_cmd = (step_counter_steer * STEP_SIZE_STEER) - INITIAL_DEFLECTION_STEER;
					pitch_cmd_msg = "Step";
					break;}
				case 'r' : {// rc transmitter
					blade_pitch_cmd_1 = servo_neutral_even - rc_array_scaled[1]; //Blade pitch is flipped to provide positive pitch.
					blade_pitch_cmd_2 = servo_neutral_odd + rc_array_scaled[1];
					
				//Saturation Block
				if(blade_pitch_cmd_1 > servo_neutral_even + SATURATION){
					blade_pitch_cmd_1 = servo_neutral_even + SATURATION;}
				else if(blade_pitch_cmd_1 <  servo_neutral_even - SATURATION){
					blade_pitch_cmd_1 = servo_neutral_even - SATURATION;}
				
				if(blade_pitch_cmd_2 > servo_neutral_odd + SATURATION){
					blade_pitch_cmd_2 = servo_neutral_odd + SATURATION;}
				else if(blade_pitch_cmd_2 <  servo_neutral_odd - SATURATION){
					blade_pitch_cmd_2 = servo_neutral_odd - SATURATION;}

					motor_cmd = rc_array_scaled[2];
					pitch_cmd_msg = "RC transmitter";
					break;}
				case 'm' : {// sine
					pitch_cmd = amplitude_steer*sin(2*3.14*omega_steer*t);
					pitch_cmd_msg = "Multisine";
					break;}
				case 'p' : {// use prescribed input data from file
					pitch_cmd = prescribed_input[prescribed_input_index][0] - (rc_array_scaled[1]);;
					pitch_cmd = -prescribed_input[prescribed_input_index][0] + (rc_array_scaled[1]);
					//pitch_right_cmd = (pitch_right_cmd/3);
					pitch_cmd_msg = "From file";
					break;}
				case 'c' : {// controller (PID)
					blade_pitch_cmd_1 = servo_neutral_odd + pitch_cmd - roll_cmd + yaw_cmd; //Signs are not intuitive because of the direction the servos are mounted in relationship to the blade direction
					blade_pitch_cmd_2 = servo_neutral_even - pitch_cmd - roll_cmd + yaw_cmd;
					blade_pitch_cmd_3 = servo_neutral_odd - pitch_cmd + roll_cmd + yaw_cmd;
					blade_pitch_cmd_4 =	servo_neutral_even + pitch_cmd + roll_cmd + yaw_cmd;
					pitch_cmd_msg = "PID Control";
					
					// actuator saturation, pitch servos
			if(blade_pitch_cmd_1 > servo_neutral_odd + SATURATION){
				blade_pitch_cmd_1 = servo_neutral_odd + SATURATION;}
			else if(blade_pitch_cmd_1 <  servo_neutral_odd - SATURATION){
				blade_pitch_cmd_1 = servo_neutral_odd - SATURATION;}
				
			if(blade_pitch_cmd_2 > servo_neutral_even + SATURATION){
				blade_pitch_cmd_2 = servo_neutral_even + SATURATION;}
			else if(blade_pitch_cmd_2 <  servo_neutral_even - SATURATION){
				blade_pitch_cmd_2 = servo_neutral_even - SATURATION;}
				
			if(blade_pitch_cmd_3 > servo_neutral_odd + SATURATION){
				blade_pitch_cmd_3 = servo_neutral_odd + SATURATION;}
			else if(blade_pitch_cmd_3 <  servo_neutral_odd - SATURATION){
				blade_pitch_cmd_3 = servo_neutral_odd - SATURATION;}
				
			if(blade_pitch_cmd_4 > servo_neutral_even + SATURATION){
				blade_pitch_cmd_4 = servo_neutral_even + SATURATION;}
			else if(blade_pitch_cmd_4 <  servo_neutral_even - SATURATION){
				blade_pitch_cmd_4 = servo_neutral_even - SATURATION;}
					
					break;} 
				default : {
					pitch_cmd = 0;
					pitch_cmd_msg = "Default, None";
					break;}}

					
			// always write the duty cycle
			pwm_out.set_duty_cycle(PITCH_1, blade_pitch_cmd_1);
		    pwm_out.set_duty_cycle(PITCH_2, blade_pitch_cmd_2);
			pwm_out.set_duty_cycle(PITCH_3, blade_pitch_cmd_3);
			pwm_out.set_duty_cycle(PITCH_4, blade_pitch_cmd_4);
			pwm_out.set_duty_cycle(MOTOR, motor_cmd);

			//-------------------------------------------------------------------------------------------------------Data Log File Output
			fout << today << "," << time_now << "," << tsd << "," << msl << ",";
			for(int i = 0 ; i < rcinput.channel_count ; i++){
				fout << rc_array[i] << ",";}
			for(int i = 0 ; i < rcinput.channel_count ; i++){
				fout << rc_array_scaled[i] << ",";}
			for(int i = 0 ; i < ARRAY_SIZE(adc_array) ; i++ ){
				fout << adc_array[i] << ",";}
			fout << roll_mpu_mahony << "," << pitch_mpu_mahony << "," << yaw_mpu_mahony << ",";
			fout << roll_lsm_mahony << "," << pitch_lsm_mahony << "," << yaw_lsm_mahony << ",";
			fout << roll_mpu_madgwick << "," << pitch_mpu_madgwick << "," << yaw_mpu_madgwick << ",";
			fout << roll_lsm_madgwick << "," << pitch_lsm_madgwick << "," << yaw_lsm_madgwick << ",";
			fout << a_mpu[0] << "," << a_mpu[1] << "," << a_mpu[2] << ","; 
			fout << g_mpu[0] << "," << g_mpu[1] << "," << g_mpu[2] << ",";
			fout << m_mpu[0] << "," << m_mpu[1] << "," << m_mpu[2] << ",";
			fout << roll_cmd << "," << pitch_cmd << "," << yaw_cmd << ",";
			fout << roll_desired << "," << pitch_desired << "," << yaw_rate_desired << ",";
			fout << blade_pitch_cmd_1 << "," << blade_pitch_cmd_2 << "," << blade_pitch_cmd_3 << "," << blade_pitch_cmd_4 << ",";
			fout << K << "," << D << "," << I << "," << K_yaw << "," << D_yaw << "," << SERVO_NEUTRAL << ",";
			fout << setprecision(9) << time_gps << "," << lat << "," << lng << "," << alt_ellipsoid << ",";
			fout << msl_gps << "," << horz_accuracy << "," << vert_accuracy << "," << status_gps << "," << motor_cmd << endl;


			if(prescribed_input_index < 3000){
				prescribed_input_index++;}
			else{
				prescribed_input_index = 3001;}

			watcher[3] = time_now - timer[3]; // used to check loop frequency
			timer[3] = time_now;
		}

		if( (time_now-timer[4]) > duration[4])
		{
			//-------------------------------------------------------------------------------------------------------------Barometer Calc
			// barometer does full update at 25Hz, no need to do the full altitude calculation any faster than 50Hz
			Tc  = barometer.getTemperature(); // temperature from sensor (C), value is recorded at surface of PCB, (higher than ambient!)
			Tk  = Tc + 273.15; // convert to Kelvin (needed for barometric formula
			Tf  = Tc * (9.0/5.0) + 32; // convert to Fahrenheit (uncomment for sanity check but it is not used for any calcs)
			Pm  = barometer.getPressure(); // pressure is natively given in mbar
			Phg = Pm*.02953; // convert to inHg
			// barometric equation
			msl = hb + (Tb/Lb)*(pow((Phg/Pb),((-R*Lb)/(g0*M)))-1); // NOTE: using local Tk instead of standard temperature (Tb)

			if( dbmsg_global || dbmsg_local) {cout << "Barometer Check (at 50Hz), Temp (C,K,F): " << Tc << " " << Tk << " " << Tf << " "
				<< "Pressure (mbar, inHg): " << Pm << " " << Phg << " Altitude MSL (ft): " << msl << endl;}

			watcher[4] = time_now - timer[4]; // used to check loop frequency
			timer[4] = time_now;
		}


		if( (time_now-timer[5]) > duration[5]){
			// unused timer loop
			watcher[5] = time_now - timer[5]; // used to check loop frequency
			timer[5] = time_now;}

		if( (time_now-timer[6]) > duration[6]){

			step_counter_steer++;
			if(step_counter_steer>NUM_STEPS_STEER-1){
				step_counter_steer = 0;}

			watcher[6] = time_now - timer[6]; // used to check loop frequency
			timer[6] = time_now;}

		if( (time_now-timer[7]) > duration[7]){

			// this step counter is used for input sweeps (originally "step" inputs)
			step_counter_throttle++;
			if(step_counter_throttle>NUM_STEPS_THROTTLE-1){
				step_counter_throttle = 0;}

			watcher[7] = time_now - timer[7]; // used to check loop frequency
			timer[7] = time_now;}

		if( (time_now-timer[8]) > duration[8]){
			//Output Message
			//if(dbmsg_global || dbmsg_local){
			if(!dbmsg_global && !dbmsg_local){
				cout << endl;
				
				//cout << rc_array[5] > 1500 << endl;
				
				//cout << "Current filename: " << filename_str << endl;
				
				//cout << "adc_array[0]:  " << adc_array[0] << " adc_array[1]:  " << adc_array[1] << " adc_array[2]:  " << adc_array[2] << " adc_array[3]:  " << adc_array[3] << " adc_array[4]:  " << adc_array[4] << " adc_array[5]:  " << adc_array[5] << endl;

				//cout << "MPU9250: ";
				//cout << "Accelerometer: " << a_mpu[0] << " " << a_mpu[1] << " " << a_mpu[2];
				//cout << " Gyroscope: " << g_mpu[0] << " " << g_mpu[1] << " " << g_mpu[2];
				//cout << " Magnetometer: " << m_mpu[0] << " " << m_mpu[1] << " " << m_mpu[2] << endl;

				//cout << "GPS Status: " << status_gps_string << " GPS Time: " << time_gps << endl;
				//cout << "Lat: ";
				//cout << setprecision(9) <<  lat;
				//cout << " deg\tLng: ";
				//cout << setprecision(9) << lng;
				//cout << " deg\tAlt(msl): " << msl_gps << " ft\tAlt(ae): " << alt_ellipsoid << endl;

				//cout << "Euler Angles (Madgwick): " << "(dt = " << dt << ")" << endl;
				//cout << "MPU9250: Roll: " << roll_mpu_madgwick << " Pitch: " << pitch_mpu_madgwick << " Yaw: " << yaw_mpu_madgwick << endl;
//				cout << "Steering Input: " << pitch_right_cmd_msg << "\tThrottle Input: " << pitch_left_cmd_msg << endl;
				cout << "Pitch 1: " << blade_pitch_cmd_1 << "	Pitch 2: " << blade_pitch_cmd_2 << "	Pitch 3: " << blade_pitch_cmd_3 << "	Pitch 4: " << blade_pitch_cmd_4 << endl;
				//cout << "Time_start: " << time_start << " tsd: " << tsd << endl; 
				//cout << " Step counter steer: " << step_counter_steer << " Step counter throttle: " << step_counter_throttle << " time: " << t << " sine: " << 0.5*sin(t) << endl;
				multisine_counter++;}
				cout << "RC0: " << rc_array_scaled[0] << " RC1: " << rc_array_scaled[1] << " RC2: " << rc_array_scaled[2] << " RC3: " << rc_array_scaled[3] <<  " RC4: " << rc_array_scaled[4] << " RC5: " << rc_array_scaled[5] << endl;
				//cout << "roll_cmd: " << roll_cmd << "	pitch_cmd: " << pitch_cmd << "	yaw_cmd: " << yaw_cmd << endl;
				//cout << "roll_error: " << roll_error << "	pitch_error: " << pitch_error << "	yaw_error: " << yaw_error << endl;
				cout << "Motor Cmd: " << motor_cmd << endl;
				//cout << "Prescribed Input Index: " << prescribed_input_index << endl; 

			//cout << tse << " - " << time_start << " = " << time_now << endl;

//dbmsg_local = true;
			if(dbmsg_global || dbmsg_local){
				// Alternate debug message, just print out the most current duration for each loop occasionally
				cout << "Printing the most current expected vs. actual time since last execution for each loop" << endl;
				for(int i = 0 ; i < NUM_LOOPS ; i++){
					cout << "For " << frequency[i] << "(Hz) loop, expected: " << duration[i] << "(us), actual: " << watcher[i] << endl;}}
//dbmsg_local = false;

			watcher[8] = time_now - timer[8]; // used to check loop frequency
			timer[8] = time_now;
		}

	}

	// tidy up and close the file when we exit the inner while loop
	fout.close();
	}
	return 0;
}

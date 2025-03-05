#include <unistd.h>
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include <inttypes.h>
#include <tools.h>

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>

#include <motor/motor.h>
#include <motor/two_drive_controller.h>
#include <motor/four_drive_controller.h>

#include <sdv_msgs/Batteries.h>
#include <sdv_msgs/Battery.h>
#include <sdv_msgs/Buzzer.h>
#include <sdv_msgs/Encoder.h>
#include <sdv_msgs/Flexiforce.h>
#include <sdv_msgs/ImuRaw.h>
#include <sdv_msgs/LED.h>
#include <sdv_msgs/TwoMotors.h>
#include <sdv_msgs/FourMotors.h>
#include <sdv_msgs/PanelButton.h>
#include <sdv_msgs/SdvStatus.h>
#include <sdv_msgs/TagRfid.h>
#include <sdv_msgs/Ultrasound.h>
#include <sdv_msgs/MotorDriver.h>
#include <sdv_msgs/Drivers.h>


using namespace std;

/* Constants */
#define PORT "/dev/ttyACM0"
#define GYRO_SENSITIVITY_2000DPS 0.070
#define SENSORS_GRAVITY_EARTH 9.80665 // Earth's gravity in m/s^2
#define SENSORS_GRAVITY_STANDARD SENSORS_GRAVITY_EARTH
#define SENSORS_DPS_TO_RADS 0.01745329251994 // Degrees/s to rad/s multiplier
#define SENSORS_MICROTESLA_TO_TESLA 1.0e-6

// Typedefs

typedef struct
{
    double x;
    double y;
    double z;
} Gyro;
typedef struct
{
    double x;
    double y;
    double z;
} Accel;
typedef struct
{
    double x;
    double y;
    double z;
} MagField;
typedef struct
{
    double x;
    double y;
    double z;
    double w;
} QPose;

// Enums
enum BoardStatus
{
    OK,
    JUST_CONNECTED,
    DISCONNECTED,
    LOCKED
};

enum MotorModel
{
    NONE = 0,
    ESCON = 1,
    POLOLU = 2
};

////////////////////////////////////////////////////////////////////////////////
//
// Prototypes
//
////////////////////////////////////////////////////////////////////////////////
int openSerialPort(string portx);
void moveMotorsCallback(const geometry_msgs::Twist &cmd);
void ledCallback(sdv_msgs::LED msg);
void buzzerCallback(sdv_msgs::Buzzer msg);
void superBuzzerCallback(sdv_msgs::Buzzer msg);
void configSerialCommunication(void);
void readAndProcessCmd();
void sendConfigCommand(string cmd);
int CMD_IMUMessage(vector<string> args);
int CMD_SAMessage(vector<string> args);
int CMD_FlexiforceMessage(vector<string> args);
int CMD_PanelButtonMessage(vector<string> args);
int CMD_BatteryMessage(vector<string> args);
int CMD_DriverStatusMessage(vector<string> args);
int CMD_OdometryMessage(vector<string> args);
int CMD_UltrasoundMessage(vector<string> args);
int CMD_EmptyFunction(vector<string> args);

//*****************************************************************************
//
// Command line function callback type.
//
//*****************************************************************************
typedef int (*pfnCmdLine)(vector<string> args);

//*****************************************************************************
//
// Command struct
//
//*****************************************************************************
typedef struct
{
    // A pointer to a string containing the name of the command.
    const int intCmd;

    // A function pointer to the implementation of the command.
    pfnCmdLine pfnCmd;

} CMD_Struct;

//*****************************************************************************
//
// Board Commands Table
//
//*****************************************************************************
CMD_Struct CMD_Table[] = {
    {0, CMD_EmptyFunction},         // Motor: Command, publish nothing
    {1, CMD_IMUMessage},            // IMU: automatic message. Publish at 20Hz
    {2, CMD_OdometryMessage},       // Encoder: automatic message. Publish at 20Hz
    {3, CMD_EmptyFunction},         // SDV Status: automatic message
    {4, CMD_UltrasoundMessage},     // Ultrasound: automatic message. Publish at 20Hz
    {5, CMD_FlexiforceMessage},     // Flexiforce: automatic message. Publish at 2Hz
    {6, CMD_BatteryMessage},        // Battery: automatic message. Publish at 0.5Hz
    {7, CMD_EmptyFunction},         // Time stamp: Service
    {8, CMD_SAMessage},             // Still Alive and Aknowledge of Still Alive Message commands
    {9, CMD_EmptyFunction},         // Code for general message
    {10, CMD_PanelButtonMessage},   // Panel Button: automatic message
    {12, CMD_EmptyFunction},        // Code reset message
    {13, CMD_DriverStatusMessage},  // Drivers Data
};
int cmd_table_size = 13;

//*****************************************************************************
//
// Variables
//
//*****************************************************************************
serial::Serial ser;
vector<string> buffer_callback;
int baudrate = 921600;
string port;
string motor_drive_type;
int n_motors = 0;

BoardStatus board_status = DISCONNECTED;
double initial_stamp_PC;
double initial_stamp_tiva;
unsigned int microseconds = 100000;
double r = 0.075;
double B = 0.44010;
double N = 3.2 * 4;

// Motors
TwoDriveController two_drive_controller;
FourDriveController four_drive_controller;

// Still Alive Messages
double last_sa_msg_time_stamp = -1.0;

// IMU: sensor fusion calculations
double PI = 3.14159265358979323846;
double GyroMeasError = PI * (60.0 / 180.0);      // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
double beta = sqrt(3.0 / 4.0) * GyroMeasError;   // compute beta
double GyroMeasDrift = PI * (1.0 / 180.0);       // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
double zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
double q[4] = {1.0f, 0.0f, 0.0f, 0.0f};          // vector to hold quaternion
double deltat = 1.0 / 10.0;                      // integration interval for both filter schemes

// Publishers
ros::Publisher encoder_pub;
ros::Publisher sdv_status_pub;
ros::Publisher ultrasound_pub;
ros::Publisher flexiforce_pub;
ros::Publisher batteries_pub;
ros::Publisher tag_rfid_pub;
ros::Publisher imu_pub;
ros::Publisher imu_raw_pub;
ros::Publisher mag_pub;
ros::Publisher panel_button_pub;
ros::Publisher motor_status_pub;

//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/**
 * Callback function used by this node in "/mobile_base/commands/velocity" topic.
 * This function converts Twist messages into Tiva's command and puts it in buffer.
 *
 * @param cmd Twist message with linear and angular speeds 
 */
void moveMotorsCallback(const geometry_msgs::Twist &cmd)
{
    // Check board status
    if (board_status == DISCONNECTED || board_status == LOCKED)
    {
        ROS_INFO("moveMotorsCallback: board is not ok");
        return;
    }

    // Read values from incoming message
    geometry_msgs::Vector3 vel_linear;
    geometry_msgs::Vector3 vel_angular;
    vel_linear = cmd.linear;
    vel_angular = cmd.angular;
    string msgss;

    // Motor drive type: diferential
    if(motor_drive_type == "diferential")
    {
        msgss = two_drive_controller.getCommandString(vel_linear, vel_angular);
        two_drive_controller.publishMotorRefSpeeds();
    }

    // Motor drive type: mecanum
    if(motor_drive_type == "mecanum")
    {
        msgss = four_drive_controller.getCommandString(vel_linear, vel_angular);
        four_drive_controller.publishMotorRefSpeeds();
    }

    if (buffer_callback.size() <= 5)
    {
        buffer_callback.insert(buffer_callback.begin(), msgss);
        //ROS_INFO("moveMotorsCallback: Sending motor command: wL = %f, wR = %f", wL, wR);
        //ROS_INFO_STREAM("Motor CMD: " << msgss);
    }
    else
    {
        ROS_INFO("moveMotorsCallback: OverStack cmd: %i", int(buffer_callback.size()));
    }

}

void ledCallback(sdv_msgs::LED msg)
{
    // Generaring message string
    string msgss;
    msgss = "l " + std::to_string(int(msg.red)) +
            " " + std::to_string(int(msg.green)) +
            " " + std::to_string(int(msg.blue)) +
            "\r";

    if (buffer_callback.size() <= 5)
    {
        buffer_callback.insert(buffer_callback.begin(), msgss);
    }
    else
    {
        ROS_INFO("led_updater: OverStack cmd: %i", int(buffer_callback.size()));
    }
}

void buzzerCallback(sdv_msgs::Buzzer msg)
{
    // Generaring message string
    string msgss;
    msgss = "n " + std::to_string(int(msg.time_on)) +
            " " + std::to_string(int(msg.time_off)) +
            " " + std::to_string(int(msg.cicles)) +
            "\r";
    if (buffer_callback.size() <= 5)
    {
        buffer_callback.insert(buffer_callback.begin(), msgss);
    }
    else
    {
        ROS_INFO("buzzerCallback: OverStack cmd: %i", int(buffer_callback.size()));
    }
}

void superBuzzerCallback(sdv_msgs::Buzzer msg)
{
    // Generaring message string
    string msgss;
    msgss = "sn " + std::to_string(int(msg.time_on)) +
            " " + std::to_string(int(msg.time_off)) +
            " " + std::to_string(int(msg.cicles)) +
            "\r";
    if (buffer_callback.size() <= 5)
    {
        buffer_callback.insert(buffer_callback.begin(), msgss);
    }
    else
    {
        ROS_INFO("superBuzzerCallback: OverStack cmd: %i", int(buffer_callback.size()));
    }
}

/**
 * This function configures the connected board: stops board messages and send 
 * reset command. Then, configure automatic messages and print options.
 */
void configSerialCommunication(void)
{
    // Send commands to stop sending messages
    ROS_INFO("configSerialCommunication: stoping prev messages");
    ser.write("if 0\r");
    ser.flushOutput();
    ser.write("sa 0\r");
    ser.flushOutput();
    ser.readline(ser.available());
    ser.flushInput();
    usleep(microseconds * 5);

    // Reset Board
    ROS_INFO("configSerialCommunication: reseting");
    ser.write("rt\r");
    ser.flushOutput();

    // Reading Timestamp in a loop
    ROS_INFO("configSerialCommunication: reading timestamp");
    bool reseted = false;
    while (!reseted and ros::ok())
    {
        std::string line_start = "";
        if (ser.available())
        {
            line_start = ser.read(1);
            //ROS_INFO_STREAM("line_strat: " << line_start);

            if (line_start == "#") // Wait for start message and read some values
            {
                // Reading stamp time from Serial input
                std::string data_stamp = ser.readline();
                ser.flushInput();

                // Stores data in a vector of big integers
                std::vector<uint64_t> vd;
                uint64_t d = 0;
                std::stringstream ss(data_stamp);
                while (ss >> d)
                    vd.push_back(d);

                // Apply Bit shifting to get complete number
                uint64_t t_board = (vd[0] << 32) + vd[1];

                // Time stamp of PC and Board
                double initial_stamp_PC = ros::Time::now().toSec();
                double initial_stamp_tiva = (double)(t_board) / 1000000;

                // Print obtained Time Stamp Data
                ROS_INFO_STREAM("data_stamp: " << tools::cleanString(data_stamp));
                ROS_INFO("initial_stamp_PC: %f", initial_stamp_PC);
                ROS_INFO("initial_stamp_tiva: %f", initial_stamp_tiva);

                // Set 'board_is_ok' flag, allowing main loop to run
                board_status = JUST_CONNECTED;
                reseted = true;
            }
        }
        else
        {
            ROS_INFO_STREAM("Waiting for reset message: " << line_start);
            exit(2);
        }
        //ros::spinOnce();
        usleep(microseconds);
    }

    ROS_INFO("configSerialCommunication: configuring board");

    // Config Board to don't send received message confirmation
    sendConfigCommand("cf 0");

    // Config board to auto stop if not receives motor commands
    sendConfigCommand("mt 1");

    // Config board to auto turn-off RGB-LED if not receives new LED commands
    sendConfigCommand("lt 1");

    // Config board to disable Sensor Data Messages if this node don't send akn msg
    sendConfigCommand("dt 1");

    // Config board to send "Still Alive Messages" to PC
    sendConfigCommand("sa 1");

    // Config board to send IMU data
    sendConfigCommand("if 1");

    // Config board to send Flexiforce data
    sendConfigCommand("ff 1");

    // Config board to send PanelButton data
    sendConfigCommand("pf 1");

    // Config board to send Batteries data
    sendConfigCommand("bf 1");

    // Config board to send Driver Status data
    sendConfigCommand("df 1");

    // Config board to send Odometry data
    sendConfigCommand("of 1");

    // Config board to send Ultrasound data
    sendConfigCommand("uf 1");

    ROS_INFO("configSerialCommunication: finished");
}

void sendConfigCommand(string cmd)
{
    cmd = cmd + "\r";
    ser.write(cmd);
    ser.flushOutput();
    usleep(microseconds);
}

/**
 * Try to open received serial port or other predefined port. A port string can
 * be "/dev/ttyACM0" in Tiva Boards.
 * 
 * @param portx String with the name of port, e.g. "/dev/ttyACM0"
 * @return Integer, -1 if can't open received or other ports
 */
int openSerialPort(string portx)
{
    //string msg = "Trying to start serial communication in %s port at %i bauds";
    //msg = snprintf("Trying to start serial communication in %s port at %i bauds", portx, baudrate);
    ROS_INFO("Trying to start serial communication in %s port at %i bauds", portx.c_str(), baudrate);
    try
    {
        ser.setPort(portx);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port: " << ser.getPort());
    }
    return 1;
}

/**
 * Read the serial stream and obtains a command. Then, process the arrived command.
 * Only find and process a command. If board sends many commands at the same time,
 * serial buffer can be overflowed.
 * 
 * @exception SerialException: Exit and close application
 * @exception invalid_argument: ignores the line
 **/
void readAndProcessCmd()
{
    try
    {
        // Search for available data in serial port
        if (board_status == OK or board_status == JUST_CONNECTED and ser.available())
        {
            /*
            Store serial input in a String. Read chars until find a *\n* character.
            If board is locked, the *\n* char never will arrive: using timestamps allow to 
            control if reading a char is taking much time.
            */
            std::stringstream input_line;
            string last_char = "";
            string input_msg;
            bool new_line_char = false;
            bool return_line_char = false;
            bool complete_cmd = false;
            double last_read = ros::Time::now().toSec();
            double delta_time = 0.0;
            bool read_again = true;
            while (read_again and !complete_cmd)
            {
                // Read 1 char and check if is a *\n* or a *\r*
                last_char = ser.read(1);
                if (last_char.length() > 0)
                {
                    input_line << last_char;
                    last_read = ros::Time::now().toSec();
                    if (last_char == "\n")
                    {
                        new_line_char = true;
                    }
                    if (last_char == "\r")
                    {
                        return_line_char = true;
                    }
                    if (new_line_char and return_line_char)
                    {
                        complete_cmd = true;
                    }
                }

                // Calculate the read task duration. If is too much, stop reading from serial
                delta_time = ros::Time::now().toSec() - last_read;
                if (delta_time > 0.05)
                {
                    //cout << "Too much time to read a char. Stop reading chars\n";
                    read_again = false;
                }

                ///////////////////////////////////
                /*
                int val = -1;
                char cc = '_';
                if(last_char.length() > 0)
                {
                    val = (int)last_char.at(0);
                    if( val != 10)
                    {
                        cc = last_char.at(0);
                    }
                    if( val == 10)
                    {
                        cc = '*';
                    }
                    if( val == 13)
                    {
                        cc = '~';
                    }
                }
                cout 
                << "Reading a char from serial: last_char(val) = [" << val 
                << "], last_char(char) = [" << cc
                << "], lenght = " << last_char.length()
                << ", delta = " << delta_time
                << ", end_line = " << new_line_char
                << ", return_line = " << return_line_char
                << ", complete_cmd = " << complete_cmd
                << "\n";
                */
                ///////////////////////////////////
            }
            input_msg = tools::cleanString(input_line.str());

            // Process String content
            if (input_msg.length() > 2 and input_msg.at(0) == '<')
            {
                // Get vector of strings
                vector<string> v_args = tools::getArgs(input_msg);

                // Get command code
                int cmd_code = -1;
                string cmd_code_string = v_args[0].substr(1, v_args[0].length() - 1);
                try
                {
                    cmd_code = stoi(cmd_code_string);
                }
                catch (std::invalid_argument &e)
                {
                    ROS_ERROR_STREAM(
                        "Invalid argument: CMD code = "
                        << v_args[0]
                        << ", string = "
                        << cmd_code_string);
                    cmd_code = -1;
                }
                //cout << "Received CMD: Code = " << int(cmd_code) << "\n";

                // Search in CMD Table and send to pointed function
                for (int i = 0; i < cmd_table_size; i++)
                {
                    if (CMD_Table[i].intCmd == cmd_code)
                    {
                        CMD_Table[i].pfnCmd(v_args);
                    }
                }
            }

            // Process reset command
            int reset_char_pos = input_msg.find("#");
            if (reset_char_pos < input_msg.length() - 1)
            {
                cout << "Received reset command\n";
                board_status = DISCONNECTED;
                configSerialCommunication();
            }

            //cout << "Received CMD = " << input_msg << "\n";

        } // End of "if(ser.available())"
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR("%s", e.what());
        exit(2);
    }
}

/**
 * Publish a message in IMU topic when a imu messages arrives from board.
 * 
 * @param args String with the IMU values
 * @return Integer, -1 if messsage is wrong
 */
int CMD_IMUMessage(vector<string> args)
{
    int channels = 3 + 3 + 3 + 4;
    //vector<string> vdata = get_args(args);
    if (args.size() != channels + 1)
    {
        ROS_INFO_STREAM("IMU: Error with received data size " << args.size());
        return -1;
    }

    //ROS_INFO("IMU: Correct data");
    double data[channels];
    for (int i = 0; i < channels; i++)
    {
        data[i] = stod(args.at(i + 1));
        //ROS_INFO("IMU[%i] = %f", i, data[i-1]);
    }

    // Linear acceleration: data arrives in g units and needs to be in m/s^2
    Accel accel;
    accel.x = data[0] * SENSORS_GRAVITY_EARTH;
    accel.y = data[1] * SENSORS_GRAVITY_EARTH;
    accel.z = data[2] * SENSORS_GRAVITY_EARTH;

    // Angular velocity: data arrives in DPS and needs to be in rad/s
    Gyro gyro;
    gyro.x = data[3] * SENSORS_DPS_TO_RADS;
    gyro.y = data[4] * SENSORS_DPS_TO_RADS;
    gyro.z = data[5] * SENSORS_DPS_TO_RADS;

    // Magnetic field: Data arrives in uT: needs to be in T (Tesla)
    // Frame is in NED convention (x = North, y = East, z = Down)
    // Transforming to ENU (x = East, y = North, z = Up)
    MagField mag_field;
    mag_field.x = data[7] * SENSORS_MICROTESLA_TO_TESLA;
    mag_field.y = data[6] * SENSORS_MICROTESLA_TO_TESLA;
    mag_field.z = data[8] * -SENSORS_MICROTESLA_TO_TESLA;

    // Orientation in quaternion format
    QPose qpose;
    qpose.x = data[9];
    qpose.y = data[10];
    qpose.z = data[11];
    qpose.w = data[12];

    // IMU Message
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    imu_msg.angular_velocity_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    imu_msg.linear_acceleration_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};

    imu_msg.linear_acceleration.x = accel.x;
    imu_msg.linear_acceleration.y = accel.y;
    imu_msg.linear_acceleration.z = accel.z;

    imu_msg.angular_velocity.x = gyro.x;
    imu_msg.angular_velocity.y = gyro.y;
    imu_msg.angular_velocity.z = gyro.z;

    imu_msg.orientation.x = qpose.x;
    imu_msg.orientation.y = qpose.y;
    imu_msg.orientation.z = qpose.z;
    imu_msg.orientation.w = qpose.w;

    // Magnetic Field Message
    sensor_msgs::MagneticField mg_msg;
    mg_msg.header.stamp = ros::Time::now();
    mg_msg.header.frame_id = "imu_link";
    mg_msg.magnetic_field.x = mag_field.x;
    mg_msg.magnetic_field.y = mag_field.y;
    mg_msg.magnetic_field.z = mag_field.z;
    mg_msg.magnetic_field_covariance[0] = 0;

    // IMU Raw Message
    sdv_msgs::ImuRaw imu_raw_msg;
    imu_raw_msg.header.stamp = ros::Time::now();
    imu_raw_msg.linear_acceleration.x = accel.x;
    imu_raw_msg.linear_acceleration.y = accel.y;
    imu_raw_msg.linear_acceleration.z = accel.z;

    imu_raw_msg.angular_velocity.x = gyro.x;
    imu_raw_msg.angular_velocity.y = gyro.y;
    imu_raw_msg.angular_velocity.z = gyro.z;

    imu_raw_msg.magnetic_field.x = mag_field.x;
    imu_raw_msg.magnetic_field.y = mag_field.y;
    imu_raw_msg.magnetic_field.z = mag_field.z;

    // Publish messages
    imu_pub.publish(imu_msg);
    mag_pub.publish(mg_msg);
    imu_raw_pub.publish(imu_raw_msg);

    return 1;
}

/**
 * Process a message that contains a SA message with timestamp data from the board.
 * 
 * @param args String with the SA message values
 * @return Integer, -1 if messsage is wrong
 */
int CMD_SAMessage(vector<string> args)
{
    // Get Time Stamp of arrived message
    last_sa_msg_time_stamp = ros::Time::now().toSec();

    // Change board status
    if (board_status == JUST_CONNECTED)
    {
        board_status = OK;
    }

    // Push a message in buffer
    string msg = "sk\r";
    buffer_callback.insert(buffer_callback.begin(), msg);

    return 1;
}

/**
 * Process a message that contains a Flexiforce data from the board.
 * 
 * @param args String with the Flexiforce message values
 * @return Integer, -1 if messsage is wrong
 */
int CMD_FlexiforceMessage(vector<string> args)
{
    // String to vector
    //vector<string> vdata = get_args(args);
    int channels = 4;
    if (args.size() != channels + 1) // 5 diferent values
    {
        ROS_INFO_STREAM("Flexiforce: Error with received data size " << args.size());
        return -1;
    }

    // Get data from string message
    double data[channels];
    for (int i = 0; i < channels; i++)
    {
        data[i] = stod(args.at(i + 1));
    }

    // Init ROS message
    sdv_msgs::Flexiforce flx_msg;
    flx_msg.header.stamp = ros::Time::now();

    flx_msg.front_left = data[0];
    flx_msg.front_right = data[1];
    flx_msg.back_left = data[2];
    flx_msg.back_right = data[3];

    // Publish
    flexiforce_pub.publish(flx_msg);

    return 1;
}

/**
 * Process a message that contains a PanelButton data from the board.
 * 
 * @param args String with the PanelButton message values
 * @return Integer, -1 if messsage is wrong
 */
int CMD_PanelButtonMessage(vector<string> args)
{
    // Check size of vector
    int channels = 1;
    if (args.size() != channels + 1)
    {
        ROS_INFO_STREAM("CMD_PanelButtonMessage: Error with received data size " << args.size());
        return -1;
    }

    // Get data from string message
    uint8_t data = stod(args.at(1));

    // Init ROS message
    sdv_msgs::PanelButton button_msg;
    button_msg.header.stamp = ros::Time::now();
    button_msg.status = data;

    // Publish
    panel_button_pub.publish(button_msg);
    return 1;
}

/**
 * Process a message that contains the Batteries data.
 * 
 * @param args String with the Batteries message values
 * @return Integer, -1 if messsage is wrong
 */
int CMD_BatteryMessage(vector<string> args)
{
    // Get first two arguments
    int n_batteries = 0;
    int n_cells = 0;
    if (args.size() > 3)
    {
        n_batteries = stod(args.at(1));
        n_cells = stod(args.at(2));
    }

    // Check size of vector
    int channels = (n_batteries * n_cells) + 2;
    if (args.size() != channels + 1)
    {
        ROS_INFO_STREAM("CMD_BatteryMessage: Error with received data size " << args.size());
        return -1;
    }

    // Get data from string message
    double data[channels];
    for (int i = 0; i < channels; i++)
    {
        data[i] = stod(args.at(i + 1));
        //cout << "  " << data[i] << "\n\r";
    }

    // Set ROS message
    sdv_msgs::Batteries batteries_msg;
    batteries_msg.header.stamp = ros::Time::now();
    int counter = 0;
    for(int i = 0; i < n_batteries; i++)
    {
        sdv_msgs::Battery battery;
        string name = "Battery " + to_string(i + 1);
        battery.name = name;
        for(int j = 0; j < n_cells; j++)
        {
            battery.cell_voltages.push_back(data[counter + 2]);
            counter++;
        }
        batteries_msg.batteries.push_back(battery);
    }

    // Publish
    batteries_pub.publish(batteries_msg);
    
    return 1;
}

/**
 * Process a message that contains the Driver Status data.
 * 
 * @param args String with the Driver Status message values
 * @return Integer, -1 if messsage is wrong
 */
int CMD_DriverStatusMessage(vector<string> args)
{
    // Get first two arguments
    int motor_model = MotorModel::NONE;
    int n_drivers = 0;
    if (args.size() > 3)
    {
        motor_model = stod(args.at(1));
        n_drivers = stod(args.at(2));
    }

    // Check size of vector.
    int channels = 0;
    int driver_fields = 0;
    string motor_names[4];
    switch (motor_model)
    {
    case MotorModel::POLOLU:
        driver_fields = 3;
        motor_names[0] = "back_left";
        motor_names[1] = "back_right";
        motor_names[2] = "front_left";
        motor_names[3] = "front_right";
        break;
    case MotorModel::ESCON:
        driver_fields = 1;
        motor_names[0] = "left";
        motor_names[1] = "right";
        break;
    default:
        break;
    }
    channels = (n_drivers * driver_fields) + 2;
    
    if (args.size() != channels + 1)
    {
        ROS_INFO_STREAM(
            "CMD_DriverStatusMessage: Error with received data size. current = " 
            << channels
            << ", required = "
            << args.size() 
            );
        return -1;
    }

    // Get data from string message
    double data[channels];
    for (int i = 0; i < channels; i++)
    {
        data[i] = stod(args.at(i + 1));
    }

    // Set ROS message
    sdv_msgs::Drivers motors_msg;
    motors_msg.header.stamp = ros::Time::now();
    int counter = 0;
    for(int i = 0; i < n_drivers; i++)
    {
        // Set current value
        sdv_msgs::MotorDriver m_msg;
        m_msg.name = motor_names[i];
        m_msg.driver_status = sdv_msgs::MotorDriver::UNKNOWN;
        m_msg.current = data[counter + 2];

        // Set driver status
        if(motor_model == MotorModel::POLOLU)
        {
            bool half_bridge_a = data[counter + 3];
            bool half_bridge_b = data[counter + 4];
            if(!half_bridge_a or !half_bridge_b)
                m_msg.driver_status = sdv_msgs::MotorDriver::DAMAGED;
            else
                m_msg.driver_status = sdv_msgs::MotorDriver::CORRECT;
            counter += 2;
        }
        counter++;

        // Append driver data to Drivers message
        motors_msg.drivers.push_back(m_msg);
    }

    // Publish
    motor_status_pub.publish(motors_msg);
    
    return 1;
}

/**
 * Process a message that contains the Odometry data.
 * 
 * @param args String with the Odometry message values
 * @return Integer, -1 if messsage is wrong
 */
int CMD_OdometryMessage(vector<string> args)
{
    // Check size of vector
    int channels = n_motors;
    if (args.size() != channels + 1)
    {
        ROS_INFO_STREAM("CMD_OdometryMessage: Error with received data size " << args.size());
        return -1;
    }

    // Get data from string message
    double data[channels];
    for (int i = 0; i < channels; i++)
    {
        data[i] = stod(args.at(i + 1));
    }

    // Motor drive type: diferential
    if(motor_drive_type == "diferential")
    {
        two_drive_controller.setActualSpeeds(data);
        two_drive_controller.publishMotorActualSpeeds();
    }

    // Motor drive type: mecanum
    if(motor_drive_type == "mecanum")
    {
        four_drive_controller.setActualSpeeds(data);
        four_drive_controller.publishMotorActualSpeeds();
    }

    return 1;
}

/**
 * Process a message that contains the Ultrasound data.
 * 
 * @param args String with the Odometry message values
 * @return Integer, -1 if messsage is wrong
 */
int CMD_UltrasoundMessage(vector<string> args)
{
    // Check size of vector
    int channels = 6;
    if (args.size() != channels + 1)
    {
        ROS_INFO_STREAM("CMD_UltrasoundMessage: Error with received data size " << args.size());
        return -1;
    }

    // Get data from string message
    double data[channels];
    for (int i = 0; i < channels; i++) 
        data[i] = stod(args.at(i + 1));
    
    // Set ROS message
    sdv_msgs::Ultrasound ultrasound_msg;
    ultrasound_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < 6; i++)
        ultrasound_msg.sensors.push_back(data[i]);
    
    // Publish
    ultrasound_pub.publish(ultrasound_msg);

    return 1;
}

/**
 * Empty functión, used to create and test *pfnCmdLine* functions
 * 
 * @param args String with Tiva data
 * @return Integer, -1 if messsage is wrong
 */
int CMD_EmptyFunction(vector<string> args)
{
    ROS_INFO("CMD_EmptyFunction: TO-DO.");
    return 1;
}

/**
 * main
 */
int main(int argc, char **argv)
{
    // Configuring node
    ros::init(argc, argv, "sdv_serial_node");
    ros::NodeHandle nh("~");

    // Reading ROS parameter: port
    if (ros::param::has("/sdv/board_port"))
    {
        nh.getParam("/sdv/board_port", port);
        ROS_INFO_STREAM("Got port value from ROS parameter: " << port);
    }

    // Reading ROS parameter: baudrate
    if (ros::param::has("/sdv/board_baudrate"))
    {
        nh.getParam("/sdv/board_baudrate", baudrate);
        ROS_INFO_STREAM("Got baudrate value from ROS parameter: " << baudrate);
    }

    // Reading ROS parameter: port
    if (ros::param::has("/sdv/board_port"))
    {
        nh.getParam("/sdv/board_port", port);
        ROS_INFO_STREAM("Got port value from ROS parameter: " << port);
    }

    // Reading ROS parameter: wheel_separation
    if (ros::param::has("/sdv/wheel_separation"))
    {
        double separation;
        nh.getParam("/sdv/wheel_separation", separation);
        if (separation != 0.0)
        {
            ROS_INFO_STREAM("Got '/sdv/wheel_separation' from ROS parameters: " << separation);
            B = separation;
            two_drive_controller.setWheelSeparation(separation);
            four_drive_controller.setWheelSeparation(separation);
        }
    }

    // Reading ROS parameter: wheel_axis_separation
    if (ros::param::has("/sdv/wheel_separation"))
    {
        double separation;
        nh.getParam("/sdv/wheel_axis_separation", separation);
        if (separation != 0.0)
        {
            ROS_INFO_STREAM("Got '/sdv/wheel_axis_separation' from ROS parameters: " << separation);
            B = separation;
            two_drive_controller.setAxisWheelSeparation(separation);
            four_drive_controller.setAxisWheelSeparation(separation);
        }
    }

    // Reading ROS parameter: motor drive type
    if (ros::param::has("/sdv/motor_drive_type"))
    {
        nh.getParam("/sdv/motor_drive_type", motor_drive_type);
        if(motor_drive_type == "diferential")
            n_motors = 2;
        if(motor_drive_type == "mecanum")
            n_motors = 4;
        ROS_INFO_STREAM("Got motor drive type value from ROS parameter: " << motor_drive_type);
    }

    // Subscribing to topics
    ros::Subscriber move_motors_sub = nh.subscribe("/mobile_base/commands/velocity", 20, moveMotorsCallback);
    ros::Subscriber led_sub = nh.subscribe("/led", 20, ledCallback);
    ros::Subscriber buzzer_sub = nh.subscribe("/buzzer", 20, buzzerCallback);
    ros::Subscriber super_buzzer_sub = nh.subscribe("/super_buzzer", 20, superBuzzerCallback);

    // Publishers
    encoder_pub = nh.advertise<sdv_msgs::Encoder>("/encoder", 20);
    sdv_status_pub = nh.advertise<sdv_msgs::SdvStatus>("/sdv_status", 20);
    ultrasound_pub = nh.advertise<sdv_msgs::Ultrasound>("/ultrasound", 20);
    flexiforce_pub = nh.advertise<sdv_msgs::Flexiforce>("/flexiforce", 20);
    batteries_pub = nh.advertise<sdv_msgs::Batteries>("/batteries", 20);
    tag_rfid_pub = nh.advertise<sdv_msgs::TagRfid>("/read_tag_rfid", 20);
    imu_raw_pub = nh.advertise<sdv_msgs::ImuRaw>("/imu_raw", 20);
    panel_button_pub = nh.advertise<sdv_msgs::PanelButton>("/panel_button", 20);
    motor_status_pub = nh.advertise<sdv_msgs::Drivers>("/motors/drivers", 20);
    mag_pub = nh.advertise<sensor_msgs::MagneticField>("/imu/mag", 20);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 20);

    two_drive_controller.setNodeHandle(&nh);
    four_drive_controller.setNodeHandle(&nh);

    // Opening serial port
    if (openSerialPort(port) == -1)
    {
        ROS_ERROR("Error opening port. Exit from sdv_serial_node.");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized --> " << ser.getPort());
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open port: " << ser.getPort());
        return -1;
    }

    // Configuring ROS loop rate
    ros::Rate loop_rate(500);

    // Some variables...
    int i_cmd = 0;
    string line_start;

    // ROS Loop
    while (ros::ok())
    {
        // Start serial communication
        if (board_status == DISCONNECTED)
        {
            configSerialCommunication();
            ROS_INFO("sdv_serial_node entering in ROS loop");
        }

        readAndProcessCmd();

        // If buffer is not empty, write messages in serial port
        if (buffer_callback.size() > 0 and board_status == OK)
        {
            // Get element from vector
            string msg = buffer_callback.back();

            // Check that is readable
            if (msg.find("rt") != string::npos)
            {
                ROS_INFO("Not complete msg...");
            }
            else // Write message in serial output
            {
                ser.write(msg);
                ser.flushOutput();
                buffer_callback.pop_back();
                //ROS_INFO_STREAM("Writing serial port --> " << msg);
            }
        }

        // Check last SA Stamp Time and change board status
        double now = ros::Time::now().toSec();
        //double d = ros::Time::now().toSec() - last_sa_msg_time_stamp;
        double d = now - last_sa_msg_time_stamp;
        if (board_status == OK)
        {
            if (d > 3.0)
            {
                ROS_ERROR("Board is not sending Still Alive Messages. Changing status to LOCKED");
                board_status = LOCKED;
            }
        }
        //cout << "Board Status = " << board_status << " , now = " << now << ", last = " << last_sa_msg_time_stamp << ", delta = " << d << "\n";

        // If board is locked due to a reset, try to reconfigure it
        if (board_status == LOCKED)
        {
            ROS_INFO("Board in locked status. Trying to reset.");
            configSerialCommunication();
            ROS_INFO("sdv_serial_node entering in ROS loop");
        }

        // Spin
        ros::spinOnce();
        loop_rate.sleep();

    } // End of ROS Loop
} // End of main function
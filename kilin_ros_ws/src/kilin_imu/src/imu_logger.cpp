/////////////////////////////////////////////////////////////////////////////
//
// imu_logger.cpp
//
// derived from CX5_GX5_CV5_15_25_Example.cpp
//
// C++ Example set-up program for the CX5-15, CX5-25, GX5-15, GX5-25, CV5-15, and CV5-25.
//
// This example shows a typical setup for the CX5-15 sensor in a wheeled-vehicle application using using C++.
// It is not an exhaustive example of all CX5-15 settings.
// If your specific setup needs are not met by this example, please consult
// the MSCL-embedded API documentation for the proper commands.
//
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER
//! FOR THEM TO SAVE TIME. AS A RESULT, HBK MICROSTRAIN SHALL NOT BE HELD
//! LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY
//! CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS
//! OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////

#include <mip/mip_all.hpp>
#include <array>
#include "mip_utils/mip_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>

using namespace mip;


#define DO_PRINT true // set to false to stop printing to console
#define DO_LOG true // set to false to stop logging
#define PRINTING_PERIOD 1000 // in ms
#define LOGGING_PERIOD 10 // in ms
#define SENSOR_SAMPLE_RATE 100 // in Hz

////////////////////////////////////////////////////////////////////////////////
// Global Variables
////////////////////////////////////////////////////////////////////////////////


//Sensor-to-vehicle frame rotation (Euler Angles)
float sensor_to_vehicle_rotation_euler[3] = {0.0, 0.0, 0.0};

//Device data stores
data_sensor::GpsTimestamp sensor_gps_time;
data_sensor::ScaledAccel  sensor_accel;
data_sensor::ScaledGyro   sensor_gyro;

data_filter::Timestamp    filter_gps_time;
data_filter::Status       filter_status;
data_filter::EulerAngles  filter_euler_angles;
data_filter::CompAngularRate  filter_comp_angular_rate;
data_filter::CompAccel    filter_comp_accel;

bool filter_state_running = false;


////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0);

void exit_gracefully(const char *message);
bool should_exit();

static bool mkdir_p(const char* path, mode_t mode);


////////////////////////////////////////////////////////////////////////////////
// Main Function
////////////////////////////////////////////////////////////////////////////////


int main(int argc, const char* argv[])
{
    // for ros2 compatability
    // Strip ROS arguments
    auto filtered_args = rclcpp::remove_ros_arguments(argc, argv);
    int filtered_argc = static_cast<int>(filtered_args.size());

    if (filtered_argc != 3) {
        std::cerr << "Usage: MIP_IMU_LOGGER <port> <baud>" << std::endl;
        throw std::underflow_error("Usage error");
    }

    // Convert std::vector<std::string> → std::vector<const char*>
    std::vector<const char*> raw_args;
    for (const auto& arg : filtered_args) {
        raw_args.push_back(arg.c_str());
    }

    // code starts

    std::unique_ptr<ExampleUtils> utils = handleCommonArgs(filtered_argc, raw_args.data());
    std::unique_ptr<mip::DeviceInterface>& device = utils->device;

    printf("Connecting to and configuring sensor.\n");

    //
    //Ping the device (note: this is good to do to make sure the device is present)
    //

    if(commands_base::ping(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not ping the device!");


    //
    //Idle the device (note: this is good to do during setup)
    //

    if(commands_base::setIdle(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set the device to idle!");


    //
    //Load the device default settings (so the device is in a known state)
    //

    if(commands_3dm::defaultDeviceSettings(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");


    float gyro_bias[3] = {0, 0, 0};

    const uint32_t sampling_time = 2000; // The default is 15000 ms and longer sample times are recommended but shortened for convenience
    const int32_t old_mip_sdk_timeout = device->baseReplyTimeout();
    printf("Capturing gyro bias. This will take %d seconds \n", sampling_time/1000);
    device->setBaseReplyTimeout(sampling_time * 2);

    if(commands_3dm::captureGyroBias(*device, sampling_time, gyro_bias) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not capture gyro bias!");

    if(commands_3dm::saveGyroBias(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not save gyro bias!");

    const uint8_t fn_selector = 1;
    const uint8_t device_selector = 3;
    const uint8_t enable_flag = 1;
    if(commands_3dm::writeDatastreamControl(*device, device_selector, enable_flag) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not enable device data stream!");

    // Reset the timeout
    device->setBaseReplyTimeout(old_mip_sdk_timeout);

    printf("Gyro bias captured with sampling time: %d, and gyro bias captured as: %f %f %f.\n", sampling_time, gyro_bias[0], gyro_bias[1], gyro_bias[2]);


    //
    //Setup Sensor data format to 100 Hz
    //

    uint16_t sensor_base_rate;

    //Note: Querying the device base rate is only one way to calculate the descriptor decimation.
    //We could have also set it directly with information from the datasheet.

    if(commands_3dm::imuGetBaseRate(*device, &sensor_base_rate) != CmdResult::ACK_OK)
         exit_gracefully("ERROR: Could not get sensor base rate format!");

    const uint16_t sensor_sample_rate = SENSOR_SAMPLE_RATE; // Hz
    const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

    std::array<DescriptorRate, 3> sensor_descriptors = {{
        { data_sensor::DATA_TIME_STAMP_GPS, sensor_decimation },
        { data_sensor::DATA_ACCEL_SCALED,   sensor_decimation },
        { data_sensor::DATA_GYRO_SCALED,    sensor_decimation },
    }};

    if(commands_3dm::writeImuMessageFormat(*device, sensor_descriptors.size(), sensor_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set sensor message format!");


    //
    //Setup FILTER data format
    //

    uint16_t filter_base_rate;

    if(commands_3dm::filterGetBaseRate(*device, &filter_base_rate) != CmdResult::ACK_OK)
         exit_gracefully("ERROR: Could not get filter base rate format!");

    const uint16_t filter_sample_rate = 100; // Hz
    const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

    std::array<DescriptorRate, 5> filter_descriptors = {{
        { data_filter::DATA_FILTER_TIMESTAMP, filter_decimation },
        { data_filter::DATA_FILTER_STATUS,    filter_decimation },
        { data_filter::DATA_ATT_EULER_ANGLES, filter_decimation },
        { data_filter::DATA_COMPENSATED_ANGULAR_RATE, filter_decimation },
        { data_filter::DATA_COMPENSATED_ACCELERATION, filter_decimation },
    }};

    if(commands_3dm::writeFilterMessageFormat(*device, filter_descriptors.size(), filter_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");


    //
    //Setup the sensor to vehicle rotation
    //

    if(commands_filter::writeSensorToVehicleRotationEuler(*device, sensor_to_vehicle_rotation_euler[0], sensor_to_vehicle_rotation_euler[1], sensor_to_vehicle_rotation_euler[2]) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set sensor-2-vehicle rotation!");

    //
    //Enable filter auto-initialization
    //

    if(commands_filter::writeAutoInitControl(*device, 1) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter autoinit control!");


    //
    //Reset the filter (note: this is good to do after filter setup is complete)
    //

    if(commands_filter::reset(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter!");


    //
    // Register data callbacks
    //
    // IMPORTANT:
    // DispatchHandler MUST outlive device->update().
    // If declared as local variables, they may be invalid later and cause crashes.
    //

    //Sensor Data
    static DispatchHandler sensor_data_handlers[3];

    device->registerExtractor(sensor_data_handlers[0], &sensor_gps_time);
    device->registerExtractor(sensor_data_handlers[1], &sensor_accel);
    device->registerExtractor(sensor_data_handlers[2], &sensor_gyro);
 
    //Filter Data
    static DispatchHandler filter_data_handlers[5];

    device->registerExtractor(filter_data_handlers[0], &filter_gps_time);
    device->registerExtractor(filter_data_handlers[1], &filter_status);
    device->registerExtractor(filter_data_handlers[2], &filter_euler_angles);
    device->registerExtractor(filter_data_handlers[3], &filter_comp_angular_rate);
    device->registerExtractor(filter_data_handlers[4], &filter_comp_accel);


    //
    //Resume the device
    //

    if(commands_base::resume(*device) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");


    //
    //Create new log file
    //

    // Get current time
    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    // Log directory: $HOME/kilin_ws/logs/imu_logs
    const char* home = getenv("HOME");
    if(!home){
        fprintf(stderr, "HOME is not set\n");
        return 1;
    }

    char log_dir[512];
    snprintf(log_dir, sizeof(log_dir), "%s/kilin_ws/logs/imu_logs", home);

    // Create directory if not exists
    if(!mkdir_p(log_dir, 0775)){
        fprintf(stderr, "Failed to create log directory: %s\n", log_dir);
        return 1;
    }

    // Format timestamp into filename (absolute path)
    char filename[1024];
    snprintf(filename, sizeof(filename), "%s/imu_log_%04d%02d%02d_%02d%02d%02d.csv",
             log_dir,
             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);

    // Open file for writing
    FILE *fp = fopen(filename, "w");
    if (fp == NULL) {
        fprintf(stderr, "Failed to open file path: %s\n", filename);
        perror("fopen");
        return 1;
    }

    // Write header
    fprintf(fp, "timestamp,roll,pitch,yaw,gx,gy,gz,ax,ay,az\n");

    printf("File created and opened: %s\n", filename);


    //
    //Main Loop: Update the interface and process data
    //

    bool running = true;
    mip::Timestamp prev_print_timestamp = getCurrentTimestamp();
    mip::Timestamp prev_log_timestamp = getCurrentTimestamp();

    printf("Sensor is configured... waiting for filter to enter running mode.\n");

    while(running)
    {
        device->update();
 
        //Check Filter State
        if((!filter_state_running) && ((filter_status.filter_state == data_filter::FilterMode::GX5_RUN_SOLUTION_ERROR) || (filter_status.filter_state == data_filter::FilterMode::GX5_RUN_SOLUTION_VALID)))
        {
            printf("NOTE: Filter has entered running mode.\n");
            filter_state_running = true;
        }

        //Once in running mode, print out data at 1 Hz
        if(filter_state_running)
        {
           mip::Timestamp curr_timestamp = getCurrentTimestamp();

           if(DO_PRINT && curr_timestamp - prev_print_timestamp >= PRINTING_PERIOD)
           {
                printf("TOW = %f: ATT_EULER = [%f %f %f]: COMP_ANG_RATE = [%f %f %f]: COMP_ACCEL = [%f %f %f]\n",
                       filter_gps_time.tow, filter_euler_angles.roll, filter_euler_angles.pitch, filter_euler_angles.yaw, 
                       filter_comp_angular_rate.gyro[0], filter_comp_angular_rate.gyro[1], filter_comp_angular_rate.gyro[2],
                       filter_comp_accel.accel[0], filter_comp_accel.accel[1], filter_comp_accel.accel[2]);

                prev_print_timestamp = curr_timestamp;
           }
           if(DO_LOG && curr_timestamp - prev_log_timestamp >= LOGGING_PERIOD)
           {
                fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                       filter_gps_time.tow, filter_euler_angles.roll, filter_euler_angles.pitch, filter_euler_angles.yaw, 
                       filter_comp_angular_rate.gyro[0], filter_comp_angular_rate.gyro[1], filter_comp_angular_rate.gyro[2],
                       filter_comp_accel.accel[0], filter_comp_accel.accel[1], filter_comp_accel.accel[2]);

                prev_log_timestamp = curr_timestamp;
           }
        }

        running = !should_exit();
    }

    fclose(fp); // close imu log csv

    exit_gracefully("Example Completed Successfully.");
}


////////////////////////////////////////////////////////////////////////////////
// Print Usage Function
////////////////////////////////////////////////////////////////////////////////

int usage(const char* argv0)
{
    printf("Usage: %s <port> <baudrate>\n", argv0);
    return 1;
}


////////////////////////////////////////////////////////////////////////////////
// Exit Function
////////////////////////////////////////////////////////////////////////////////

void exit_gracefully(const char *message)
{
    if(message)
        printf("%s\n", message);

#ifdef _WIN32
    int dummy = getchar();
#endif

    exit(0);
}


////////////////////////////////////////////////////////////////////////////////
// Check for Exit Condition
////////////////////////////////////////////////////////////////////////////////

bool should_exit()
{
  return false;
}


////////////////////////////////////////////////////////////////////////////////
// mkdir -p helper
////////////////////////////////////////////////////////////////////////////////

static bool mkdir_p(const char* path, mode_t mode)
{
    if(!path || !*path)
        return false;

    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s", path);

    // Remove trailing slash
    size_t len = strlen(tmp);
    if(len > 0 && tmp[len - 1] == '/')
        tmp[len - 1] = '\0';

    for(char* p = tmp + 1; *p; p++)
    {
        if(*p == '/')
        {
            *p = '\0';
            if(mkdir(tmp, mode) != 0 && errno != EEXIST)
                return false;
            *p = '/';
        }
    }

    if(mkdir(tmp, mode) != 0 && errno != EEXIST)
        return false;

    return true;
}

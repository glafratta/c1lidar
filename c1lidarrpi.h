/**
 * Copyright (C) 2021-2025 by Bernd Porr
 **/

#ifndef C1LIDARRPI_H
#define C1LIDARRPI_H

#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "rplidarsdk/sdk/include/rplidar.h"

using namespace rp::standalone::rplidar;

/**
 * One of the 8192 distance datapoints of a scan
 **/
class C1LidarData
{
  public:
    /**
     * Distance in m
     **/
    float r;

    /**
     * Angle in rad with phi=0 in front of the robot
     **/
    float phi;

    /**
     * X position in m where positive values are in front
     * of the robot and negative behind.
     **/
    float x;

    /**
     * Y position in m where positive values are left in front of the
     * robot and negative right in front of the robot.
     **/
    float y;

    /**
     * Signal strength as a value between 0 and 1.
     **/
    float signal_strength = 0;

    /**
     * Flag if the reading is valid
     **/
    bool valid = false;
};

/**
 * Class to continously acquire data from the LIDAR
 **/
class C1Lidar
{
  public:
    /**
     * Serial device on a Raspberry PI
     */
    static constexpr char RPI_SERIAL_DEV[] = "/dev/ttyAMA0";
    
    /**
     * Serial device on a Rock5
     */
    static constexpr char ROCK5_SERIAL_DEV[] = "/dev/ttyS2";

    /**
     * The RPM of the LIDAR which is const at 10Hz or 600RPM
     * Unit is RPM.
     */
    static constexpr float RPM = 10 * 60;

    /**
     * Number of distance readings during one 360 degree
     * rotation.
     **/
    static constexpr unsigned nDistance = 8192;

    /**
     * Baudrate of the C1
     **/
    static constexpr int baudrate = 460800;

    /**
     * Starts the data acquisition by spinning up the
     * motor and then saving the data in the current
     * buffer and providing the data via the callback.
     **/
    void start (const char *serial_port);

    /**
     * Stops the data acquisition
     **/
    void stop ();

    /**
     * Destructor which stops the motor and the data acquisition thread.
     **/
    ~C1Lidar () { stop (); }

    /**
     * Callback interface which needs to be implemented by the user.
     * The newScanAvail is an array with fixed length so can also be
     * processed by an iterator.
     **/
    struct DataInterface
    {
        virtual void newScanAvail (C1LidarData (&)[C1Lidar::nDistance]) = 0;
    };

    /**
     * Register the callback interface here to receive data.
     **/
    void registerInterface (DataInterface *di) { dataInterface = di; }

    /**
     * Returns the current databuffer which is not being written to.
     **/
    inline C1LidarData (&getCurrentData ())[nDistance]
    {
        readoutMtx.lock ();
        return c1LidarData[!currentBufIdx];
        readoutMtx.unlock ();
    }

  private:
    static const int nPackets = 90;
    DataInterface *dataInterface = nullptr;
    void getData ();
    void run ();
    int tty_fd = 0;
    bool running = true;
    C1LidarData c1LidarData[2][nDistance];
    std::thread th;
    std::mutex readoutMtx;
    bool dataAvailable = false;
    int currentBufIdx = 0;
    RPlidarDriver *drv;
    RplidarScanMode scanMode;
};

#endif

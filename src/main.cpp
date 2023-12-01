
#include "../include/Parser.hpp"
#include <thread>
#include <chrono>
#include <boost/asio.hpp>

#define BAUDRATE_DATA  B921600


int32_t radar_handle_1;
int32_t radar_handle_2;
std::string radar_1_data_port = "/dev/ttyUSB1";
std::string radar_2_data_port = "/dev/ttyUSB3";

std::string target_ip_address = "10.0.201.84";
int target_port = 12345; 

AWR_Sensor sensor_1;
AWR_Sensor sensor_2;

std::vector<s_Detected_Object> objects;

std::mutex object_lock;

boost::asio::io_context io_context;

std::thread *read_serial_thread_1;
std::thread *read_serial_thread_2;
std::thread *get_objects_thread_1;
std::thread *get_objects_thread_2;
std::thread *send_udp_thread;

bool exit_request = 0;



bool Open_Serial_Port(int radar_number)
{
    int32_t *handle = (radar_number==1)? &radar_handle_1 : &radar_handle_2;
    std::string data_port = (radar_number==1)? radar_1_data_port : radar_2_data_port;

    if (*handle <= 0)
    {
        *handle = open(data_port.c_str(), O_RDONLY); 
        if (*handle < 0) {
            std::cout << "FAILED TO OPEN THE SERIAL PORT, ERROR CODE: " + *handle << std::endl;
            return false;
        }
        struct termios terminal_setting;

        cfsetispeed (&terminal_setting, (speed_t)BAUDRATE_DATA);
        cfsetospeed (&terminal_setting, (speed_t)BAUDRATE_DATA);
        terminal_setting.c_cflag     &=  ~PARENB;            // Make 8n1
        terminal_setting.c_cflag     &=  ~CSTOPB;
        terminal_setting.c_cflag     &=  ~CSIZE;
        terminal_setting.c_cflag     |=  CS8;
        terminal_setting.c_cflag     &=  ~CRTSCTS;           // no flow control
        terminal_setting.c_cc[VMIN]   =  1;                  // read block
        terminal_setting.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
        terminal_setting.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
        cfmakeraw(&terminal_setting);
        if (tcsetattr(*handle, TCSANOW, &terminal_setting) != 0)
        {
            std::cout << "FAILED TO OPEN THE DATA SERIAL PORT" << std::endl;
            return false;
        }
        return true;
    }
    else if (handle != NULL)
    {
         std::cout <<  "FAILED TO OPEN DATA PORT: ERROR CODE " + std::to_string(*handle) << std::endl;
        return false;
    }
    else
    {
         std::cout << "DATA PORT IS ALREADY OPEN" << std::endl;
        return false;
    }
}


void Read_Sensors_Raw_Data_1()
{
    uint8_t received_character;
    int8_t read_result;
    while (!exit_request)
    {
        if (radar_handle_1 <= 0)
        {
            sleep(2);
            continue;
        }
        read_result = read(radar_handle_1, &received_character, 1);
        if (read_result == 1)
        { 
            sensor_1.Add_Character_To_Buffer(received_character);
        }
    }
}


void Read_Sensors_Raw_Data_2()
{
    uint8_t received_character;
    int8_t read_result;
    while (!exit_request)
    {
        if (radar_handle_2 <= 0)
        {
            sleep(2);
            continue;
        }
        read_result = read(radar_handle_2, &received_character, 1);
        if (read_result == 1)
        { 
            sensor_2.Add_Character_To_Buffer(received_character);
        }
    }
}


void Get_Processed_Data_1()
{
    std::vector<s_Detected_Object> obj;
    while (!exit_request)
    {
        if (sensor_1.Get_Detected_Objects(&obj) == true)
        {
            std::lock_guard<std::mutex> lock(object_lock);
            std::cout << "RADAR 1 HAS FOUND " << obj.size() << " OBJECTS." << std::endl; 
            objects.insert(objects.end(), obj.begin(), obj.end());
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}


void Get_Processed_Data_2()
{
    std::vector<s_Detected_Object> obj;
    while (!exit_request)
    {
        if (sensor_2.Get_Detected_Objects(&obj) == true)
        {
            std::lock_guard<std::mutex> lock(object_lock);
            std::cout << "RADAR 2 HAS FOUND " << obj.size() << " OBJECTS." << std::endl; 
            objects.insert(objects.end(), obj.begin(), obj.end());
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
}


void Send_UDP()
{
    boost::asio::ip::udp::socket socket(io_context);

    boost::asio::ip::udp::endpoint target_endpoint(boost::asio::ip::address::from_string(target_ip_address), target_port);
    socket.open(boost::asio::ip::udp::v4());

    while (!exit_request)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::lock_guard<std::mutex> lock(object_lock);
        if (objects.size() <= 0)
        {
           std::cout << "UDP STACK: NO DETECTED OBJECT" << std::endl; 
           continue;
        }
        else
        {
           std::cout << "NOW SENDING " << objects.size() << " DETECTED OBJECTS ..." << std::endl; 
            socket.async_send_to(
                boost::asio::buffer(objects, objects.size()*sizeof(s_Detected_Object)),
                target_endpoint,
                [](const boost::system::error_code& error, std::size_t bytesTransferred) {
                    if (!error) {
                        std::cout << "MESSAGE SENT, TOTAL OBJECTS: " << objects.size() << std::endl;
                    } else {
                        std::cerr << "ERROR SENDING MESSAGE: " << error.message() << std::endl;
                    }
                }
            );
        }
        objects.clear();
    }
}


void IO_Context_Run()
{
    std::cout << "IO CONTEXT STARTED" << std::endl;
    io_context.run();
}


void Exit_Signal_Handler(int signal_number)
{
    std::cout << "EXIT CALLED" << std::endl; 
    io_context.stop();
    io_context.~io_context();           // TODO: Io contet won't stop, and inhibits app from exit. need to be fixed later
    exit_request = true;
}


int main(int argc, char* argv[])
{
    if (argc == 2) 
    {
        std::cout << "IP ADDRESS SET TO: " << argv[1] << " , PORT: 12345" << std::endl;
        target_ip_address = (argv[1]);
    }
    else if (argc == 3) 
    {
        std::cout << "IP ADDRESS SET TO: " << argv[1] << " , PORT: " << argv[2] << std::endl;
        target_ip_address = (argv[1]);
        target_port = std::stoi(argv[2]);
    }
    else
    {
        std::cout << "IP ADDRESS SET TO: 192.168.0.9 , PORT: 12345" << std::endl;
    }

    if (Open_Serial_Port(1) == true)
    {
        std::cout << "DATA PORT 1 OPENED SUCCESSFULY" << std::endl;
    }
    if (Open_Serial_Port(2) == true)
    {
        std::cout << "DATA PORT 2 OPENED SUCCESSFULY" << std::endl;
    }

    std::signal(SIGINT, Exit_Signal_Handler);
    
    if (radar_handle_1 > 0)
    {
        read_serial_thread_1 = new std::thread(Read_Sensors_Raw_Data_1);
        read_serial_thread_1->detach();
        get_objects_thread_1 = new std::thread(Get_Processed_Data_1);
        get_objects_thread_1->detach();
    }
    
    if (radar_handle_2 > 0)
    {
        read_serial_thread_2 = new std::thread(Read_Sensors_Raw_Data_2);
        read_serial_thread_2->detach();
        get_objects_thread_2 = new std::thread(Get_Processed_Data_2);
        get_objects_thread_2->detach();
    }

    if ((radar_handle_1 > 0) || (radar_handle_2 > 0))
    {
        send_udp_thread = new std::thread(Send_UDP);
        send_udp_thread->detach();
    
        std::thread io_context_thread(IO_Context_Run);
        io_context_thread.detach();
    }
    else
    {
        std::cout << "NO RADAR CONNECTED" << std::endl;
    }

    while (true)
    {
        sleep(1);
    }
    
}
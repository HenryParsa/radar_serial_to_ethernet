
#ifndef AWR1843AOP_AWR_SENSOR_HPP
    #define AWR1843AOP_AWR_SENSOR_HPP

    #include <unistd.h>
    #include <stdio.h>
    #include <errno.h>
    #include <termios.h>
    #include <fcntl.h>
    #include <thread>
    #include <mutex>
    #include <vector>
    #include <iostream>
    #include <iomanip>
    #include <math.h>
    #include <string>
    #include <condition_variable>
    #include "Common.hpp"


    class AWR_Sensor 
    {
        private:

            std::mutex _mutex_objects;
            std::vector<uint8_t> _packet_vector;

            enum e_PARSE_STATE {
                AWR1843_PARSE_STATE0_UNSYNC = 0,
                magic02=1,
                magic01=2,
                magic04=3,
                magic03=4,
                magic06=5,
                magic05=6,
                magic08=7,
                magic07=8,
                AWR1843_PARSE_STATE1_GOT_SYNC = 9,
            };

            e_BUFFERS selected_buffer = e_BUFFERS::Ping;
            enum e_PARSE_STATE state = e_PARSE_STATE::AWR1843_PARSE_STATE0_UNSYNC ;
            const int16_t MAX_PACKE_BUFFER_SIZE = 5000;
            s_Header _header;
            std::condition_variable _condition_var;
            bool _detected_object_buffer_has_updated;
            int16_t _number_of_detected_objects;
            int _current_processing_frame_number;
            std::vector<s_Detected_Object> _s_Objects_Ping;
            std::vector<s_Detected_Object> _s_Objects_Pong;
            uint32_t _total_processed_detection_frames;
            int _total_detected_objects_in_frame = 0;
            s_TLV _tlv;
            bool    _print_diag_data;
            uint    _byte_counter;
            int     _device_handle;
            void    _Header_Parser();
            void    _TLV_Parser(int index);


        public:
            explicit AWR_Sensor();
            ~AWR_Sensor();
            bool    Get_Detected_Objects(std::vector<s_Detected_Object> * objects);
            void    Add_Character_To_Buffer(uint8_t character) ;
            void    Print_Diag_Data(bool enable) ;

    };

#endif
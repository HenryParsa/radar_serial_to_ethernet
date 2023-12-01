
#ifndef AWR1843AOP_COMMON_HPP
    #define AWR1843AOP_COMMON_HPP
   
    #include <cstring>
    #include <vector>
    #include <cstdint>

    struct s_Detected_Object
    {
        float speed;
        float x;
        float y;
        float z;
        int sub_frame;
    }; 

    struct s_Header
    {
        int version;
        int total_packet_len;
        int platform;
        int frame_number;
        int cpu_cycle_time;
        int total_objects;
        int number_of_TLVs;
        int sub_frame_number;
    } ;

    struct s_Tracking_Output
    {
        float x_center;
        float y_center;
        float x_velocity;
        float y_velocity;
        float x_size;
        float y_size;
    };

    struct s_Cluster_Output
    {
        float x_center;
        float y_center;
        float z_center;
        float x_size;
        float y_size;
        float z_size;
    }; 


    struct s_TLV{
        int tlv_type;
        int tlv_lentgh;
        int number_of_objects;
        int q_format;
    };


    enum e_BUFFERS
    {
        Ping,
        Pong
    };

    static float Read_Two_Bytes(unsigned char a, unsigned char b, bool Endian = false)
    {
        int value;
        if (Endian)
        {       // Big endians
            value = (a << 8) | b;
        } else 
        {
            value = (b << 8) | a;
        }
        if (value & 0x8000)
        {       // Handle negative values
            value |= 0xFFFF0000;
        }
        return static_cast<float>(value);
    };

    static float Read_Four_Bytes(unsigned char a, unsigned char b, unsigned char c, unsigned char d, bool Endian)
    {
        int value;
        if (Endian)
        {       // Big endian
            value = (a << 24) | (b << 16) | (c << 8) | d;
        }
        else
        {
            value = (d << 24) | (c << 16) | (b << 8) | a;
        }
        if (value & 0x80000000) {   // Handle negative values
            value |= 0xFFFFFFFF00000000;
        }
        return static_cast<float>(value);
    };


    static float Get_Q9_Formatted(int Value, int Q_value)
    {
        float out = static_cast<float>(Value) / (1 << Q_value);
        return out;
    };


    static float Convert_Int_To_Float(int32_t int_value) 
    {
        unsigned char bytes[4];
        memcpy(bytes, &int_value, sizeof(int_value));
        float float_value = *reinterpret_cast<float*>(bytes);
        return float_value;
    };


    static float Read_Four_Bytes(const std::vector<uint8_t> & vector, int start_index, bool Endian)
    {
        int value;
        uint8_t a = vector[start_index];
        uint8_t b = vector[start_index+1];
        uint8_t c = vector[start_index+2];
        uint8_t d = vector[start_index+3];
        if (Endian) // Big endian
            value = a << 24 | b << 16 | c << 8 | d;
        else
            value = d << 24 | c << 16 | b << 8 | a;
            
        if (value & 0x80000000)    // Handle negative values
            value |= 0xFFFFFFFF00000000;
        
        return static_cast<float>(value);
    };


static float Read_Four_Bytes_And_Increase_Index(const std::vector<uint8_t>& vector, int &start_index, bool Endian)
{
    int value;
    uint8_t a = vector[start_index];
    uint8_t b = vector[start_index + 1];
    uint8_t c = vector[start_index + 2];
    uint8_t d = vector[start_index + 3];
    if (Endian) // Big endian
        value = a << 24 | b << 16 | c << 8 | d;
    else
        value = d << 24 | c << 16 | b << 8 | a;

    if (value & 0x80000000)    // Handle negative values
        value |= 0xFFFFFFFF00000000;
    start_index += 4;
    return static_cast<float>(value);
};



static float Read_Two_Bytes(const std::vector<uint8_t> & vector, int start_index, bool Endian)
{
    int value;
    uint8_t a = vector[start_index];
    uint8_t b = vector[start_index+1];
    if (Endian)
    {       // Big endian
        value = a << 8 | b;
    } else 
    {
        value = b << 8 | a;
    }
    if (value & 0x8000)
    {       // Handle negative values
        value |= 0xFFFF0000;
    }
    return static_cast<float>(value);
};

static float Read_Two_Bytes_And_Increase_Index(const std::vector<uint8_t>& vector, int &start_index, bool Endian)
{
    int value;
    uint8_t a = vector[start_index];
    uint8_t b = vector[start_index + 1];
    if (Endian)
    {       // Big endian
        value = a << 8 | b;
    }
    else
    {
        value = b << 8 | a;
    }
    if (value & 0x8000)
    {       // Handle negative values
        value |= 0xFFFF0000;
    }
    start_index += 2;
    return static_cast<float>(value);
};

static float single_precision_float(const std::vector<uint8_t>& vector, int& start_index)
{
    float return_value = 0;
    unsigned char data[] = { vector[start_index],vector[start_index + 1],vector[start_index + 2],vector[start_index + 3] };
    std::memcpy(&return_value, data, sizeof(float));
    return return_value;
};


static float single_precision_float_Increase_Index(const std::vector<uint8_t>& vector, int& start_index)
{
    float return_value = 0;
    unsigned char data[] = { vector[start_index],vector[start_index + 1],vector[start_index + 2],vector[start_index + 3] };
    std::memcpy(&return_value, data, sizeof(float));
    start_index += 4;
    return return_value;
};


#endif
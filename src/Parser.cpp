
#include "../include/Parser.hpp"


AWR_Sensor::AWR_Sensor()
{
}


AWR_Sensor::~AWR_Sensor()
{
}


bool AWR_Sensor::Get_Detected_Objects(std::vector<s_Detected_Object> * objects)
{    
    std::unique_lock<std::mutex> lock(_mutex_objects);
    _condition_var.wait(lock, [&]{ return _detected_object_buffer_has_updated; }); // Wait until the flag becomes true

    std::vector<s_Detected_Object> *p_object;
    if (selected_buffer == e_BUFFERS::Ping) p_object = & _s_Objects_Pong;
    else p_object = & _s_Objects_Ping;

    // std::cout << "GETTING DETECTED OBJECT FROM BUFFER ADDRESS: " << p_object << std::endl;

    if (p_object->size() <= 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        _detected_object_buffer_has_updated = false;
        return 0;
    }
    objects->clear();
    for (const auto& source : *p_object) {
        s_Detected_Object temp_obj;
        temp_obj.speed = source.speed;
        temp_obj.x = source.x;
        temp_obj.y = source.y;
        temp_obj.z = source.z;
        temp_obj.sub_frame = source.sub_frame;
        objects->push_back(temp_obj);
    }
    _detected_object_buffer_has_updated = false;
    // std::cout << "FINISHED GETTING DETECTED OBJECT FROM BUFFER, BUFFER STATUS: " << _detected_object_buffer_has_updated << std::endl;
    return true;
}


void AWR_Sensor::Add_Character_To_Buffer(uint8_t c)
{
    // std::cout << ".";
    switch (state) 
    {
        case AWR1843_PARSE_STATE0_UNSYNC:
            {   
                
                if(c == 0x02){
                state=magic01;}
            }
            break;
        case magic01:
            {
                if(c == 0x01){state=magic04;}
                else {state = AWR1843_PARSE_STATE0_UNSYNC;}
            }
            break;
        case magic04:
            {
                if(c == 0x04){state=magic03;}
                else {state = AWR1843_PARSE_STATE0_UNSYNC;}
            }
            break;
        case magic03:
            {
                if(c == 0x03){state=magic06;}
                else {state = AWR1843_PARSE_STATE0_UNSYNC;}
            }
            break;
        case magic06:
            {
                if(c == 0x06){state=magic05;}
                else {state = AWR1843_PARSE_STATE0_UNSYNC;}
            }
            break;
        case magic05:
            {
                if(c == 0x05){state=magic08;}
                else {state = AWR1843_PARSE_STATE0_UNSYNC;}
            }
            break;
        case magic08:
            {
                if(c == 0x08){state=magic07;}
                else {state = AWR1843_PARSE_STATE0_UNSYNC;} 
            }
            break;
        case magic07:
            {
                if(c== 0x07)
                {
                    if (_print_diag_data) std::cout << "MAGIC KEY FOUND" << std::endl;
                    _packet_vector.clear();
                    _byte_counter=0;
                    state=AWR1843_PARSE_STATE1_GOT_SYNC;
                }
                else state = AWR1843_PARSE_STATE0_UNSYNC;
            }
            break;
        case AWR1843_PARSE_STATE1_GOT_SYNC:
            {
                _packet_vector.push_back(c);
                _byte_counter++;
                
                if(_packet_vector[_byte_counter-1] == 0x07){
                if(_packet_vector[_byte_counter-2] == 0x08){
                if(_packet_vector[_byte_counter-3] == 0x05){
                if(_packet_vector[_byte_counter-4] == 0x06){
                if(_packet_vector[_byte_counter-5] == 0x03){
                if(_packet_vector[_byte_counter-6] == 0x04){
                if(_packet_vector[_byte_counter-7] == 0x01){
                if(_packet_vector[_byte_counter-8] == 0x02){
                     if (_print_diag_data) std::cout << std::endl;
                     if (_print_diag_data) std::cout << "END OF PACKET, NEXT MAGIC KEY FOUND" << std::endl;
                    _Header_Parser();
                    _packet_vector.clear();
                    _byte_counter=0;
                }}}}}}}}}
                if (_packet_vector.size() > MAX_PACKE_BUFFER_SIZE)
                {
                    _packet_vector.clear();
                    _byte_counter = 0;
                }
            break;     
    }
}


void AWR_Sensor::_Header_Parser()
{   
    int index=0;
    _header.version         = Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0);
    _header.total_packet_len= Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0);
    _header.platform        = Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0);
    _header.frame_number    = Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0);
    _header.cpu_cycle_time  = Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0);
    _header.total_objects   = Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0);
    _header.number_of_TLVs  = Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0);
    _header.sub_frame_number= Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0);

    if (_header.total_packet_len != _packet_vector.size())      // TODO: sometimes when the radar is already running and we start the program, we get incomplete data. If we don't get that at the startup then we never face that issue. we need to figureout why
    {
        std::cout << " ^^^ ATTENTION: INCOMPLETE DATA DETECTED, DROPPING THIS FRAME." << std::endl;
        std::cout << "                PACKET LENGHT: " << _header.total_packet_len << ", TOTAL RECEIVED: " << _packet_vector.size() << std::endl;
        _packet_vector.clear();
        return;
    }

    if (_header.frame_number > _current_processing_frame_number)
    {
        if (_print_diag_data) std::cout <<  "----------> START PROCESSING NEW FRAME: " << _header.frame_number << std::endl;
        _total_detected_objects_in_frame = 0;
        _current_processing_frame_number = _header.frame_number;
        if (selected_buffer == e_BUFFERS::Ping)
        {
            selected_buffer = e_BUFFERS::Pong;
            _s_Objects_Pong.clear();
            // std::cout   <<  "PONG BUFFER SELECTED (" << std::to_string(selected_buffer) << ")   ADDRESS: " << &_s_Objects_Pong <<std::endl;
        }
        else
        {
             selected_buffer = e_BUFFERS::Ping;
            _s_Objects_Ping.clear();
            // std::cout   <<  "PING BUFFER SELECTED (" << std::to_string(selected_buffer) << ")   ADDRESS: " << &_s_Objects_Ping << std::endl;
        }

        _detected_object_buffer_has_updated = true;
        _condition_var.notify_one();
    }
    _TLV_Parser(index); 
}


void AWR_Sensor::_TLV_Parser(int index)
{
    int processed_tlvs = 0;
    if (_print_diag_data) std::cout <<  "PACKET LENGHT: " + std::to_string(_header.total_packet_len) 
                                    <<  "  FRAME: " + std::to_string(_header.frame_number) 
                                    <<  "  CPU CYCLE: " + std::to_string(_header.cpu_cycle_time)
                                    <<  "  TOTAL DETECTED OBJECTS: " + std::to_string(_header.total_objects)
                                    <<  "  TOTAL TLVS: " + std::to_string(_header.number_of_TLVs)
                                    <<  "  SUBFRAME: " + std::to_string(_header.sub_frame_number)
                                    <<  std::endl;

    std::lock_guard<std::mutex> lock(_mutex_objects);

    while (processed_tlvs != _header.number_of_TLVs)
    {      
        _tlv.tlv_type    = Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0);
        _tlv.tlv_lentgh  = Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0);

        std::vector<s_Detected_Object> *p_object;
        if (selected_buffer == e_BUFFERS::Ping) p_object = & _s_Objects_Ping;
        else  p_object = & _s_Objects_Pong;

        if(_tlv.tlv_type==1 && _tlv.tlv_lentgh!=0)
        {
            s_Detected_Object detected_object;
            for(int i=0; i<_header.total_objects; i++)
            {
                detected_object.x           = Convert_Int_To_Float(Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0));
                detected_object.y           = Convert_Int_To_Float(Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0));
                detected_object.z           = -( Convert_Int_To_Float(Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0)));
                detected_object.speed       = Convert_Int_To_Float(Read_Four_Bytes_And_Increase_Index(_packet_vector, index, 0));
                detected_object.sub_frame   = _header.sub_frame_number;
                p_object->push_back(detected_object);
                _total_detected_objects_in_frame += 1;
            }
            _total_processed_detection_frames ++;
        }
        else
        {
            index = index + _tlv.tlv_lentgh;
        }
         processed_tlvs ++;
    }
}


void AWR_Sensor::Print_Diag_Data(bool enable)
{
    _print_diag_data = enable;
    if (_print_diag_data) 
    {
        std::cout << "PRINT DIAG ENABLED" << std::endl;
    }
    else
    {
        std::cout << "PRINT DIAG DISABLED" << std::endl;
    }
}
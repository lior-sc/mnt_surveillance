#include "mnt_surveillance_camera_node/codecs/codec_N1.hpp"

using mnt_surveillance::camera_node::codec::Codec_N1;

Codec_N1::Codec_N1()
{
    // do nothing
}

std::list<uint16_t> Codec_N1::decode_data(std::list<uint8_t> encoded_data)
{
    /**
     * I know this is not the most efficient way but lets go with it for now
    */
    std::list<uint8_t>::iterator it = encoded_data.begin();
    std::list<uint16_t> decoded_data;
    uint16_t decoded_pixel = 0;
    uint8_t encoded_byte = 0;
    int bit_counter = 0;

    while(it != encoded_data.end())
    {
        encoded_byte = *it;
        for (int i=2;i<=8;i+=2)
        {
            decoded_pixel = decoded_pixel | ((encoded_byte >> (8-i)) & 0x03);
            bit_counter += 2;
            if(bit_counter == 10)
            {
                // move data to msb;
                decoded_data.push_back(decoded_pixel << 6);
                it++;
                encoded_byte = *it;
                decoded_pixel = 0;
                bit_counter = 0;

            }
        }
    }
    return decoded_data;
}

std::list<uint8_t> Codec_N1::encode_data(std::list<uint16_t> decoded_data)
{
    /**
     * I know this is not the most efficient way but lets go with it for now
    */
    std::list<uint16_t>::iterator it = decoded_data.begin();
    std::list<uint8_t> encoded_data;
    uint16_t decoded_pixel = 0;
    uint8_t encoded_byte = 0;
    int bit_counter = 0;

    while(it != decoded_data.end())
    {
        decoded_pixel = *it >> 6;
        for (int i=2;i<=10;i+=2)
        {
            encoded_byte = decoded_pixel | ((decoded_pixel >> (10-i)) & 0x03);
            bit_counter += 2;
            if(bit_counter >= 8)
            {
                encoded_data.push_back(encoded_byte);
                it++;
                decoded_pixel = *it >> 6;
                encoded_byte = 0;
                bit_counter = 0;
            }
        }
    }
    return encoded_data;
}

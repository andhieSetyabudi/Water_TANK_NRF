#include "serialProtocol.h"

serialProtocol::serialProtocol()
{
    this->keyWord = NULL;
    this->key_len = 0;
    // this->ptr_ = NULL;
    this->local_ident = {0};
    this->local_ident.cmd = COM_UNKNWON;
}


serialProtocol::serialProtocol(byte* key, uint8_t len)
{
    serialProtocol();
    this->attachKey(key, len);
}

serialProtocol::serialProtocol(byte* key, uint8_t len, protocolPtr ptr_list[], uint8_t ptr_len)
{
    serialProtocol();
    this->attachKey(key, len);
    this->attachISR(ptr_list, ptr_len);
}

serialProtocol::~serialProtocol()
{
    delete this->keyWord;
    // delete this->ptr_;
}


void serialProtocol::attachKey( byte* key, uint8_t len)
{
    this->keyWord = key;
    // memcpy(this->keyWord, key, len);
    this->key_len = len;
}
void serialProtocol::attachISR(protocolPtr ptr_list[], uint8_t ptr_len)
{
    // for ( uint8_t u = 0; u < ptr_len; u++ )
    // {
    //     this->ptr_[u] = ptr_list[u];
    // }
    this->ptr_ = ptr_list;
    this->ptr_len = ptr_len;
}


bool serialProtocol::isCMDValid(uint8_t key)
{
    if ( this->keyWord == NULL )
        return false;
    for(uint8_t i=0; i<this->key_len; i++)
    {
        uint8_t k = (uint8_t)this->keyWord[i];
        if(key==k)
            return true;
    }
    return false;
}


bool serialProtocol::isPackageValid(const uint8_t* raw, uint8_t length, CMD_Identifier* ret)
{
    // memset((uint8_t*)ret, 0, sizeof(CMD_Identifier));
    uint8_t loc = 0;
    uint8_t proc = 0;
    int byte_left =0;
    uint32_t CRC_calc=0;
    while(loc<length)
    {
        // printf("data raw : %d \r\n", raw[loc]);
        if( raw[loc] != 0x00  || loc < length) // compare with NULL
        {
            switch(proc)
            {
                case 0 :    // looking for header
                        if( raw[loc] == COM_HEADER )
                        {
                            ret->header=loc;
                            proc++;
                        }
                        break;
                case 1 :    // looking for key_word
                        if( this->isCMDValid(raw[loc]) )
                        {
                            ret->cmd=raw[loc];
                            proc++;
                        }
                        else
                            proc = 0;
                        break;
                case 2 :    // load the data_length
                        ret->data_length=raw[loc];
                        // printf("data length = %d\r\n", ret->data_length);
                        if( ret->data_length > 0 )
                        {
                            ret->data_location = loc+1;
                            byte_left = length - loc;
                            // printf("byte left is : %i", byte_left);
                            if(byte_left < (6+ret->data_length) )
                                return false;
                            else
                            {
                                if( ret->data_length > sizeof(ret->dataRaw))
                                    memcpy( ret->dataRaw, raw+ret->data_location, sizeof(ret->dataRaw) );
                                else
                                    memcpy( ret->dataRaw, raw+ret->data_location, ret->data_length );
                            };
                        }
                            
                        else
                            ret->data_location = 0;
                        proc++;
                        break;
                case 3 :    // load CRC-32bit
                        // confirming rest of byte
                        byte_left = length - loc;
                        if( byte_left < 6 )
                            return false;
                        if( ret->data_length > 0 )
                            loc +=ret->data_length;
                        ret->CRC32_ =  raw[loc];
                        ret->CRC32_ = (ret->CRC32_ << 8) | raw[loc+1];
                        ret->CRC32_ = (ret->CRC32_ << 8) | raw[loc+2];
                        ret->CRC32_ = (ret->CRC32_ << 8) | raw[loc+3];
                        CRC_calc = this->crc32((const char *)ret->dataRaw,ret->data_length);
                        // mySer.println("");
                        // mySer.print("data CRC = ");
                        // mySer.println(ret->CRC32_,HEX);
                        // mySer.print("data CRC from calculate : ");
                        // mySer.println(CRC_calc,HEX);
                        if( CRC_calc != ret->CRC32_ )
                            return false;
                        proc++;
                        loc +=3;
                        
                        // printf("data CRC from calculate : %x \r\n", CRC_calc);
                        
                        break;
                case 4 :    // confirming the footer
                default :
                        if( raw[loc+1] == COM_END )
                        {
                            ret->nack_ack = raw[loc];
                            // printf("found footer\r\n");
                            ret->footer=loc+1;
                            proc++;
                            return true;
                        }
                        else
                            proc = 0;
                        break;
            }    
        }
        else
            return false;
        if( proc > 4 )
            break;
        loc++;
    }
    if( proc > 4)
        return true;
    if( loc >= length && proc <= 4 )
        return false;
    return true;
}

uint8_t serialProtocol::createPackage(byte CMD, uint8_t nack_ack, uint8_t* raw, uint8_t len, uint8_t* package_, uint8_t package_len)
{
    if( !isCMDValid(CMD) )
        return 0;
    if ( nack_ack != COM_ACK && nack_ack != COM_NAK )
        return 0;
    if( package_len > 32 )
        package_len = 32;
    if( package_len < 9 ) // minimum data size, encapsulated size in bytes is 9
        return 0;
    if( len > 23 )      // max payload of NRF is 32bytes, but 9 bytes already used by package encapsulated
        return 0;
    // create header
    package_[0] = COM_HEADER;
    // command
    package_[1] = CMD;
    // data_length
    package_[2] = len;
    // data raw
    if( len > 0 )
    {
        for(uint8_t l = 0; l < len; l++)
        {
            if ( 3+l <= 32 )
                package_[3+l] = raw[l];
        }
        
    }
    // CRC32-bit
    uint32_t CRC32_ = this->crc32((const char*)raw,len);
    uint8_t crcByte[4];
    this->uint32To4bytes(CRC32_, crcByte);
    for(uint8_t l=0; l<4; l++)
    {
        package_[3+len+l] = crcByte[l];    
    }
    // NACK/ACK
    package_[3+len+4] = nack_ack;
    // Footer
    package_[3+len+5] = COM_END;
    return ( 9+len );
}

bool serialProtocol::updatePackage(const char* dataRaw, uint8_t len)
{
    this->local_ident = {0};
    if( this->isPackageValid((const uint8_t*)dataRaw, len, &this->local_ident) )
    {
        if ( this->keyWord != NULL && this->ptr_ != NULL && this->key_len!= 0 )
        { // execute your void raw
            for(uint8_t loc_ptr = 0; ( loc_ptr < this->ptr_len && this->ptr_!=NULL ); loc_ptr++)
            {
                if( this->local_ident.cmd == this->keyWord[loc_ptr] )
                {
                    if( this->ptr_[loc_ptr] != NULL )
                        this->ptr_[loc_ptr]();
                }
            }
        }
        // mySer.println("FOUND CMD !");
        return true;
    }
    else
        return false;
}
#include <Arduino.h>
#include "C3DFBS.h"

#if defined __has_include
#  if __has_include ("BitBang_I2C.h")
#    include "BitBang_I2C.h"
#    define HAS_I2C_BITBANG  
#  endif
#endif

/// @brief Constructor
/// @param mode Communication protocol - COMMS_I2C or COMMS_SPI
/// @param inputArg If mode is COMMS_I2C, inputArg is the i2c address of the C3DFBS device. 
///                 If mode is COMMS_SPI, inputArg is the chip select pin for the C3DFBS device.
/// @param interrupt The interrupt pin for this device
/// @param nReset The reset pin for this device (active LOW)
C3DFBS::C3DFBS(comms_protocol_t mode, uint8_t inputArg, int interrupt, int nReset, C3DFBSCallback onStreamStarted, C3DFBSCallback onStreamStopped)
{
    _interrupt = interrupt;
    _nReset = nReset;
    _mode = mode;

	if( mode == COMMS_I2C )    	
		_i2c_address = inputArg;    
	
	if( mode == COMMS_SPI )	    
		_chip_select = inputArg;    

    _onStreamStarted = onStreamStarted;
    _onStreamStopped = onStreamStopped;
}

C3DFBS::~C3DFBS()
{
}

void C3DFBS::begin()
{
    pin_mode(_interrupt, INPUT);
    pin_mode(_nReset, OUTPUT);

    switch(_mode)
    {
        case COMMS_I2C:
            Wire.begin();
            Wire.setClock(400000);
        break;

        case COMMS_SPI:
            _spi_settings = SPISettings(3000000, MSBFIRST, SPI_MODE0);
		    SPI.begin();
            pin_mode(_chip_select, OUTPUT);
            write_pin(_chip_select, HIGH);
        break;

        default:
            break;
    }
}

void C3DFBS::pin_mode(uint8_t pin, uint8_t mode)
{
    pinMode(pin, mode);
}

void C3DFBS::write_pin(uint8_t pin, uint8_t value)
{
    digitalWrite(pin, value);
}

int C3DFBS::read_pin(uint8_t pin)
{
    return digitalRead(pin);
}

void C3DFBS::set_chip_select(uint8_t pin)
{
    _chip_select = pin;
}

/// @brief Reset the sensor.
/// @warning This function includes inline delays of 50ms total. 
/// Any pending address change (made with changeI2CAddress() ) will take effect after a device reset.
void C3DFBS::reset()
{
    write_pin(_nReset, LOW);
    delay(50);
    write_pin(_nReset, HIGH);
    delay(50); // Delay min 20ms
}

void C3DFBS::assertReset()
{
    write_pin(_nReset, LOW);
}

void C3DFBS::deassertReset()
{
    write_pin(_nReset, HIGH);
}

/// @brief Read the 3D force vector
/// @param vector An array to hold x, y, z forces
/// @return True if force vector was read successfully
bool C3DFBS::forces(float* vector)
{
    if(_streaming)
    {
        // Return the most recent data, stored in private _forces array
        memcpy(vector, (uint8_t*)_forces, sizeof(_forces) );
    }
    else
    {
        float data[7];
        status_t status = getData(data);
        if(status != SUCCESS)
            return false;

        memcpy(vector, (uint8_t*)data, 3*sizeof(float));
    }

    return true;
}

void C3DFBS::setClockFrequency(uint32_t f)
{
    switch(_mode)
    {
        case COMMS_I2C:
            Wire.setClock(f);
        break;

        case COMMS_SPI:
            _spi_settings = SPISettings(f, MSBFIRST, SPI_MODE0);
        break;

        default:
        break;
    }
}

C3DFBS::status_t C3DFBS::read(int address, uint8_t* buffer, uint8_t length, bool await_interrupt, uint32_t inter_byte_delay_us)
{
	status_t returnError = SUCCESS;
	uint8_t i = 0;

	switch (_mode) 
    {
        case COMMS_I2C:
            if( address >= 0)
            {
                Wire.beginTransmission(_i2c_address);
                Wire.write(address);
                if( Wire.endTransmission() != 0 )            
                    return HW_ERROR;
            }

            if(buffer == nullptr || length == 0)
                return SUCCESS;            

            // Request bytes from slave device
            Wire.requestFrom((int)_i2c_address, (int)length);
            while ( (Wire.available()) && (i < length))
            {
                *buffer = Wire.read();   
                if(inter_byte_delay_us > 0)
                    delayMicroseconds(inter_byte_delay_us);
                buffer++;
                i++;
            }            
            break;

        case COMMS_SPI:
            if(!await_interrupt){
                // Enforce a delay between SPI commands
                while(abs((long)(micros() - _last_spi_transaction)) < SPI_INTER_CMD_DELAY_US);
            }

            if(await_interrupt)            
                waitForInterrupt(HIGH, _data_timeout_us);
            
            SPI.beginTransaction(_spi_settings);
            write_pin(_chip_select, LOW);

            if(await_interrupt)            
                waitForInterrupt(LOW, _data_timeout_us);            

            if(address >= 0)
                SPI.transfer(address); // address = command

            if( (length != 0) && (buffer != nullptr) )
            {
                if(!await_interrupt)                
                    delayMicroseconds(SPI_CMD_TO_REPLY_DELAY_US);

                while ( i < length )
                {
                    *buffer = SPI.transfer(0x00);
                    if(inter_byte_delay_us > 0)
                        delayMicroseconds(inter_byte_delay_us);
                    buffer++;
                    i++;
                }                   
            }
            _last_spi_transaction = micros();  

            write_pin(_chip_select, HIGH);
            SPI.endTransaction();        
            break;

        default:
            break;
	}

	return returnError;
}

C3DFBS::status_t C3DFBS::write(int address, uint8_t *buffer, uint8_t length)
{
	status_t returnError = SUCCESS;

	switch (_mode) 
    {
	case COMMS_I2C:
        // Only send address if it is valid
        if(address >= 0)
        {
            Wire.beginTransmission(_i2c_address);
            Wire.write(address);
            if( Wire.endTransmission() != 0 )		
                returnError = HW_ERROR;
        }
        
        if( (length == 0) || (buffer == nullptr) )
            return returnError;

        Wire.beginTransmission(_i2c_address);
		Wire.write(buffer, length);
		if( Wire.endTransmission() != 0 )		
			returnError = HW_ERROR;		
		break;

	case COMMS_SPI:
        // Enforce a delay between SPI commands
        while(fabs((long)(micros() - _last_spi_transaction)) < SPI_INTER_CMD_DELAY_US);

        SPI.beginTransaction(_spi_settings);
        write_pin(_chip_select, LOW);

        // Send address/command
		SPI.transfer(address);

        if( (length != 0) && (buffer != nullptr) )
        {
            delayMicroseconds(SPI_CMD_TO_REPLY_DELAY_US);
            SPI.transfer(buffer, length);
        }

        _last_spi_transaction = micros();  
        
        write_pin(_chip_select, HIGH);
        SPI.endTransaction();
        
		break;
		
	default:
		break;
	}

	return returnError;
}

C3DFBS::status_t C3DFBS::waitForInterrupt(uint8_t value, int timeout_us)
{
    if(timeout_us < 0)
    {   // Wait forever
        while(read_pin(_interrupt) != value);
    }
    else
    {   // Wait with timeout
        uint32_t start = micros();
        while(read_pin(_interrupt) != value)
        {
            if(micros() - start > (uint32_t)timeout_us)
                return STREAM_READ_ERROR; // timeout
        }   
    }

    return SUCCESS;         
}

C3DFBS::status_t C3DFBS::readU32(uint8_t cmd, uint32_t* data)
{
    ByteUnion<uint32_t> t;
    auto ret = read(cmd, t.bytes, sizeof(uint32_t));

    if(ret != SUCCESS)
        return ret;    

    *data = t.value;

    return SUCCESS;    
}

C3DFBS::status_t C3DFBS::writeU32(uint8_t cmd, uint32_t data)
{
    ByteUnion<uint32_t> t;
    t.value = data;

    return write(cmd, t.bytes, sizeof(uint32_t));
}

C3DFBS::status_t C3DFBS::sendCommand(uint8_t cmd) 
{
	return write(cmd);
}

bool C3DFBS::isAlive()
{
    uint8_t value;
    auto ret = read(Command::NOP, &value);

    if(ret != SUCCESS)
        return false;    
    
    return value == SENSOR_OK; // The expected return value
}

bool C3DFBS::isValidAddress(uint8_t i2c_address)
{
    uint8_t value;
    // Copy the internal address
    uint8_t prev_address = _i2c_address;

    // Override the internal address before issuing the NOP command
    _i2c_address = i2c_address;
    auto ret = read(Command::NOP, &value);

    // Restore the internal address
    _i2c_address = prev_address;

    if(ret != SUCCESS)
        return false;    
    
    return value == SENSOR_OK; // The expected return value
}

C3DFBS::status_t C3DFBS::getVersion(String* ver)
{
    const int strSize = 32;
    uint8_t str[strSize]; 

    auto ret = read(Command::GET_VERSION, str, strSize);

    if(ret != SUCCESS)
        return ret;    

    *ver = String((char*)str);   

    return SUCCESS;
}

C3DFBS::status_t C3DFBS::whoAmI(String* s)
{
    const int strSize = 32;
    uint8_t str[strSize]; 

    auto ret = read(Command::GET_WHO_AM_I, str, strSize);

    if(ret != SUCCESS)
        return ret;    

    *s = String((char*)str);   

    return SUCCESS;
}

C3DFBS::status_t C3DFBS::getTemperature(float* temp)
{
    ByteUnion<float> t;

    auto ret = read(Command::GET_TEMP, t.bytes, sizeof(float));

    if(ret != SUCCESS)
        return ret;    

    *temp = t.value;

    return SUCCESS;    
}

C3DFBS::status_t C3DFBS::setDataFrequency(uint32_t freq)
{
    _data_timeout_us = (uint32_t)(1000000 * 2.0 * 1.0 / freq); // Timeout is twice the active period
    return writeU32(Command::SET_ACTIVE_FREQ, freq);
}

C3DFBS::status_t C3DFBS::getDataFrequency(uint32_t* freq)
{
    status_t status = readU32(Command::GET_ACTIVE_FREQ, freq);  

    if(status == SUCCESS)
        _data_timeout_us = (uint32_t)(1000000 * 2.0f * 1.0f / *freq); // Timeout is twice the active period 

    return status;
}

/// @brief Configure which fields are returned when the sensor enters active streaming mode.
/// @param fields A bitfield of included data. Bit flag values are contained in the DataFields enum.
/// @return Code indicating command success.
C3DFBS::status_t C3DFBS::setDataFields(uint32_t fields)
{
    auto ret = writeU32(Command::SET_ACTIVE_DATA, fields);
    if(ret != SUCCESS)
        return ret;

    // Readback and verify, then set field read size
    uint32_t newFields;
    ret = getDataFields(&newFields);
    if(ret != SUCCESS)
        return ret;

    if(fields != newFields)
        return COMMAND_FAILED;

    _dataFieldsMask = newFields;
    setReadSizeBytes();

    return SUCCESS;
}

C3DFBS::status_t C3DFBS::getDataFields(uint32_t* fields)
{
    return readU32(Command::GET_ACTIVE_DATA, fields);  
}

bool C3DFBS::isStreaming()
{
    return _streaming;
}

C3DFBS::status_t C3DFBS::startDataStream()
{
    // Guard to prevent starting if nothing to be streamed
    if(_fieldReadSizeBytes <= 0)
        return OUT_OF_BOUNDS;
    
    uint8_t value;
    status_t status = read(Command::ENTER_ACTIVE, &value);

    if(status == SUCCESS)
    {
        if(value != SENSOR_OK) // The expected return value    
            status = COMMAND_FAILED;

        _streaming = true;
        if(_onStreamStarted != nullptr)
            _onStreamStarted(this);
    }
    
    return status;   
}

C3DFBS::status_t C3DFBS::stopDataStream()
{
    uint8_t value;
    status_t status = COMMAND_FAILED;

    // There are some outstandind quirks when exiting ACTIVE mode.
    // I2C and SPI behave differently, so we need a case handler here.

    switch(_mode)
    {
        case COMMS_I2C:
            // The first response may be lost.
            // Send one command, but don't try to read the reply
            status = sendCommand(Command::NOP);
            if(status != SUCCESS)  
                return status;

            // Send another command and read the reply                  
            status = read(Command::NOP, &value);
            break;

        case COMMS_SPI:
            uint8_t dummy[2]; // Read two bytes, discard the first.      
            status = read(NOP, (uint8_t*)&dummy, sizeof(dummy), true, EXIT_ACTIVE_INTER_BYTE_DELAY_US); // Wait for interrupt, wait between bytes
            value = dummy[1];
            break;

        default:
            // Should never reach here
            break;
    }

    // Clear streaming flag here regardless of status
    _streaming = false;
    if(_onStreamStopped != nullptr)
        _onStreamStopped(this);

    if(status == SUCCESS)
    {
        if(value != SENSOR_OK) // The expected return value    
            status = COMMAND_FAILED;
    }
    
    return status;    
}

/// @brief Read the data stream. The stream will contain the fields configured with setDataFields().
/// @param buffer A buffer large enough to hold the returned data. 
//  The minimum buffer size can be queried with getDataStreamSize().
/// The constant ALL_FIELDS_DATA_SIZE_BYTES defines the buffer size required if the data stream is configured to send everything -> setDataFields(All).
/// @return Code indicating read success.
C3DFBS::status_t C3DFBS::readDataStream(uint8_t* buffer)
{
    if(_fieldReadSizeBytes <= 0)
        return OUT_OF_BOUNDS;

    // Set address to -1 to read only. Read bytes from sensor.
    auto status = read(-1, buffer, _fieldReadSizeBytes, true);

    if(status == SUCCESS)
    {
        if(_dataFieldsMask & ForceX)
            _forces[0] = *(float*)buffer;
        else
            _forces[0] = NAN;

        if(_dataFieldsMask & ForceY)
            _forces[1] = *(float*)(buffer + sizeof(float));
        else
            _forces[1] = NAN;

        if(_dataFieldsMask & ForceZ)
            _forces[2] = *(float*)(buffer + 2*sizeof(float));
        else
            _forces[2] = NAN;        
    }

    return status;

}

/// @brief This is an overloaded method provided for convenience. Stream data will be copied into the provided DataPacket.
/// @warning Only the fields set with setDataFields() will be copied into the provided DataPacket.
/// @param data A DataPacket struct.
/// @return Code indicating command success.
C3DFBS::status_t C3DFBS::readDataStream(DataPacket* data)
{
    uint8_t buffer[ALL_FIELDS_DATA_SIZE_BYTES];
    auto ret = readDataStream(buffer);

    if(ret != SUCCESS)
        return ret;

    // Fill struct with received data
    uint32_t offset = 0;

    if(_dataFieldsMask & Temperature)
    {
        data->temperature = *reinterpret_cast<float*>(buffer + offset);
        offset += sizeof(float);
    }
    if(_dataFieldsMask & ForceZ)
    {
        data->forceZ = *reinterpret_cast<float*>(buffer + offset);
        offset += sizeof(float);
    }
    if(_dataFieldsMask & ForceY)
    {
        data->forceY = *reinterpret_cast<float*>(buffer + offset);
        offset += sizeof(float);
    }
    if(_dataFieldsMask & ForceX)
    {
        data->forceX = *reinterpret_cast<float*>(buffer + offset);
        offset += sizeof(float);
    }

    return SUCCESS;
}

/// @brief Get data. Data is an array of floats: 
/// [ force_x, force_y, force_z, [Reserved], [Reserved], [Reserved], temperature ].     
/// @param buffer A buffer to hold received data.
/// @return Code indicating command success.
C3DFBS::status_t C3DFBS::getData(float* buffer)
{
    const int fieldSize = sizeof(float);

    const int length = NUM_DATA_FIELDS*fieldSize;
    uint8_t data[length];

    auto ret = read(Command::GET_DATA, data, length);

    if(ret != SUCCESS)
        return ret;   

    memcpy(buffer, data, length);
    return SUCCESS; 
}

/// @brief Get data (function overload). Formats received data into a named struct.   
/// @param data A DataPacket* struct object to hold data.
/// @return Code indicating command success.
C3DFBS::status_t C3DFBS::getData(DataPacket* data)
{
    const int numfields = NUM_DATA_FIELDS;
    const int fieldSize = sizeof(float);

    const int length = numfields*fieldSize;
    float buffer[length];

    status_t ret = getData(buffer);

    if(ret != SUCCESS)
        return ret;   

    // Populate struct
    uint32_t offset = 0;
    data->forceX = buffer[offset++];
    data->forceY = buffer[offset++];
    data->forceZ = buffer[offset++];
    offset += 3; // Skip reserved bytes
    data->temperature = buffer[offset++];

    return SUCCESS; 
}

void C3DFBS::printData(DataPacket* data, bool isStream, Print* dest)
{
    if(isStream)
    {
        // Use stream flags to select which data are printed
        dest->print("Fx: ");
        if(_dataFieldsMask & ForceX)
            dest->printf("%3.2f, ", data->forceX);
        else
            dest->printf("--, ");

        dest->print("Fy: ");
        if(_dataFieldsMask & ForceY)
            dest->printf("%3.2f, ", data->forceY);
        else
            dest->printf("--, ");

        dest->print("Fz: ");
        if(_dataFieldsMask & ForceZ)
            dest->printf("%3.2f, ", data->forceZ);
        else
            dest->printf("--, ");

        dest->print("Temp: ");
        if(_dataFieldsMask & Temperature)
            dest->printf("%3.2f", data->temperature);
        else
            dest->printf("--");

    }
    else
    {
        // Print all data
        dest->printf("Fx: %3.2f, Fy: %3.2f, Fz: %3.2f, Temp: %3.2f", data->forceX, data->forceY, data->forceZ, data->temperature);
    }

    if(isnan(data->forceX) || isnan(data->forceY) || isnan(data->forceZ))  
        dest->print(" (unbiased)");  
    
    dest->print("\n");  
}

C3DFBS::status_t C3DFBS::removeBias()
{ 
    uint8_t value = SENSOR_UNKNOWN; // Default
    auto ret = read(Command::BIAS_SENSOR, &value);

    if(ret != SUCCESS)
        return ret;   
        
    if(value == SENSOR_OK)
        return ret; // Success. Sensor biased

    if(value == SENSOR_ERROR)
        return COMMAND_FAILED; // The sensor has replied with a NACK

    // Any other value indicates a problem (probably corrupt data)
    return HW_ERROR;    
}

C3DFBS::status_t C3DFBS::changeI2CAddress(uint8_t new_address)
{
    // Use the private variable _i2c_address as the current address
    return changeI2CAddress(new_address, _i2c_address);
}

/// @brief Change the i2c address of the button sensor, by providing the new and current i2c address.
/// @warning The new address only takes effect after power cycling the sensor.
/// @param new_address The new i2c address. Valid range [0x50 - 0x72]
/// @param current_address The current i2c address. In the case of an error, this may not match the _i2c_address variable internal to this class.
/// @return Code indicating command success.
C3DFBS::status_t C3DFBS::changeI2CAddress(uint8_t new_address, uint8_t current_address)
{
    const int bootloader_address = 0x55; // Reserved address.
    uint8_t storedAddress = 0x00;

    // Validate address range.    
    if( (new_address < 0x50 || new_address > 0x72) || new_address == bootloader_address )
        return OUT_OF_BOUNDS;

    // Override the current address
    auto previous_address = _i2c_address;
    _i2c_address = current_address;

    // Then send the NEW I2C address
    auto ret = write(Command::SET_I2C_ADDR, &new_address);

    if(ret != SUCCESS)
    {        
        _i2c_address = previous_address;
        return ret;
    }

    // Delay to allow time to store in flash
    delay(FLASH_WRITE_DELAY_MS);

    // Read back the address to verify.     
    ret = read(-1, &storedAddress);

    if(ret != SUCCESS)
    {        
        _i2c_address = previous_address;
        return ret;
    }
    
    if(storedAddress != new_address)          
        return CONFIRMATION_ERROR; // If we got here the transfer command succeeded, but the wrong address was returned!    

    // Reset the sensor to activate the new address
    reset();

    // Modify the address
    _i2c_address = new_address; 

    // Then see if it responds to the new address
    if(!isAlive())
    {
        // Sensor did not respond at this new address
        _i2c_address = current_address; // Restore old address
        return COMMAND_FAILED; 
    }    

    return SUCCESS;    
}

C3DFBS::status_t C3DFBS::setCommsMode(uint8_t protocol)
{
    C3DFBS::status_t ret = COMMAND_FAILED;
    uint8_t new_protocol = SENSOR_UNKNOWN; // Initialise to something invalid

    // Establish a working comms protocol
    bool use_default_protocol = isAlive();
    
    // Send the new protocol via a working comms interface
    if(use_default_protocol)
    {
        ret = write(Command::CMD_SET_COMMS, (uint8_t*)&protocol);
        if(ret != SUCCESS)  
            return ret;

        delay(FLASH_WRITE_DELAY_MS); // Wait for flash write
                    
        ret = read(-1, &new_protocol);
    }
 #ifdef HAS_I2C_BITBANG    
    else
    {   
        // Try bit banging via SPI pins
        if(_mode == COMMS_SPI)
        {                 
            BBI2C bbi2c;
            bbi2c.bWire = 0;
            bbi2c.iSDA = SS;
            bbi2c.iSCL = SCK;
            I2CInit(&bbi2c, 100000);  
            uint8_t resp;

            // Send command
            uint8_t cmd = Command::CMD_SET_COMMS;
            resp = I2CWrite(&bbi2c, _i2c_address, &cmd, 1);
            if(!resp)        
                return HW_ERROR;
            
            // Send the new protocol
            resp = I2CWrite(&bbi2c, _i2c_address, (uint8_t*)&protocol, 1);
            if(!resp)        
                return HW_ERROR;

            delay(FLASH_WRITE_DELAY_MS); // Wait for flash write

            // Read back the protocol to verify
            resp = I2CRead(&bbi2c, _i2c_address, (uint8_t*)&new_protocol, 1);
            if(!resp)        
                return HW_ERROR;   
        }    
    }    
 #endif         

    if(ret != SUCCESS)                
        return ret;

    if(new_protocol != protocol)
        return COMMAND_FAILED;

    // Reset the sensor to activate the new comms mode
    reset();
    return SUCCESS;
}

/// @brief Set communication protocol
/// @param mode 
/// @return 
C3DFBS::status_t C3DFBS::setCommunicationProtocol(uint8_t mode)
{
	/*  Firmware version >= 1.1.0 uses bootstrapping to set the comms mode.
		The state of SDA/CS and SCL/SCK pins are used to determine the desired comms mode.

		----------------------------------------
		|      |  bit 1  |  bit 0 |            |
		----------------------------------------
		| INT  | SCL/SCK | SDA/CS | COMMS MODE |
		----------------------------------------
		| HIGH |    x   |    x    |  Unchanged | <- If INT is HIGH, no change.
		----------------------------------------
		| LOW  |    0   |    0    |  I2C Mode  |
		----------------------------------------
		| LOW  |    0   |    1    |  SPI Mode  |
		----------------------------------------
		| LOW  |    1   |    0    |  UART Mode |
		----------------------------------------
		| LOW  |    1   |    1    |  Unchanged | 
		----------------------------------------

		The user must set the SDA/CS and SCL/SCK pins, 
		then set INT HIGH again (within 3 seconds) to lock in the desired comms mode. 
	*/
    
    // Prepare pins for OUTPUT
    pin_mode(_nReset, OUTPUT);
    pin_mode(_interrupt, OUTPUT);
    pin_mode(_chip_select, OUTPUT);
    pinMode(SDA, OUTPUT);
    pinMode(SCL, OUTPUT);    
    pinMode(SCK, OUTPUT);

    // Place sensor in reset
    write_pin(_nReset, LOW);

    // Set INT pin LOW to enter bootstrapping mode on boot    
    write_pin(_interrupt, LOW);

    /* Set mode-select pins
       Write the logic values to both the I2C and SPI clock/data pins, 
       so that this function succeeds regardless of how the user has the device connected. 
    */
    // Bit 0 - either SDA or CS
    write_pin(_chip_select, mode & 0x01);
    digitalWrite(SDA, mode & 0x01);
    // Bit 1 - either SCL or SCK
    digitalWrite(SCL, mode & 0x02); 
    digitalWrite(SCK, mode & 0x02);

    // Delay to make sure device goes offline
    delay(50); 

    // Release reset to restart the sensor
    write_pin(_nReset, HIGH);
    
    /* Delay to ensure device has started. 
      Min delay = 30ms:
       - 25ms until the bootloader exits and the user app has started
       - 5ms for the sensor to read the INT pin 
    */
    delay(50);
    
    // Set INT pin HIGH to lock-in the comms mode
    write_pin(_interrupt, HIGH);

    // Delay at least 30ms to allow time for flash variables to be written
    delay(50);

    // Restore pin function
    pin_mode(_interrupt, INPUT);

    return SUCCESS;
}

C3DFBS::status_t C3DFBS::setCommunicationProtocol(comms_protocol_t protocol)
{
    return setCommunicationProtocol((uint8_t)protocol);
}

void C3DFBS::setReadSizeBytes()
{
    int readSize = 0;
    int i = 0;
    uint32_t n = _dataFieldsMask;
    
    while (i < _numberOfDataFields) 
    {
        if(n & 0x0001)
            readSize += _fieldReadSizes[i];
        n >>= 1;
        i++;
    }

    _fieldReadSizeBytes = readSize;
}

/// @brief Get the size of the data stream (in bytes) as configured by setDataFields()
/// @return Integer number of bytes in the data stream.
int C3DFBS::getDataStreamSize()
{
    return _fieldReadSizeBytes;
}

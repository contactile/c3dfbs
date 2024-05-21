#ifndef C3DFBS_H
#define C3DFBS_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

class C3DFBS
{
    public:

        typedef enum
        {
            SUCCESS,
            HW_ERROR,
            NOT_SUPPORTED,
            GENERIC_ERROR,
            OUT_OF_BOUNDS,
            COMMAND_FAILED,
            STREAM_READ_ERROR,
            CONFIRMATION_ERROR,
            OUT_OF_MEMORY, 
        }status_t;

        typedef enum
        {
            COMMS_I2C = 0,
            COMMS_SPI = 1,
            COMMS_UNKNOWN = 3,
        }comms_protocol_t;

        typedef struct
        {
            float temperature;
            float forceZ;
            float forceY;
            float forceX;
        }DataPacket;      

        typedef enum : uint32_t
        {           
            Temperature = 0x20,
            ForceZ = 0x200,
            ForceY = 0x400,
            ForceX = 0x800,
            AllForce = 0xE00,
        }DataFields;

        #define NUM_DATA_FIELDS 7
        #define ALL_FIELDS_DATA_SIZE_BYTES (5*U16_SIZE + 7*FLOAT_SIZE)
        #define C3DFBS_DEFAULT_I2C_ADDRESS 0x57
        #define SENSOR_OK 0xAA
        #define SENSOR_ERROR 0xFE
        #define SENSOR_UNKNOWN 0xFF

        typedef void (*C3DFBSCallback)(C3DFBS*);     

        C3DFBS(comms_protocol_t mode, uint8_t inputArg, int interrupt, int nReset, C3DFBSCallback onStreamStarted = nullptr, C3DFBSCallback onStreamStopped = nullptr);
        virtual ~C3DFBS();

        void begin();
        void reset();  
        bool isAlive();      
        bool forces(float* vector);      

        status_t getData(float* buffer);  
        status_t getData(DataPacket* buffer);
        status_t getTemperature(float* temp);
        status_t getVersion(String* ver);
        status_t whoAmI(String* s);
        status_t setDataFrequency(uint32_t freq);
        status_t getDataFrequency(uint32_t* freq);
        status_t setDataFields(uint32_t fields);
        status_t getDataFields(uint32_t* fields);
        status_t startDataStream();
        status_t stopDataStream();
        status_t readDataStream(uint8_t* buffer);
        status_t readDataStream(DataPacket* data);
        status_t removeBias();
        status_t changeI2CAddress(uint8_t new_address);
        status_t changeI2CAddress(uint8_t new_address, uint8_t current_address);        
        status_t setCommunicationProtocol(comms_protocol_t protocol);
        status_t setCommunicationProtocol(uint8_t protocol);
        bool isStreaming();
        status_t setCommsMode(uint8_t protocol);  

        void printData(DataPacket* data, bool isStream = false, Print* dest = &Serial);

        int getDataStreamSize();
        bool isValidAddress(uint8_t i2c_address); 
        void setClockFrequency(uint32_t f); 
        void assertReset();
        void deassertReset();

    protected:
        typedef enum
        {
            ERROR = 0x00,
            NOP = 0x01,
            GET_DATA = 0x03,
            GET_TEMP = 0x04,
            GET_VERSION = 0x06,
            SET_ACTIVE_FREQ = 0x09,
            GET_ACTIVE_FREQ = 0x0A,
            SET_ACTIVE_DATA = 0x0B,
            GET_ACTIVE_DATA = 0x0C,
            ENTER_ACTIVE = 0x0D,
            BIAS_SENSOR = 0x0F,
            SET_I2C_ADDR = 0x10,
            GET_WHO_AM_I = 0x11,
            CMD_SET_COMMS = 0x13,
            Default
        }Command;   

        // These pin manipulation functions are virtual to provide a mechanism
        // to override Arduino pinMode, digitalWrite, and digitalRead functions,
        // as may be required if using, e.g. a port expander IC.
        virtual void pin_mode(uint8_t pin, uint8_t mode);
        virtual void write_pin(uint8_t pin, uint8_t value);
        virtual int read_pin(uint8_t pin);
        void set_chip_select(uint8_t pin);

        template <typename T>
        union ByteUnion
        {
            unsigned char bytes[sizeof(T)];
            T value;
        };

        uint32_t _dataFieldsMask;        
        uint8_t _i2c_address = C3DFBS_DEFAULT_I2C_ADDRESS;

        // Read and write via the current command protocol
        status_t read(int address, uint8_t *data, uint8_t length = 1, bool await_interrupt = false, uint32_t inter_byte_delay = 0);
        status_t write(int address, uint8_t *data = nullptr, uint8_t length = 1); 
        status_t sendCommand(uint8_t cmd);
        
    private:       
        #define U16_SIZE sizeof(uint16_t)
        #define FLOAT_SIZE sizeof(float)

        #define SPI_CMD_TO_REPLY_DELAY_US           250  // Delay between command phase and data phase
        #define SPI_INTER_CMD_DELAY_US              10   // Delay between the end of one data phase and the start of the next command
        #define FLASH_WRITE_DELAY_MS                30   // Delay between command phase and data phase for FLASH write commands
        #define EXIT_ACTIVE_INTER_BYTE_DELAY_US     10   // Delay between bytes in the data phase for the stopDataStream() command

        const int _numberOfDataFields = 12;
        const int _fieldReadSizes[12] = {U16_SIZE, U16_SIZE, U16_SIZE, U16_SIZE, U16_SIZE, FLOAT_SIZE, FLOAT_SIZE, FLOAT_SIZE, FLOAT_SIZE, FLOAT_SIZE, FLOAT_SIZE, FLOAT_SIZE};
        float _forces[3] = {NAN, NAN, NAN};
        int _fieldReadSizeBytes = 0;
        uint32_t _last_spi_transaction = 0;
        uint32_t _data_timeout_us = 1000000;
        SPISettings _spi_settings;
        
        comms_protocol_t _mode;
        int _chip_select;
        int _interrupt;
        int _nReset;
                
        bool _streaming = false;
        void (*_onStreamStarted)(C3DFBS* sender) = nullptr;
        void (*_onStreamStopped)(C3DFBS* sender) = nullptr;
        
        status_t waitForInterrupt(uint8_t value, int timeout_us = -1);         
        status_t readU32(uint8_t cmd, uint32_t* data);
        status_t writeU32(uint8_t cmd, uint32_t data);
        
        void setReadSizeBytes();   
        
        
};

#endif // C3DFBS_H
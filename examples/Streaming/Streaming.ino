/* Streaming.ino

  This example demonstrates how to stream data from a Contactile 3D Force Button Sensor. 

*/

#include <C3DFBS.h>

// Define pins
const uint8_t ss_pin = SS;      // Slave select pin
const uint8_t gpio_pin = 8;     // Used for interrupts/signalling
const uint8_t reset_pin = 9;    // Used to reset/program the C3DFBS sensor

/* Create a C3DFBS class instance in SPI mode */
C3DFBS sensor = C3DFBS(C3DFBS::COMMS_SPI, ss_pin, gpio_pin, reset_pin);
C3DFBS::status_t status;

// Stream control
uint32_t frequency = 100; // 25 - 1500 Hz
bool streaming = false;

void setup() 
{
  Serial.begin(115200);

  /* Wait for serial connection before running example */
  while(!Serial);

  Serial.println("-- C3DFBS streaming example --\n");

  sensor.begin();
  sensor.setClockFrequency(1000000); // select a clock frequency compatible with your microcontroller

  /* Wait for C3DFBS connection */
  while(1) 
  {
    Serial.print("Verifying connection... ");
    if(sensor.isAlive())
    {   
      Serial.println("OK");
      break;
    }
    else
    {   
      Serial.println("Error: Sensor missing. Check connections.");
    }
    
    delay(1000);
  }

  /* Print C3DFBS information */
  String info;  

  Serial.println("Querying sensor... ");
  status = sensor.whoAmI(&info);
  if(status == C3DFBS::SUCCESS)
  {  
    Serial.print("Connected to ");
    Serial.println(info);
  }
  else
  {  
    Serial.print("Error: Failed to read who-am-i. Error code ");
    Serial.println(status);
  }

  status = sensor.getVersion(&info);
  if(status == C3DFBS::SUCCESS)
  {
    Serial.print("Version: %s\n");
    Serial.println(info);
  }
  else
  {  
    Serial.print("Error: Failed to read version. Error code ");
    Serial.println(status);
  }

  /* Zero the sensor. This function should be invoked when the sensor is in an idle state, with no forces being applied */
  Serial.print("Biasing sensor... ");
  status = sensor.removeBias();
  if(status == C3DFBS::SUCCESS) 
  { 
    Serial.println("OK. Sensor bias removed");  
  }
  else
  {  
    Serial.print("Error: Failed to remove sensor bias. Error code ");
    Serial.println(status);
  }
  /* Configure data stream contents. */  
  Serial.print("Setting data stream contents... ");
  uint32_t fields = C3DFBS::AllForce | C3DFBS::Temperature;
  status = sensor.setDataFields(fields);
  if(status == C3DFBS::SUCCESS)
  {
    Serial.print("OK. Data fields mask set to 0x");
    Serial.print(fields, HEX);
    Serial.print(". Expecting ");
    Serial.print(sensor.getDataStreamSize());
    Serial.println(" bytes in data stream");
  }
  else
  { 
    Serial.print("Error: Setting data fields failed. Error code ");
    Serial.println(status);
  }

  /* Configure data stream frequency */
  Serial.print("Setting data stream frequency... ");
  status = sensor.setDataFrequency(frequency);
  if(status == C3DFBS::SUCCESS)
  {   
    Serial.print("OK. Stream frequency set to ");
    Serial.print(frequency);
    Serial.println(" Hz");
  }
  else
  {
    Serial.print("Error: Setting stream frequency failed. Error code ");
    Serial.println(status);
  }
  Serial.println("Configuration complete.");
  Serial.println("Press 's' to start or stop streaming data");

}

void loop() 
{
  while(Serial.available())
  {
    uint8_t c = (uint8_t)Serial.read();

    if(c == 's' || c == 'S')
    { // Start/stop the stream when the S key is pressed           
      if(streaming)
      {
        streaming = false;
        status = sensor.stopDataStream();
        if(status != C3DFBS::SUCCESS)
        {                  
          Serial.print("Error: Stream failed to stop with error code ");
          Serial.println(status);
        }
      }
      else
      {                
        status = sensor.startDataStream();
        if(status != C3DFBS::SUCCESS)
        {               
          Serial.print("Error: Unable to start data stream. Error code ");
          Serial.println(status);
        }
        else
        {            
          streaming = true;
        }
      }
    }
  }

  if(streaming)
  {
    // Read data stream
    C3DFBS::DataPacket data;
    status = sensor.readDataStream(&data);

    if(status == C3DFBS::SUCCESS)
    {
      /* Print data
          Note that although we are printing everything here, 
          only the fields set with setDataFields(...) will be updated.
          The other fields will contain default values. 
      */
      Serial.print(millis());
      Serial.print(" Data: [ temp ");
      Serial.print(data.temperature, 2);
      Serial.print(", forceX ");
      Serial.print(data.forceX, 2);
      Serial.print(", forceY ");
      Serial.print(data.forceY, 2);
      Serial.print(", forceZ ");
      Serial.print(data.forceZ, 2);
      Serial.println(" ]");
    }
    else
    {
      Serial.print(millis());
      Serial.print(" Error: Data stream read failed with error code ");
      Serial.println(status);
      delay(100);
    }
  }
    
}

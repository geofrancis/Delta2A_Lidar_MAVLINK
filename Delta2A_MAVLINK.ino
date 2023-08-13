
#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/common/mavlink_msg_obstacle_distance.h"


#define MOTOR_MOSFET_PIN       19 // define the pin to control the motor key 
#define FRAME_HEADER           0XAA // define the packet header 
#define DATA_HEADER            0xAD // define the value of the data start byte 
#define SIZE_HI_BYTE           1 // define the location in the packet of the high size byte data without checksum 
#define SIZE_LO_BYTE           2 // define the place in the packet of the low byte of the size of the data without checksum 
#define DATA_START_BYTE        5 // define the place in the packet of the payload start byte    
#define DISTANCE_DATA_SYZE     7 // define the location in the packet of the distance data length 
#define START_ANGLE_HI_BYTE    11 // define the location of the high byte of the packet's start angle 
#define START_ANGLE_LO_BYTE    12 // determine the location of the low byte of the packet's start angle 
#define FIRST_DIST_HI_BYTE     14 // determine the location of the high byte of the first distance probes in packet 
#define FIRST_DIST_LO_BYTE     15 // define low byte location of first distance probe in packet 
// end of datasheet 
// array to store distances 

uint8_t raw_data [ 256 ];       // buffer for storing "raw" data 
bool data_ready = false ;   // data ready flag 
long lidar_delay = 1000 ; // output interval to the serial port 
long current_millis = 0 ; // current millis() to compare 


unsigned long previousMillis = 0;   
const long interval = 100;        

int lidarAngle = 0;
int messageAngle = 0;
uint16_t distances[72];
int target = 0;
unsigned char data_buffer[4] = {0};
int adjusted = 0;
int distance = 0;
int range = 0;
unsigned char CS;
uint8_t Index;
byte received;

HardwareSerial Serial1(USART1);
HardwareSerial Serial2(USART2);

char serial_buffer[15];



void setup()
{
  lidar.setup(RPM);
  Serial.begin(1500000); // USB
  Serial1.begin(230400); // FC  
  Serial2.begin(1500000); // FC  

  pinMode(MOTOR_MOSFET_PIN,OUTPUT);      // set motor key pin to output mode 
  digitalWrite(MOTOR_MOSFET_PIN,HIGH);   // set motor key output to logical one mode 
  memset(raw_data, 0, 256);              // reset the buffer
  memset(distances, UINT16_MAX, sizeof(distances)); // Filling the distances array with UINT16_MAX
}
int16_t Dist = 0;    // Distance to object in centimeters

void loop()
{
lidarAngle = packet.angle_quad;
messageAngle = map(lidarAngle, 0, 89, 0, 72);
  bool got_packet;
  
  got_packet=lidar.processAvailable(&packet);
  lidar.applyMotorPID();

  if(got_packet)
  {
    readLIDAR();
    send_pos();
  }
  
  //serial print will slow down the board, disable if not using.
  //  Serial.print("angle ");
  //  Serial.print(lidarAngle);
  //  Serial.print("distance ");
  //  Serial.println((packet.distances[0]));    
  //  Serial.println(distances[messageAngle/res]);
}

void readLIDAR(){
 


   // if the timeout has elapsed and the data is ready
    if(millis() - current_millis > lidar_delay && data_ready) {
        for(int i = 0; i < 360; i++) {

             // output text to serial monitor 
            Serial.print("Angle: ");
          // if the angle is less than 10 
            if(i < 10){
                Serial.print(" ");
            }
            Serial.print(i);
            Serial.print("\tDistance: ");
            Serial.println(lidar_data[i]);

           //  write current millis 
            current_millis = millis();

            reset the data readiness flag 
            data_ready = false;
        }
    }

   // read the start byte of the packet
    while(Serial1.available()) {

        // read the start byte of the packet
        if (Serial1.read() == (raw_data[0] = FRAME_HEADER)){
            for (int i = 1; i!= 256; i++){

                // fill buffer 
                raw_data[i] = Serial1.read();
            }
        }
    }

// if the data byte matches the packet header
        if(raw_data[DATA_START_BYTE] == DATA_HEADER) {

        // get the payload length (2nd and 3rd bytes of raw data)
        uint16_t raw_data_length = (raw_data[SIZE_HI_BYTE] << 8) + raw_data[SIZE_LO_BYTE];

       // get checksum from raw data
        uint16_t checksum = (raw_data[raw_data_length] << 8) + raw_data[raw_data_length+1];
        delay(10);

        // call the function for calculating the checksum
        if (checksum == checksum_cmp(raw_data, raw_data_length)) {    // if the sum passed 

            // calculation of the starting angle of this packet 
            float start_angle = ((raw_data[START_ANGLE_HI_BYTE] << 8)
                                    + raw_data[START_ANGLE_LO_BYTE])*0.01;

            // count the number of distance samples: number of samples = (distance data size - 5) / 3 
            uint8_t read_count = (raw_data[DISTANCE_DATA_SYZE] - 5) /  3;
            for (int n = 0; n < read_count; n++){

              // count the angle of each sample:
              // angle = start angle + 22.5° * (sample number - 1)/ number of samples float angle = start_angle + 22.5 * ( n - 1
                float angle = start_angle + 22.5 * (n - 1) / read_count;

                // iterator for lidar_data array 
                int i = int(angle);

                // distance calculation
                float distance = ((raw_data[FIRST_DIST_HI_BYTE+n*3] << 8)
                                    + raw_data[FIRST_DIST_LO_BYTE+n*3]) * 0.25;

                // writing the distance to the array 
                lidar_data[i] = distance;
            }
        }
        data_ready = true;
    }
}

// checksum calculation function.
uint16_t checksum_cmp(uint8_t *raw_data, uint16_t raw_data_length) {
    uint16_t _checksum = 0;
    while (raw_data_length--) {
      // add up all bytes of the packet 
        _checksum += *raw_data++;
    }
    return _checksum;

    
    if(distance > 0){
    Dist = (distance/10);
}
 else{
     Dist = 0;
     }
}


void send_pos(){///////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 196;    
  uint64_t time_usec = 0; /*< Time since system boot*/
  uint8_t sensor_type = 0;
  distances[messageAngle] = Dist-2.0f; //UINT16_MAX gets updated with actual distance values
  uint8_t increment = 5;
  uint16_t min_distance = 10; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 650; /*< Maximum distance the sensor can measure in centimeters*/
  float increment_f = 0;
  float angle_offset = -10;
  uint8_t frame = 12;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 30;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 int type = MAV_TYPE_GROUND_ROVER;
  // Pack the message
  
 mavlink_msg_obstacle_distance_pack(sysid,compid,&msg,time_usec,sensor_type,distances,increment,min_distance,max_distance,increment_f,angle_offset,frame);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);

  
  unsigned long currentMillis = millis();
 if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;


  mavlink_msg_heartbeat_pack(1,196, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial2.write(buf, len);
  }
  
  

}

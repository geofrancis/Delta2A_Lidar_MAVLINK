#define MOTOR_MOSFET_PIN       19 // define the pin to control the motor key 
// from the datasheet: 
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

uint8_t raw_data [ 256 ]; 		  // buffer for storing "raw" data 
bool data_ready = false ;   // data ready flag 
long lidar_delay = 1000 ; // output interval to the serial port 
long current_millis = 0 ; // current millis() to compare 

void setup() {

    Serial.begin(115200);                  // initiate serial port for output to USB
    Serial1.begin(230400);                 // first serial port for reading lidar data 
    pinMode(MOTOR_MOSFET_PIN,OUTPUT);      // set motor key pin to output mode 
    digitalWrite(MOTOR_MOSFET_PIN,HIGH);   // set motor key output to logical one mode 
    memset(raw_data, 0, 256);              // reset the buffer
}

void loop() {

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

        // читаем стартовый байт пакета
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
        if (checksum == checksum_cmp(raw_data, raw_data_length)) { 		// if the sum passed 

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
}

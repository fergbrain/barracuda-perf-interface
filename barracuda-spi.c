/**
 * @file      barracuda-spi.c
 * @authors   Andrew Ferguson, for Fergcorp, LLC
 *
 * @brief Control the network ports and LEDs on the chassis of Barracuda Web
 * Filter with serial port interface chassis managment (via USB-to-Serial), with OPNSense installed
 * or other software. Based on work by Matthew Lietz.
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdbool.h>

static uint8_t RED_LED = 0x40;
static uint8_t YELLOW_LED = 0x20;
static uint8_t GREEN_LED = 0x10;

static uint8_t NETWORK_BYPASS = 0x08;

static int serial_fd;

#define SERIAL_PORT "/dev/cuaU0"


uint8_t extractUint8(const char* str) {
    if (str[0] == '\0' || str[1] == '\0') {
        // Handle error condition: string does not contain enough characters
        return 0;
    }

    char substring[3] = { str[1], str[2], '\0' };
    return (uint8_t)strtol(substring, NULL, 16);
}


static void do_init(void){
    struct termios serial_settings;
 
    // Open the serial port
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);

    printf("file status = 0x%x\n", serial_fd); //debugging

    if (serial_fd == -1) {
        perror("Failed to open serial port");
        exit(1);
    }

    // Configure the serial port settings
    if (tcgetattr(serial_fd, &serial_settings) != 0) {
        perror("Failed to get serial port settings");
        close(serial_fd);
        exit(1);
    }

    // Set baud rate to 115200
    cfsetispeed(&serial_settings, B115200);
    cfsetospeed(&serial_settings, B115200);

    // Set 8N1 (8 data bits, no parity, 1 stop bit)
    serial_settings.c_cflag &= ~PARENB;
    serial_settings.c_cflag &= ~CSTOPB;
    serial_settings.c_cflag &= ~CSIZE;
    serial_settings.c_cflag |= CS8;

    // Apply the serial port settings
    if (tcsetattr(serial_fd, TCSANOW, &serial_settings) != 0) {
        perror("Failed to set serial port settings");
        close(serial_fd);
        exit(1);
    }

}

// Write the bits to the Parallel Port
static void do_out(uint8_t outval)
{

  char tx_buffer[10];

  sprintf(tx_buffer, "GRB%02XD%02X\n", outval, outval);
  printf("Sending: %s", tx_buffer);

  //Write to serial port
  if (write(serial_fd, tx_buffer, sizeof(tx_buffer)) == -1) {
        perror("Failed to write to serial port");
        close(serial_fd);
        exit(1);
    }


}


static unsigned long do_in(void){
    // Send and receive signals
    char tx_buffer[] = "GRBD\n"; // Get Gpio Read B and D
    char rx_buffer[100];
    
    // Write to serial port
    if (write(serial_fd, tx_buffer, sizeof(tx_buffer)) == -1) {
        perror("Failed to write to serial port");
        close(serial_fd);
        return 1;
    }
    
    // Read from serial port
    if (read(serial_fd, rx_buffer, sizeof(rx_buffer)) == -1) {
        perror("Failed to read from serial port");
        close(serial_fd);
        return 1;
    }
        
    // Display received data
    printf("Received: '%s'\n", rx_buffer);      

    uint8_t val1 = extractUint8(rx_buffer + 3);
    uint8_t val2 = extractUint8(rx_buffer + 5);

    printf("Value1: '%x'\n", val1);
    printf("Value2: '%x'\n", val2);

    uint8_t combined = (val1 & 0xF0) | (val2 & 0x0F);

    printf("Combined: '%x'\n", combined);    

    return combined;

}

// Change the desired bit in the int that hold the ppi status
void set_pin( uint8_t *val, char pinValue, uint8_t *Pin ) {
  if( !(*val & *Pin) ) {
    if( pinValue == 48 ) { // 0 char
      *val = *val + *Pin;
    }
  } else if( *val & *Pin ) {
    if( pinValue == 49 ) { // 1 char
      *val = *val - *Pin;
    }
  }
}



int main(int argc, char *argv[] ) {
    do_init();
    uint8_t val = do_in();    
    uint8_t startval = val;
    bool printStatus = false;


  // Loop through parameters
  for(int i = 1; i < argc; ++i) {
    if( strcmp(argv[i], "--green") == 0 ||
        strcmp(argv[i], "-g") == 0 ) {
      ++i;
      set_pin( &val, *argv[i], &GREEN_LED );
    } else if( strcmp(argv[i], "--yellow") == 0 ||
               strcmp(argv[i], "-y") == 0 ) {
      ++i;
      set_pin( &val, *argv[i], &YELLOW_LED );
    } else if( strcmp(argv[i], "--red") == 0 ||
               strcmp(argv[i], "-r" ) == 0 ) {
      ++i;
      set_pin( &val, *argv[i], &RED_LED );
    } else if( strcmp(argv[i], "--port") == 0 ||
               strcmp(argv[i], "-p")  == 0 ) {
      ++i;
      set_pin( &val, *argv[i], &NETWORK_BYPASS );
    } else if( strcmp(argv[i], "--all") == 0 ||
               strcmp(argv[i], "-a") == 0 ) {
      ++i;
      set_pin( &val, *argv[i], &GREEN_LED );
      set_pin( &val, *argv[i], &YELLOW_LED );
      set_pin( &val, *argv[i], &RED_LED );
    } else if( strcmp(argv[i], "--status") == 0 ||
               strcmp(argv[i], "-s") == 0 ) {
      printStatus = true;
    } else {
      printf("Barracuda Parallel Port Manager\n"
        "Usage: barracuda-ppi [OPTION] [1 turn on, 0 turn off]\n"
        "-g, --green   turn green LED on/off\n"
        "-y, --yellow  turn yellow LED on/off\n"
        "-r, --red     turn red LED on/off\n"
        "-a, --all     turn all LED's on/off\n"
        "-p, --port    turn port bypass on/off\n"
        "-s, --status  print LED/port status\n" );
    }
  }

    printf("Startval: %x\n", startval);
    printf("CurrVal: %x\n", val);

  if( startval != val ) {
    do_out( val );
  }

if(printStatus) {   
 if(!(val & GREEN_LED))
      printf("Green LED is on.\n");
    if(!(val & YELLOW_LED))
      printf("Yellow LED is on.\n");
    if(!(val & RED_LED))
      printf("Red LED is on.\n");
    if(val & NETWORK_BYPASS)
      printf("Network Bypass is off.\n");
    else
      printf("Network Bypass is on.\n");
}
    // Close the serial port
    close(serial_fd);

    return 0;
}

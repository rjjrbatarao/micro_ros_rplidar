/*
    Note the default code has bootloop using timer when using serial
    maybe because it has to reconnect?
*/


#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "IPAddress.h"
#include <driver/uart.h>
#include <driver/gpio.h>
// custom low level microros library

#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

//#define NO_REBOOT

#define TXD_PIN_2 GPIO_NUM_17
#define RXD_PIN_2 GPIO_NUM_16
#define RXD_BUFFER_SIZE 1024

#define WIFI_SSID "MikroTik0"
#define WIFI_PASSWORD "test"

#define UDP_AGENT "10.0.0.1"
#define UDP_PORT 8888

static const char wifi_ssid[64] = WIFI_SSID;
static const char wifi_password[64] = WIFI_PASSWORD;
char agent_ip[64] = UDP_AGENT;
uint agent_port = UDP_PORT;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

enum transport_type {
    TYPE_SERIAL,
    TYPE_UDP,
};

transport_type t_type = TYPE_SERIAL;

// static const uint8_t request_packet[] = {0xa5, 0x20};

typedef struct RPLidarMeasurement
{
    uint8_t quality;
    float angle;
    float distance;
} __packed RPLidarMeasurement; 

void uart_init_2()
{
    uart_config_t uart_config_2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
        //.rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_NUM_2, &uart_config_2);
    uart_set_pin(UART_NUM_2, TXD_PIN_2, RXD_PIN_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, RXD_BUFFER_SIZE, 0, 0, NULL, 0);
}

void send_simple_start_scan_request ()
{
    static const uint8_t request_packet[] = {0xa5, 0x20};
    uart_write_bytes(UART_NUM_2, (const char*)request_packet, sizeof(request_packet));
}

void send_express_scan_request()
{
    static const uint8_t request_packet[] = {0xa5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22};
    uart_write_bytes(UART_NUM_2, (const char *)request_packet, sizeof(request_packet));
    // vTaskDelay(10/portTICK_PERIOD_MS);
}

void send_reset_scan_request()
{
    static const uint8_t request_packet[] = {0xa5, 0x40};
    uart_write_bytes(UART_NUM_2, (const char*) request_packet, sizeof(request_packet));
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void recieve_response_descriptor()
{
    uint8_t startFlag1[] = {0};
    uint8_t startFlag2[] = {0};

    //================check startFlag==========================
    do
    {
        uart_read_bytes(UART_NUM_2, startFlag1, sizeof(startFlag1), 100 / portTICK_PERIOD_MS);
        if (startFlag1[0] == 0xa5)
        {
            uart_read_bytes(UART_NUM_2, startFlag2, sizeof(startFlag2), 100 / portTICK_PERIOD_MS);
        }

        //======for Debug=========
        printf("read byte: 0x%02x 0x%02x\n", startFlag1[0], startFlag2[0]);

    } while (!(startFlag1[0] == 0xa5 && startFlag2[0] == 0x5a));

    uint8_t bytes[5];
    uart_read_bytes(UART_NUM_2, bytes, sizeof(bytes), 100 / portTICK_PERIOD_MS);

    //=========check for Debug==========
    printf("Response Descriptor: \n0x%02x | 0x%02x | 0x%02x | 0x%02x | 0x%02x | 0x%02x | 0x%02x\n", startFlag1[0], startFlag2[0], bytes[0], bytes[1], bytes[2], bytes[3], bytes[4]);
}

void recieve_measurement_simple_scan ()
{
    uint8_t quality;
    uint16_t angle_q6;
    uint16_t distance_q2;

    uint8_t byte[5];
    uart_read_bytes(UART_NUM_2, byte, sizeof(byte), 100/portTICK_PERIOD_MS);
    quality = byte[0]>>2;
    angle_q6 = (uint16_t)(byte[2]<<7) | (byte[1]>>1);
    distance_q2 = (uint16_t)(byte[4]<<8) | byte[3];
    float actual_angle = angle_q6 / 64.0;
    float actual_distance = distance_q2 / 4.0;
    bool start_flag = 0x01 & byte[0];

    if (start_flag) printf("new 360 scan!!!\n");
    printf("quality: %d\nangle:%.2f\ndistance:%.2f\n", quality, actual_angle, actual_distance);
    //vTaskDelay(50/portTICK_PERIOD_MS);
}

void receive_measurement_legacy_version()
{
    uint8_t sync1[] = {0};
    uint8_t sync2[] = {0};
    uint8_t chkSum = 0;
    uint8_t chkValid = 0;

    //============check sync byte=================
    do
    {
        uart_read_bytes(UART_NUM_2, sync1, sizeof(sync1), 100/portTICK_PERIOD_MS);
        chkSum = sync1[0] & 0x0F;
        sync1[0] = sync1[0] >> 4;
        if(sync1[0]==0xA)
        {
            uart_read_bytes(UART_NUM_2, sync2, sizeof(sync2), 100/portTICK_PERIOD_MS);
        }
        chkSum |= sync2[0] << 4;
        sync2[0] = sync2[0] >> 4;

        //===============for Debug===============
        printf("0x%01x | 0x%01x | %u\n", sync1[0], sync2[0], chkSum);

    } while (! (sync1[0]==0xA && sync2[0]==0x5));

    uint8_t angle_data[2];
    uart_read_bytes(UART_NUM_2, angle_data, sizeof(angle_data), 100/portTICK_PERIOD_MS);
    chkValid ^= angle_data[0] ^ angle_data[1];

    uint16_t start_angle_q6 = (uint16_t)((angle_data[1] & !(0x80)) <<8 ) | angle_data[0];
    float actual_angle = start_angle_q6 / 64.0;

    //=============print start angle (DEBUG)==============
    printf("*****************Response Packet*****************\n");
    printf("Start angle (fix point number): %u\nstart flag bit: %1x\n", start_angle_q6, angle_data[1]>>7);
    printf("Actual angle: %.2f\n", actual_angle);
    printf("0x%02x | 0x%02x\n", angle_data[0], angle_data[1]);

    //==========each cabin inside the data packet===========
    for(int i=0; i< 16; i++)
    {
        uint8_t data[5];
        uint16_t distance1=0, distance2=0;
        uint8_t angle_compensate_1=0, angle_compensate_2=0;
        uint8_t cabin = 1;

        uart_read_bytes(UART_NUM_2, data, sizeof(data), 100/portTICK_PERIOD_MS);
        chkValid ^= data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];

        distance1 = (uint16_t) (data[1] << 6) | (data[0]>>2);
        distance2 = (uint16_t) (data[3] << 6) | (data[2]>>2);
        angle_compensate_1 = (0x0F & data[4]) | ((data[0]<<4) & 0x20);
        angle_compensate_2 = (data[4] >> 4) | ((data[2]<<4) & 0x20);

        printf("\n*****************CABIN %d*****************\n",i);
        cabin += 2;

        //==================DEBUG======================
        printf("Distance 1: %u\nDistance 2: %u\nAngle 1: %u\nAngle 2: %u\n", distance1,distance2, angle_compensate_1, angle_compensate_2);
        printf("0x%02x | 0x%02x | 0x%02x | 0x%02x | 0x%02x\n", data[0], data[1], data[2], data[3], data[4]);
    }
    //checkSum
    if(chkSum == chkValid)printf("CheckSum: %u\n", chkValid);
    {
        printf("Data Verified!!!\n");
    }
}

uint16_t angleDiff(uint8_t start_angle_1, uint8_t start_angle_2)
{
    return (uint16_t)(start_angle_2 - start_angle_1) + (start_angle_1 > start_angle_2) ? 360 : 0;
}

/* --------------------------------------------------micro ros--------------------------------------------*/

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    rcl_publish(&publisher, &msg, NULL);
    msg.data++;
  }
}

 bool custom_serial_transport_open(struct uxrCustomTransport * transport);
 bool custom_serial_transport_close(struct uxrCustomTransport * transport);
 size_t custom_serial_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
 size_t custom_serial_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);


static inline void set_microros_serial_transports(){
	rmw_uros_set_custom_transport(
		true,
		NULL,
		custom_serial_transport_open,
		custom_serial_transport_close,
		custom_serial_transport_write,
		custom_serial_transport_read
	);
}

 bool custom_udp_transport_open(struct uxrCustomTransport * transport);
 bool custom_udp_transport_close(struct uxrCustomTransport * transport);
 size_t custom_udp_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
 size_t custom_udp_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

struct micro_ros_agent_locator {
	IPAddress address;
	int port;
};

static inline void set_microros_udp_transports(){
    static struct micro_ros_agent_locator locator;
	locator.address.fromString(agent_ip);
	locator.port = agent_port;

	rmw_uros_set_custom_transport(
		false,
		(void *) &locator,
		custom_udp_transport_open,
		custom_udp_transport_close,
		custom_udp_transport_write,
		custom_udp_transport_read
	);
}



  bool custom_serial_transport_open(struct uxrCustomTransport * transport)
  {
    Serial.begin(115200);
    return true;
  }

  bool custom_serial_transport_close(struct uxrCustomTransport * transport)
  {
    Serial.end();
    return true;
  }

  size_t custom_serial_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
  {
    (void)errcode;
    size_t sent = Serial.write(buf, len);
    return sent;
  }

  size_t custom_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
  {
    (void)errcode;
    Serial.setTimeout(timeout);
    return Serial.readBytes((char *)buf, len);
  }




  static WiFiUDP udp_client;

  bool custom_udp_transport_open(struct uxrCustomTransport * transport)
  {
    struct micro_ros_agent_locator * locator = (struct micro_ros_agent_locator *) transport->args;
    udp_client.begin(locator->port);
    return true;
  }

  bool custom_udp_transport_close(struct uxrCustomTransport * transport)
  {
    udp_client.stop();
    return true;
  }

  size_t custom_udp_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
  {
    (void)errcode;
    struct micro_ros_agent_locator * locator = (struct micro_ros_agent_locator *) transport->args;

    udp_client.beginPacket(locator->address, locator->port);
    size_t sent = udp_client.write(buf, len);
    udp_client.endPacket();
    udp_client.flush();

    return sent;
  }

  size_t custom_udp_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
  {
    (void) errcode;

    uint32_t start_time = millis();

    while(millis() - start_time < timeout && udp_client.parsePacket() == 0){
      delay(1);
    }

    size_t readed  = udp_client.read(buf, len);

    return (readed < 0) ? 0 : readed;
  }


/** ----------------------------------------------end microros-------------------------------------------*/

void programTask(void *pvParameter) {

    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // uart_init_2();
    // //send_reset_scan_request();
    // vTaskDelay(1000/portTICK_PERIOD_MS);
    // send_express_scan_request();
    // recieve_response_descriptor();
    // receive_measurement_legacy_version();

    if(t_type == TYPE_UDP){
        WiFi.begin(wifi_ssid, wifi_password);
        while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        }
    }

    if(t_type == TYPE_SERIAL)
            set_microros_serial_transports(); 
    if(t_type == TYPE_UDP)
            set_microros_udp_transports();  
          
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
        
    if(t_type == TYPE_SERIAL){
        
        rclc_node_init_default(&node, "micro_ros_serial_node", "", &support);
        rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "micro_ros_serial_node_publisher");

        #ifndef NO_REBOOT
        const unsigned int timer_timeout = 1000;
        rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback);

        rclc_executor_init(&executor, &support.context, 1, &allocator);
        rclc_executor_add_timer(&executor, &timer);
        #endif

    }


    if(t_type == TYPE_UDP){
        
        rclc_node_init_default(&node, "micro_ros_udp_node", "", &support);
        rclc_publisher_init_best_effort(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "micro_ros_udp_node_publisher");
        
    }

    msg.data = 0;

    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        #ifndef NO_REBOOT
        receive_measurement_legacy_version();
        if(t_type == TYPE_SERIAL){
         rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        if(t_type == TYPE_UDP){
            rcl_publish(&publisher, &msg, NULL);
            msg.data++;
        }
        #else
        rcl_publish(&publisher, &msg, NULL);
        msg.data++;
        #endif
    }
}

extern "C" void app_main()
{
    // initialize arduino library before we start the tasks
    initArduino();
    xTaskCreate(&programTask, "program_task", 16384, NULL, 5, NULL);
}



/**
 * i2c_platform_esp.c
 * 
 * I2C device interface for the 
 * Espressif Internet-of-Things (IoT) Development Framework ESP-IDF
 *
 * I2C_Master is the global I2C interface shared by all devices
 * 
 * (c) 2021 by David Asher
 * https://github.com/david-asher
 * https://www.linkedin.com/in/davidasher/
 * This code is licensed under MIT license, see LICENSE.txt for details
 */

#include "i2c_platform_esp.h"

typedef struct {

    i2c_port_t 		  port; 
    gpio_num_t 		  pin_sda; 
    gpio_num_t 		  pin_scl; 
    uint32_t   		  freq;
    uint8_t           dev_address;
    i2c_cmd_handle_t  cmd_handle;

} I2C_Master_t;

I2C_Master_t *I2C_Master = (I2C_Master_t *) NULL;

void i2c_scan()
{
    printf("\r\nI2C device scan: ");
    for (uint8_t i = 1; i < 127; i++)
    {
        int ret;
        uint8_t test_address = i << 1;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK( i2c_master_start(cmd) );
        ESP_ERROR_CHECK( i2c_master_write_byte(cmd, test_address | I2C_MASTER_WRITE, 1) );
        ESP_ERROR_CHECK( i2c_master_stop(cmd) );
        ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK) continue;
        printf("0x%02X | ", test_address );
    }
    printf( "\r\n" );
}

I2C_Master_t *i2c_master_setup()
{
    I2C_Master_t *new_master = (I2C_Master_t *) malloc( sizeof( I2C_Master_t ) );
    new_master->port = I2C_DEFAULT_PORT;
    new_master->pin_sda = I2C_DEFAULT_SDA; 
    new_master->pin_scl = I2C_DEFAULT_SCL;
    new_master->freq = I2C_DEFAULT_FREQ;
    new_master->dev_address = I2C_NO_DEVICE;
    new_master->cmd_handle = NULL;
    return new_master;
}

void i2c_get_config( i2c_port_t *port, gpio_num_t *pin_sda, gpio_num_t *pin_scl, uint32_t *freq )
{
    *port = I2C_Master->port;
    *pin_sda = I2C_Master->pin_sda; 
    *pin_scl = I2C_Master->pin_scl;
    *freq = I2C_Master->freq;
}

void i2c_init_driver()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_Master->pin_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_Master->pin_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_Master->freq;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    ESP_ERROR_CHECK( i2c_param_config( I2C_Master->port, &conf ) );
    ESP_ERROR_CHECK( i2c_driver_install( I2C_Master->port, conf.mode, 0, 0, 0 ) );
}

void i2c_init_config( i2c_port_t port, gpio_num_t pin_sda, gpio_num_t pin_scl, uint32_t freq )
{
    if ( I2C_Master != (I2C_Master_t *) NULL ) return;
    I2C_Master = i2c_master_setup();
    I2C_Master->port = port;
    I2C_Master->pin_sda = pin_sda; 
    I2C_Master->pin_scl = pin_scl;
    I2C_Master->freq = freq;
    i2c_init_driver();
}

void i2c_init() 
{
    if ( I2C_Master != (I2C_Master_t *) NULL ) return;
    I2C_Master = i2c_master_setup();
    i2c_init_driver();
    i2c_filter_enable(I2C_Master->port, 7);
}

void i2c_remove() 
{
    ESP_ERROR_CHECK( i2c_driver_delete( I2C_Master->port ) );
    free( I2C_Master );
    I2C_Master = (I2C_Master_t *) NULL;
}

void i2c_upgrade( uint32_t upgrade_freq )
{
    i2c_port_t port;
    gpio_num_t pin_sda;
    gpio_num_t pin_scl;
    uint32_t   old_freq;

    i2c_get_config( &port, &pin_sda, &pin_scl, &old_freq );
    i2c_remove();
    i2c_init_config( port, pin_sda, pin_scl, upgrade_freq );
}

bool i2c_start( uint8_t i2c_device_address, i2c_rw_t read_write )
{
    // be thread-safe
    if ( I2C_Master->cmd_handle == (i2c_cmd_handle_t) NULL ) {
        // I2C_MASTER was not processing a command, so start a new one
        I2C_Master->dev_address = i2c_device_address;
        I2C_Master->cmd_handle = i2c_cmd_link_create();
    }
    else if ( I2C_Master->dev_address != i2c_device_address ) {
        // check for the case where a second thread is attempting to start a command on a second device
        // while a first device is processing a command, and reject it
        return false;
        // start() may legitimately be called multiple times, e.g. to write a command then read the result
    }
    ESP_ERROR_CHECK( i2c_master_start( I2C_Master->cmd_handle ) );
    ESP_ERROR_CHECK( i2c_master_write_byte( I2C_Master->cmd_handle, I2C_Master->dev_address | read_write, ACK_CHECK_EN ) );
    return true;
}

size_t i2c_write_byte( uint8_t data_byte_out )
{
    if ( I2C_Master->cmd_handle == (i2c_cmd_handle_t) NULL ) return 0;
    ESP_ERROR_CHECK( i2c_master_write_byte( I2C_Master->cmd_handle, data_byte_out, ACK_CHECK_EN ) );
    return 1;
}

size_t i2c_write( uint8_t *pByteBuffer, size_t NumByteToWrite )
{
    uint16_t byte_index;
    if ( I2C_Master->cmd_handle == (i2c_cmd_handle_t) NULL ) return 0;
    // Note: Needed to use i2c_master_write_byte as i2c_master_write will not expect an ack after each byte
    while ( NumByteToWrite-- > 0 )
    {
        ESP_ERROR_CHECK( i2c_master_write_byte( I2C_Master->cmd_handle, *pByteBuffer++, ACK_CHECK_EN ) );
    }
    return byte_index;
}

uint8_t i2c_read_byte()
{
    uint8_t byteBuffer = 0;
    ESP_ERROR_CHECK( i2c_master_read_byte( I2C_Master->cmd_handle, &byteBuffer, I2C_MASTER_LAST_NACK ) );
    return byteBuffer;
}

size_t i2c_read( uint8_t *pByteBuffer, size_t NumByteToRead )
{
    ESP_ERROR_CHECK( i2c_master_read( I2C_Master->cmd_handle, pByteBuffer, NumByteToRead, I2C_MASTER_LAST_NACK ) );
    return NumByteToRead;
}

esp_err_t i2c_transmit()
{
    ESP_ERROR_CHECK( i2c_master_stop( I2C_Master->cmd_handle ) );
    esp_err_t i2c_error = i2c_master_cmd_begin(I2C_Master->port, I2C_Master->cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete( I2C_Master->cmd_handle );
    I2C_Master->dev_address = I2C_NO_DEVICE;
    I2C_Master->cmd_handle = (i2c_cmd_handle_t) NULL;
    return i2c_error;
}


#include "ADS1298.h"
#include <nrfx_spim.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define POLL_DREADY

#ifndef POLL_DREADY
    #include <nrfx_gpiote.h>
#endif


volatile enum state_t {idle, send_command, receive_data, read_data, processing_data} state = idle;
bool new_data_available = false;

static const nrfx_spim_t m_spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);
volatile uint8_t rx_buffer[27], tx_buffer[8];
uint8_t sample_number = 0;

const struct device *gpio_dev_0;

inline int32_t buffer_to_channel(uint8_t id);
bool ADS1298_receive_data();
ADS1298_data_t ADS1298_convert_data();

#ifndef POLL_DREADY
    void data_ready_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
#endif

void spim_event_handler(nrfx_spim_evt_t const *p_event, void *p_context) 
{
    switch (p_event->type)
    {
    case NRFX_SPIM_EVENT_DONE:
        if(state == receive_data) {
            new_data_available = true;
        } 
        state = idle;
        break;
    default:
        break;
    }
    return;
}

#ifndef POLL_DREADY
void data_ready_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    if(pin == PIN_DRDY && action == NRF_GPIOTE_POLARITY_HITOLO) {
        ADS1298_receive_data();
    }
}
#endif
#define APP_SPIM_CS_PIN (2)
#define APP_SPIM_SCK_PIN (8)
#define APP_SPIM_MISO_PIN (24)
#define APP_SPIM_MOSI_PIN (9)
bool ADS1298_init() {
    nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(APP_SPIM_SCK_PIN, APP_SPIM_MOSI_PIN, APP_SPIM_MISO_PIN, NRFX_SPIM_PIN_NOT_USED);
    config.frequency = NRF_SPIM_FREQ_2M;
    config.ss_active_high = false;
    config.bit_order = SPIM_CONFIG_ORDER_MsbFirst;
    config.mode = NRF_SPIM_MODE_1;
    
    //nrfx_spim_uninit(&m_spi);
    int err = nrfx_spim_init(&m_spi, &config, spim_event_handler, NULL);

     if(err == NRFX_SUCCESS) {
    } else {
        return false;
    }
    
    gpio_dev_0 = device_get_binding("GPIO_0");

    if (gpio_dev_0 == NULL) {
		return false;
	}

    #ifdef POLL_DREADY
        err = gpio_pin_configure(gpio_dev_0, 7, GPIO_INPUT);
    #else
        // Interrupt
        if(!nrfx_gpiote_is_init()) {
            err = nrfx_gpiote_init(6);

            if(err == NRFX_SUCCESS) {
            } else {
                return false;
            }
        }

        nrfx_gpiote_in_config_t irq_config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(false ); // TODO: what is argument?
        err = nrfx_gpiote_in_init(PIN_DRDY, &irq_config, data_ready_handler);

        if(err == NRFX_SUCCESS) {
        } else {
            return false;
        }
    #endif

    #ifndef POLL_DREADY
        nrfx_gpiote_in_event_enable(PIN_DRDY, true);
    #endif

    return true; 
}

void poll_dready() {
    if(gpio_pin_get(gpio_dev_0, 7) == 0) {
        ADS1298_receive_data();
    }
}

int spi_transfer(const nrfx_spim_xfer_desc_t *desc) {
    return nrfx_spim_xfer(&m_spi, desc, 0);
}

bool ADS1298_send_command(uint8_t command) {
    if(state == idle) {
        nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TX(tx_buffer, 1);
        spim_xfer_desc.p_rx_buffer = NULL;
        spim_xfer_desc.rx_length = 0;
        tx_buffer[0] = command;
        state = send_command;

        int err = spi_transfer(&spim_xfer_desc);
        if(err != NRFX_SUCCESS) {
        }
        return true;
    } else {
        return false;
    }
}

/* function de read bloquant */
uint8_t ADS1298_read_data() {
    if(state == idle) {
        state = read_data;
        new_data_available = false;
        nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_RX(rx_buffer, 1);
        spi_transfer(&spim_xfer_desc);
        while(state != idle)
        {
          __WFE();
        }
        return rx_buffer[0];
    } else {
        return ;
    }
}

bool ADS1298_receive_data() {
    if(state == idle) {
        state = receive_data;
        new_data_available = false;
        nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_RX(rx_buffer, 27);
        spi_transfer(&spim_xfer_desc);
        return true;
    } else {
        return false;
    }
}

bool ADS1298_poll_new_data(ADS1298_data_t *data) {
    if(new_data_available) {
        new_data_available = false;
        state = processing_data;

        *data = ADS1298_convert_data();
       
        state = idle;
        return true;
    } else {
        return false;
    }
}

ADS1298_data_t ADS1298_convert_data() {
    /* Data structure: */
    /* 24 bit status */
    /*  - 1100 + LOFF_STATP(8 bit) + LOFF_STATN(8 bit) + GPIO(4 bit) */
    /* 8ch * 24 bits data */

    ADS1298_data_t data;
    
    data.lead_off_positive = rx_buffer[0] << 4;
    data.lead_off_positive |= (rx_buffer[1] & 0b11110000) >> 4;
    data.lead_off_negative = rx_buffer[1] << 4;
    data.lead_off_negative |= (rx_buffer[2] & 0b11110000) >> 4;

    data.gpio_0 = rx_buffer[2] & 0b00001000;
    data.gpio_1 = rx_buffer[2] & 0b00000100;
    data.gpio_2 = rx_buffer[2] & 0b00000010;
    data.gpio_3 = rx_buffer[2] & 0b00000001;

    data.channgel_0 = buffer_to_channel(0);
    data.channgel_1 = buffer_to_channel(1);
    data.channgel_2 = buffer_to_channel(2);
    data.channgel_3 = buffer_to_channel(3);
    data.channgel_4 = buffer_to_channel(4);
    data.channgel_5 = buffer_to_channel(5);
    data.channgel_6 = buffer_to_channel(6);
    data.channgel_7 = buffer_to_channel(7);

    return data;
}

inline int32_t buffer_to_channel(uint8_t id) {
    return rx_buffer[3 + id] << 16 | (rx_buffer[3 + id + 1] << 8) | rx_buffer[3 + id + 2];
}

bool get_spi_busy() {
    return state != idle;
}



uint8_t ads1298_read_ID()             
{
                ADS1298_send_command(0x20);// OPCODE1 = 0x20 = 0b0010 0000 : 001 = commande READ RREG |  0 0000 = Adresse du registe ID 
                k_usleep(64);
		ADS1298_send_command(0x00); // OPCODE2 = number of register -1 = 0= 0x00
                k_usleep(64);
	        return ADS1298_read_data();  
}


bool ads1298_write_command(uint8_t val)
{	
	ADS1298_send_command(val);
	return true;	
}




//uint8_t ads1298_read_register(uint8_t reg)
//{
//		uint8_t val = 0;
//		ads1298_read_multiple_register(reg, &val, 1);
//		return val;
//}

//bool ads1298_read_multiple_register(uint8_t reg, uint8_t* val, uint8_t num)
//{
//		ADS1298_send_command(COMMAND_RREG | (reg & 0x1F));
//                k_usleep(64);
//		ADS1298_send_command((num - 1) & 0x1F);
//                k_usleep(64);
//		for(int i = 0; i < num; i++)
//			*(val + i) = ADS1298_read_data();
//    return true;
		
//}


bool ads1298_write_register(uint8_t reg, uint8_t val)
{
    ads1298_write_multiple_register(reg, &val, 1);
    return true;
}

bool ads1298_write_multiple_register(uint8_t reg, uint8_t* val, uint8_t num)
  {	
		ADS1298_send_command(ADS129X_CMD_WREG | (reg & 0x1F));
                k_usleep(64);
		ADS1298_send_command((num - 1) & 0x1F);
                k_usleep(64);
	
		for(int i = 0; i < num; i++)
                    {
                      ADS1298_send_command(*(val + i));
                      k_usleep(64);
                    }

		return true;	
  }
  
bool ADS1298_send_start() {
    return ADS1298_send_command(COMMAND_START);
}
bool ADS1298_send_stop() {
    return ADS1298_send_command(COMMAND_STOP);
}
bool ADS1298_send_read_continuous() {
    return ADS1298_send_command(COMMAND_RDATAC);
}

bool ADS1298_send_read_data() {
    return ADS1298_send_command(COMMAND_RDATA);
}

bool ADS1298_send_stop_read_continuous() {
    return ADS1298_send_command(COMMAND_SDATAC);
}

bool ADS1298_send_reset() {
    return ADS1298_send_command(COMMAND_RESET);
}

bool ADS1298_send_wakeup() {
    return ADS1298_send_command(COMMAND_WAKEUP);
}

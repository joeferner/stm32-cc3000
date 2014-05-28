
extern unsigned char wlan_tx_buffer[];

typedef void (*spi_rx_handler_t)(void *p);

void SpiOpen(spi_rx_handler_t rx_handler);
void SpiClose();
void SpiResumeSpi();
long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength);
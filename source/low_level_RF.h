#ifndef LOW_LEVEL_RF_H
#define LOW_LEVEL_RF_H


void spi_write(char addr, char val);
int spi_read(char addr);
void RF_RST_Set(int state);
void RF_SLP_TR_Set(int state);
void RF_CS_while_active(void);
void RF_CS_Set(int state);
int spi_exchange(char data);
void RF_IRQ_Init(void);

#endif  // LOW_LEVEL_RF_H

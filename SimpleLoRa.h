#ifndef __NB_SIMPLELORA
#define __NB_SIMPLELORA

#include "nworkbench.h"
#include "SX1272_Lib/sx127x_lora.h"

#define RADIO_MODE_TX_ONLY 2
#define RADIO_MODE_RX_ONLY 1
#define RADIO_MODE_BIDIR   0


class nBlock_SimpleLoRa: public nBlockSimpleNode<1> {
public:
    nBlock_SimpleLoRa(
        bool cmwx1zzabz,        
        float frequency, 
        uint8_t preamblemsb, 
        uint8_t preamblelsb, 
        uint8_t  codingrate,
        uint8_t spreadingfactor, 
        float bandwidth, 
        bool headermode, 
        bool crcon, 
        uint8_t payloadlength,
        PinName mosi, 
        PinName miso, 
        PinName sck, 
        PinName cs, 
        PinName rst, 
        PinName dio0, 
        PinName dio1,
        uint8_t mode, 
        int power,
        PinName tcxo
        bool useleds,
        PinName ledtx,
        PinName ledrx,
        PinName ledtest
    );
    void triggerInput(nBlocks_Message message);
    void endFrame(void);
private:
    uint32_t _mode;

    SX127x_lora  _lora_select;
    SX127x       _board;
    DigitalOut _tcxo;
    DigitalOut _txled;
    DigitalOut _rxled;
    DigitalOut _testled;

    char _tx_buffer[32];
    char _rx_buffer[32];
    int  _tx_updated;
    uint8_t _payloadlength;
};

#endif

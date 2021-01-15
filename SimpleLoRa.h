#ifndef __NB_SIMPLELORA
#define __NB_SIMPLELORA

#include "nworkbench.h"
#include "sx127x_lora.h"

#define RADIO_MODE_TX_ONLY 2
#define RADIO_MODE_RX_ONLY 1
#define RADIO_MODE_BIDIR   0

// number of outputs is declared here: 2outputs <2>
class nBlock_SimpleLoRa: public nBlockSimpleNode<2> {
public:
    nBlock_SimpleLoRa(
        bool cmwx1zzabz,        
        float frequency, 
        uint8_t preamblemsb, 
        uint8_t preamblelsb, 
        uint8_t  codingrate,
        uint8_t spreadingfactor, 
        int bandwidth, 
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
        PowerTx power,
        PinName tcxo,
        bool useleds,
        PinName ledtx,
        PinName ledrx
    );
    void triggerInput(nBlocks_Message message);
    void endFrame(void);
    bool _cmwx1zzabz;
    bool _useleds;
    uint32_t framecounter;
    SX127x       _board;
    SX127x_lora  _lora_select;
    bool waitingTXDONE = false;

private:
    uint32_t _mode;



    DigitalOut _tcxo;
    DigitalOut _ledtx;
    DigitalOut _ledrx;

    char _tx_buffer[256];
    char _rx_buffer[256];
    int  _tx_updated;
    uint8_t _payloadlength;

};

#endif

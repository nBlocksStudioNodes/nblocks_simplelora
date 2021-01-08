#include "SimpleLoRa.h"

nBlock_SimpleLoRa::nBlock_SimpleLoRa (
        bool cmwx1zzabz,        
        float frequency, uint8_t preamblemsb, uint8_t preamblelsb, uint8_t codingrate,
        uint8_t spreadingfactor, float bandwidth, bool headermode, bool crcon, uint8_t payloadlength,
        PinName mosi, PinName miso, PinName sck, PinName cs, PinName rst, PinName dio0, PinName dio1,
        uint8_t mode, int power, PinName tcxo
        ): _board(mosi, miso, sck, cs, rst,dio0, dio1), _lora_select(_board) _tcxo(tcxo) {
    
    if(payloadlength < 33) _payloadlength = payloadlength;
    if(payloadlength > 32) _payloadlength = 32;             // Limit payloadlength for saving RAM space
    if (cmwx1zzabz) { _tcxo = 1; }                          // CMWX1ZZABZ will not start without this
    _board.hw_reset();                                      // Reset
    wait_us(100000);                                        // **** 100ms ****
    _board.init();                                          // Initialize TX
    _lora_select.enable();                                  // Initialize LoRa TX    
    
    _board.set_frf_MHz(frequency);                          // Set Carrier Frequency
    _board.write_reg(REG_LR_PREAMBLEMSB,preamblemsb);       // Preamble MSB 
    _board.write_reg(REG_LR_PREAMBLELSB,preamblelsb);       // Preamble LSB

    _lora_select.setCodingRate(codingrate);                 // Set Coding Rate to 
    _lora_select.setHeaderMode(headermode);                 // Set Explicit Header Mode
    _lora_select.setSf(spreadingfactor);                    // Set Spreading Factor 
    _lora_select.setBw_KHz(bandwidth);                      // Set BandWidth 
    _lora_select.setRxPayloadCrcOn(crcon);                  // Turn Off CRC
    _lora_select.setTxPower(power);                         // Set Power      
    _board.write_reg(REG_LR_PAYLOADLENGTH,payloadlength);   // Set Payload Length
    if (cmwx1zzabz) _tcxo = 0;                               // CMWX1ZZABZ Back to Low Power

    _mode = mode;
    _tx_updated = 0;
    
    outputType[0] = OUTPUT_TYPE_ARRAY;
    for (int i=0; i<32; i++) {
        _tx_buffer[i] = 0;
        _rx_buffer[i] = 0;
    }
}
void nBlock_SimpleLoRa::triggerInput(nBlocks_Message message) {
    // Ignore inputs if we are RX only
    if (_mode == RADIO_MODE_RX_ONLY) return;
    
    switch (message.dataType) {
        case OUTPUT_TYPE_INT:
        case OUTPUT_TYPE_FLOAT:
            break;

        case OUTPUT_TYPE_STRING:
            strcpy(_tx_buffer, message.stringValue);
            _tx_updated = 1;
            break;

        case OUTPUT_TYPE_ARRAY:
            char * data_array = (char *)(message.pointerValue);
            for (uint32_t i=0; i<message.dataLength; i++) {
                _tx_buffer[i] = data_array[i];
            }
            _tx_updated = 1;
            break;
            
    }
}
void nBlock_SimpleLoRa::endFrame(void) {
    // If we are a receiver, we check for incoming data first
    // so it is not lost by becoming transmitter
    if (_lora_select.service() == SERVICE_READ_FIFO) {
        //output[0] = (uint32_t)(&_rx_buffer);
        //available[0] = _config.data_length;
        
        //uiBuffer=_board.rx_buf[0];                        // size is rx_buf[256] 
        //packet_rssi = lora_select.get_pkt_rssi();         // testing how to get RSSI...
        _tcxo = 1;
        output[1] = _lora_select.get_pkt_rssi();            // rssi to a separate output(3)            [SimpleSerial](3)
        output[0] = (uint32_t)(&_board.rx_buf);             // the payload to the first Node output(2) [SimpleSerial](2)
        available[0] = _payloadlength;                      // Is there a better way? what if the other side is using different payloadlength?
        available[1] = 1;        
    }
    
    // If we have to transmit, the buffer is currently holding the data
    if (_tx_updated) {
        _tx_updated = 0;
        //_radio.Transmit(_tx_buffer);
        _tcxo = 1;
        _board.tx_buf = _tx_buffer;
        _lora_select.start_tx(_payloadlength);
        wait_us(2000);           
        if (lora_select.service() == SERVICE_TX_DONE){      // non blocking
            led_tx = !led_tx; 
            nmode = 2;
        } 
    }
    

    return;
}


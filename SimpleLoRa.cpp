#include "SimpleLoRa.h"

nBlock_SimpleLoRa::nBlock_SimpleLoRa (
        bool cmwx1zzabz,        
        float frequency, uint8_t preamblemsb, uint8_t preamblelsb, uint8_t codingrate,
        uint8_t spreadingfactor, float bandwidth, bool headermode, bool crcon, uint8_t payloadlength,
        PinName mosi, PinName miso, PinName sck, PinName cs, PinName rst, PinName dio0, PinName dio1,
        uint8_t mode, PowerTx power, PinName tcxo,
        bool useleds, PinName txled, PinName rxled, PinName testled
        ):  _lora_select(_board), _board(mosi, miso, sck, cs, rst,dio0, dio1), _tcxo(tcxo), _txled(txled), _rxled(rxled), _testled(testled) {
    /*
    if (useleds){
        _txled(txled);
        _rxled(rxled);
        _testled(testled);
    }*/
    _cmwx1zzabz = cmwx1zzabz;
    _useleds = useleds;
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
    if (cmwx1zzabz) _tcxo = 0;                              // CMWX1ZZABZ Back to Low Power

    _mode = mode;
    _tx_updated = 0;
    
    outputType[0] = OUTPUT_TYPE_ARRAY;
    for (int i=0; i<32; i++) {
        _tx_buffer[i] = 0;
        _rx_buffer[i] = 0;
    }
}

void nBlock_SimpleLoRa::triggerInput(nBlocks_Message message) {   
    if (_mode == RADIO_MODE_RX_ONLY) return;                        // Ignore inputs if we are RX only
    switch (message.dataType) {
        case OUTPUT_TYPE_INT:
        case OUTPUT_TYPE_FLOAT:
            break;
        case OUTPUT_TYPE_STRING:
            strcpy(_tx_buffer, message.stringValue);                // -------FILL THE THE _tx_buffer with the string, 
            for (uint32_t i=0; i<256; i++){
                _board.tx_buf[i] = _tx_buffer[i];                   // then  copy to lora _board.tx_buf
            }
            _tx_updated = 1;
            break;
        case OUTPUT_TYPE_ARRAY:
            char * data_array = (char *)(message.pointerValue);
            for (uint32_t i=0; i<message.dataLength; i++) {
               // _tx_buffer[i] = data_array[i];                    //
               _board.tx_buf[i] = data_array[i];                    // -----FILL THE THE lora TX BUFFER
            }
            _tx_updated = 1;
            break;          
    }
}

void nBlock_SimpleLoRa::endFrame(void) {
    if (_cmwx1zzabz) _tcxo = 1;
    if (_useleds) { _testled = !_testled; }                         // we have endFrame
    if (_mode != RADIO_MODE_TX_ONLY){
        if (_lora_select.service() == SERVICE_READ_FIFO) {     
            if (_useleds) { _rxled = !_rxled; }                     // we have RX
            output[1] = _lora_select.get_pkt_rssi();                // rssi to a the 2nd Node output           [SimpleSerial](3)
            output[0] = (uint32_t)(&_board.rx_buf);                 // the payload to the first Node output    [SimpleSerial](2)
            available[0] = _payloadlength; 
            available[1] = 1;        
        }
    }

    if (_mode != RADIO_MODE_RX_ONLY){    
        if (_tx_updated) {
            _tx_updated = 0;
            //_board.tx_buf = _tx_buffer;                           // _radio.Transmit(_tx_buffer);
            _lora_select.start_tx(_payloadlength);
            wait_us(2000);           
            if (_lora_select.service() == SERVICE_TX_DONE){         // we have TX
                if (_useleds) { _txled = !_txled; }
            } 
        }
    }

    return;
}


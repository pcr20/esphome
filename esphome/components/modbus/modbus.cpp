#include "modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace modbus {

static const char *const TAG = "modbus";

void Modbus::setup() {
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }
}
void Modbus::loop() {
  const uint32_t now = millis();

  if (now - this->last_modbus_byte_ > 50) {
    this->rx_buffer_.clear();
    this->last_modbus_byte_ = now;
  }
  // stop blocking new send commands after send_wait_time_ ms regardless if a response has been received since then
  if (now - this->last_send_ > send_wait_time_) {
    waiting_for_response = 0;
  }

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    if (this->parse_modbus_byte_(byte)) {
      this->last_modbus_byte_ = now;
    } else {
      this->rx_buffer_.clear();
    }
  }
}

bool Modbus::parse_modbus_byte_(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *raw = &this->rx_buffer_[0];
  ESP_LOGV(TAG, "Modbus received Byte  %d (0X%x)", byte, byte);
  // Byte 0: modbus address (match all)
  if (at == 0)
    return true;
  uint8_t address = raw[0];
  uint8_t function_code = raw[1];
  // Byte 2: Size (with modbus rtu function code 4/3)
  // See also https://en.wikipedia.org/wiki/Modbus
  if (at == 2)
    return true;

//modbus response: [addr,f_code,bytes, data[bytes],...,CRC1,CRC2] total length = bytes + 5
//modbus command: [addr,f_code,reg_addr1,reg_addr2,num_reg1,num_reg2,CRC1,CRC2] total length = 3 + 5

  //guess it's a response
  uint8_t data_len = raw[2];
  uint8_t data_offset = 3;
  bool is_response=true;
  uint16_t start_reg=0;
  uint16_t num_regs=0;

  // Per https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf Ch 5 User-Defined function codes
  if (((function_code >= 65) && (function_code <= 72)) || ((function_code >= 100) && (function_code <= 110))) {
    // Handle user-defined function, since we don't know how big this ought to be,
    // ideally we should delegate the entire length detection to whatever handler is
    // installed, but wait, there is the CRC, and if we get a hit there is a good
    // chance that this is a complete message ... admittedly there is a small chance is
    // isn't but that is quite small given the purpose of the CRC in the first place

    // Fewer than 2 bytes can't calc CRC
    if (at < 2)
      return true;

    data_len = at - 2;
    data_offset = 1;

    uint16_t computed_crc = crc16(raw, data_offset + data_len);
    uint16_t remote_crc = uint16_t(raw[data_offset + data_len]) | (uint16_t(raw[data_offset + data_len + 1]) << 8);

    if (computed_crc != remote_crc)
      return true;

    ESP_LOGD(TAG, "Modbus user-defined function %02X found", function_code);

  } else {
    uint16_t computed_crc;
    uint16_t remote_crc;
    uint8_t runs=0;
    bool crc_ok=false;
    while (runs<2)
    {
        if (function_code == 0x3 || function_code == 0x4)
        {
          if (is_response)
          {
          //read register response
          // data starts at offset 3 and length is variable, in the 3rd byte
            data_len = raw[2];
            data_offset = 3;
          }
          else
          {
          //read register commands
          // data starts at offset 2 and length is 4 for read registers commands
          data_offset = 2;
          data_len = 4;       
          start_reg= uint16_t(raw[3]) | (uint16_t(raw[2]) << 8);
          num_regs= uint16_t(raw[5]) | (uint16_t(raw[4]) << 8);     
          }
        }
        else if ((function_code == 0x5 || function_code == 0x06 || function_code == 0xF || function_code == 0x10))
        {
          if (is_response)
          {
          // the response for write command mirrors the requests and data starts at offset 2 instead of 3 for read commands
           data_offset = 2;
           data_len = 4;
          }
          else
          {
           // write commands have data starting at offset 7 but size is variable.
           data_offset = 7;
           data_len = raw[6];
           start_reg= uint16_t(raw[3]) | (uint16_t(raw[2]) << 8);
           num_regs= uint16_t(raw[5]) | (uint16_t(raw[4]) << 8);   
          }
        }
        else if ((function_code & 0x80) == 0x80)
        {
        // Error ( msb indicates error )
        // response format:  Byte[0] = device address, Byte[1] function code | 0x80 , Byte[2] exception code, Byte[3-4] crc
          data_offset = 2;
          data_len = 1;
        }
        else
        {
            ESP_LOGW(TAG, "Unknown function code %02X", function_code);
            return false;
        }
        
        //check CRC
        computed_crc=crc16(raw, data_offset + data_len);
        remote_crc=remote_crc = uint16_t(raw[data_offset + data_len]) | (uint16_t(raw[data_offset + data_len + 1]) << 8);
        if (computed_crc == remote_crc)
        {
        //we've guessed correctly command or response
            crc_ok=true;
            break;
        }
        else
        {
            runs++;
            is_response= not is_response;
            crc_ok=false;
        }
    }
 

    // wait until the buffer has filled up
    
    // Byte data_offset..data_offset+data_len-1: Data
    if (at < data_offset + data_len)
      return true;

    // Byte 3+data_len: CRC_LO (over all bytes)
    if (at == data_offset + data_len)
      return true;

    ESP_LOGV(TAG, "Found function 0x%02x is_response %d crc_ok %d runs %d", function_code, is_response,crc_ok,runs);
   
      
    // Byte data_offset+len+1: CRC_HI (over all bytes)
    //computed_crc = crc16(raw, data_offset + data_len);
    //remote_crc = uint16_t(raw[data_offset + data_len]) | (uint16_t(raw[data_offset + data_len + 1]) << 8);
    if (computed_crc != remote_crc) {
      if (this->disable_crc_) {
        ESP_LOGD(TAG, "Modbus CRC Check failed, but ignored! %02X!=%02X", computed_crc, remote_crc);
      } else {
        ESP_LOGW(TAG, "Modbus CRC Check failed! %02X!=%02X", computed_crc, remote_crc);
        return false;
      }
    }
  }
  std::vector<uint8_t> data(this->rx_buffer_.begin() + data_offset, this->rx_buffer_.begin() + data_offset + data_len);
  bool found = false;
  for (auto *device : this->devices_) {
    if (device->address_ == address) {
      ESP_LOGV(TAG, "Matched addr 0X%x", address);
      // Is it an error response?
      if ((function_code & 0x80) == 0x80) {
        ESP_LOGD(TAG, "Modbus error function code: 0x%X exception: %d", function_code, raw[2]);
        if (waiting_for_response != 0) {
          device->on_modbus_error(function_code & 0x7F, raw[2]);
        } else {
          // Ignore modbus exception not related to a pending command
          ESP_LOGD(TAG, "Ignoring Modbus error - not expecting a response");
        }
      } else if (this->role == ModbusRole::SERVER)
        {
            if (function_code == 0x3 || function_code == 0x4) {
                device->on_modbus_read_registers(function_code,start_reg,num_regs);
                //void ModbusController::on_modbus_read_registers(uint8_t function_code, uint16_t start_address,uint16_t number_of_registers)
           
            }
            else if (function_code == 0x10)
            {
                device->on_modbus_write_registers(function_code,start_reg,num_regs,data);
            }
        }
        else {
        device->on_modbus_data(data);
      }
      found = true;
    }
  }
  waiting_for_response = 0;

  if (!found) {
    ESP_LOGW(TAG, "Got Modbus frame from unknown address 0x%02X! ", address);
  }

  // return false to reset buffer
  return false;
}

void Modbus::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  Send Wait Time: %d ms", this->send_wait_time_);
  ESP_LOGCONFIG(TAG, "  CRC Disabled: %s", YESNO(this->disable_crc_));
}
float Modbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void Modbus::send(uint8_t address, uint8_t function_code, uint16_t start_address, uint16_t number_of_entities,
                  uint8_t payload_len, const uint8_t *payload,bool disable_send) {
  static const size_t MAX_VALUES = 128;

  // Only check max number of registers for standard function codes
  // Some devices use non standard codes like 0x43
  if (number_of_entities > MAX_VALUES && function_code <= 0x10) {
    ESP_LOGE(TAG, "send too many values %d max=%zu", number_of_entities, MAX_VALUES);
    return;
  }

  std::vector<uint8_t> data;
  data.push_back(address);
  data.push_back(function_code);
  if (this->role == ModbusRole::CLIENT) {
    data.push_back(start_address >> 8);
    data.push_back(start_address >> 0);
    if (function_code != 0x5 && function_code != 0x6) {
      data.push_back(number_of_entities >> 8);
      data.push_back(number_of_entities >> 0);
    }
  }
  else
  { //this->role == ModbusRole::SERVER
    if (function_code == 0x10)
    {
    data.push_back(start_address >> 8);
    data.push_back(start_address >> 0);    
      data.push_back(number_of_entities >> 8);
      data.push_back(number_of_entities >> 0);    
    }
  }

  if (payload != nullptr) {
    if (this->role == ModbusRole::SERVER || function_code == 0xF || function_code == 0x10) {  // Write multiple
      data.push_back(payload_len);  // Byte count is required for write
    } else {
      payload_len = 2;  // Write single register or coil
    }
    for (int i = 0; i < payload_len; i++) {
      data.push_back(payload[i]);
    }
  }

  auto crc = crc16(data.data(), data.size());
  data.push_back(crc >> 0);
  data.push_back(crc >> 8);

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->write_array(data);
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
  waiting_for_response = address;
  last_send_ = millis();
  ESP_LOGV(TAG, "Modbus write: %s", format_hex_pretty(data).c_str());
}

// Helper function for lambdas
// Send raw command. Except CRC everything must be contained in payload
void Modbus::send_raw(const std::vector<uint8_t> &payload,bool disable_send) {
  if (payload.empty()) {
    return;
  }

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  auto crc = crc16(payload.data(), payload.size());
  if (not disable_send)
  {
  this->write_array(payload);
  this->write_byte(crc & 0xFF);
  this->write_byte((crc >> 8) & 0xFF);
  this->flush();
  }
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
  waiting_for_response = payload[0];
  ESP_LOGV(TAG, "Modbus write raw: %s", format_hex_pretty(payload).c_str());
  last_send_ = millis();
}

}  // namespace modbus
}  // namespace esphome

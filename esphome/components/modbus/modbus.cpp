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
  const int now_available= this->available();
static uint32_t exec_times[256]; //will initialise to zero
static int uart_availables[256]; //will initialise to zero
static int exec_times_counter=0;
static uint32_t last_now=0;
static int last_available=0;
static int sum_availables=0;
static uint32_t sum_exec_times=0;
static int max_availables=0;
static uint32_t max_exec_times=0;
uint32_t temp=exec_times[exec_times_counter]; //oldest member of exec_times
int temp2=uart_availables[exec_times_counter]; //oldest member of uart_availables
exec_times_counter++;
if (exec_times_counter==256) 
{
  exec_times_counter = 0;

  ESP_LOGI(TAG, "av: %fms max: %04dms size av: %f max: %04d",((float)sum_exec_times)/256,max_exec_times,((float)sum_availables)/256,max_availables);
  max_exec_times=0; //reset max tracker
  max_availables=0;
}
exec_times[exec_times_counter]=now-last_now;
last_now=now;
uart_availables[exec_times_counter]=now_available-last_available;
last_available=now_available;
sum_exec_times=sum_exec_times+exec_times[exec_times_counter]-temp; //previous sum + new time - oldest time
if (exec_times[exec_times_counter]>max_exec_times)  max_exec_times=exec_times[exec_times_counter];
sum_availables=sum_availables+uart_availables[exec_times_counter]-temp2; //previous sum + new time - oldest time
if (uart_availables[exec_times_counter]>max_availables)  max_availables=uart_availables[exec_times_counter];




  if (now - this->last_modbus_byte_ > 50) {
    this->rx_buffer_.clear();
    this->last_modbus_byte_ = now;
  }
  // stop blocking new send commands after send_wait_time_ ms regardless if a response has been received since then
  if (now - this->last_send_ > send_wait_time_) {
    waiting_for_response = 0;
  }
 uint32_t start,end,us_max,start_t,end_t;
 us_max=0;
 bool result;
 start_t=micros();
 this->last_modbus_byte_=now;
  while (this->available()  && ((now - this->last_modbus_byte_)<10)) {
    uint8_t byte;
    this->read_byte(&byte);
    start=micros();
    result=this->parse_modbus_byte_(byte);
    end=micros();
    if ((end-start)>us_max) us_max=(end-start);
    if (result) {
      this->last_modbus_byte_ = now;
    } else {
      this->rx_buffer_.clear();
    }
  }
   end_t=micros();
  if ((us_max>0)||(end_t-start_t>1000)) ESP_LOGD(TAG, "max %d total %d", us_max,end_t-start_t);
}

bool Modbus::parse_modbus_byte_(uint8_t byte) {
  static const size_t MAX_MESSAGE_SIZE = 512;
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
  if (at <= 2)
    return true;

//modbus response: [addr,f_code,bytes, data[bytes],...,CRC1,CRC2] total length = bytes + 5
//modbus command: [addr,f_code,reg_addr1,reg_addr2,num_reg1,num_reg2,CRC1,CRC2] total length = 3 + 5



  frame_type_enum frame_type=no_frame;
  unsigned int data_len[6];
  unsigned int data_offset[6];
  bool is_response[6];
  data_len[response_custom]=at - 2;
  data_offset[response_custom]=1;
  is_response[response_custom]=true;
  data_len[error_80]=1;
  data_offset[error_80]=2;
  is_response[error_80]=true;
  data_len[command_0304]=4;
  data_offset[command_0304]=2;
  is_response[command_0304]=false;
  data_len[response_05060F10]=4;
  data_offset[response_05060F10]=2;
  is_response[response_05060F10]=true;
  data_len[response_0304]=raw[2];
  data_offset[response_0304]=3;
  is_response[response_0304]=true;
  data_len[command_05060F10]=raw[6];
  data_offset[command_05060F10]=7;
  is_response[command_05060F10]=false;


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
      frame_type = response_custom;
      uint16_t computed_crc = crc16(raw, data_offset[frame_type] + data_len[frame_type]);
      uint16_t remote_crc = uint16_t(raw[data_offset[frame_type] + data_len[frame_type]]) | (uint16_t(raw[data_offset[frame_type] + data_len[frame_type] + 1]) << 8);

      if (computed_crc != remote_crc)
        return true;

      ESP_LOGD(TAG, "Modbus user-defined function %02X found", function_code);

    } else  if ((function_code & 0x80) == 0x80)
    {
    // Error ( msb indicates error )
    // response format:  Byte[0] = device address, Byte[1] function code | 0x80 , Byte[2] exception code, Byte[3-4] crc
      frame_type = error_80;
    }
    else if ((function_code == 0x3 || function_code == 0x4))
    {
        //check for command
        if ((at > data_offset[command_0304]+data_len[command_0304])) //read commands are 8 bytes
        {
            uint16_t computed_crc = crc16(raw, data_offset[command_0304] + data_len[command_0304]);
            uint16_t remote_crc = uint16_t(raw[data_offset[command_0304] + data_len[command_0304]]) | (uint16_t(raw[data_offset[command_0304] + data_len[command_0304] + 1]) << 8);
            if (computed_crc == remote_crc)
            {
                frame_type = command_0304;
            }
        }
        //check for response
        if ((at > data_offset[response_0304]+data_len[response_0304]) && (frame_type==no_frame))
        {
            uint16_t computed_crc = crc16(raw, data_offset[response_0304] + data_len[response_0304]);
            uint16_t remote_crc = uint16_t(raw[data_offset[response_0304] + data_len[response_0304]]) | (uint16_t(raw[data_offset[response_0304] + data_len[response_0304] + 1]) << 8);
            if (computed_crc == remote_crc)
            {
                frame_type = response_0304;
            }
        }

        if ((at<MAX_MESSAGE_SIZE)  && (frame_type==no_frame))
        {
                return true; //not enough bytes
        }
    }
    else if ((function_code == 0x5 || function_code == 0x06 || function_code == 0xF || function_code == 0x10))
    {
        //check for command
        if (at > data_offset[command_05060F10]+data_len[command_05060F10])
        {
            uint16_t computed_crc = crc16(raw, data_offset[command_05060F10] + data_len[command_05060F10]);
            uint16_t remote_crc = uint16_t(raw[data_offset[command_05060F10] + data_len[command_05060F10]]) | (uint16_t(raw[data_offset[command_05060F10] + data_len[command_05060F10] + 1]) << 8);
            if (computed_crc == remote_crc)
            {
                frame_type = command_05060F10;
            }
        }
        //check for response
        if ((at > data_offset[response_05060F10]+data_len[response_05060F10]) && (frame_type==no_frame)) //write responses are 8 bytes)
        {
            uint16_t computed_crc = crc16(raw, data_offset[response_05060F10] + data_len[response_05060F10]);
            uint16_t remote_crc = uint16_t(raw[data_offset[response_05060F10] + data_len[response_05060F10]]) | (uint16_t(raw[data_offset[response_05060F10] + data_len[response_05060F10] + 1]) << 8);
            if (computed_crc == remote_crc)
            {
                frame_type = response_05060F10;
            }
        }

        if ((at<MAX_MESSAGE_SIZE)  && (frame_type==no_frame))
        {
                return true; //not enough bytes
        }

    }
    else
    {
        ESP_LOGW(TAG, "Unknown function code %02X", function_code);
        return false;
    }

    uint16_t computed_crc = crc16(raw, data_offset[frame_type] + data_len[frame_type]);
    uint16_t remote_crc = uint16_t(raw[data_offset[frame_type] + data_len[frame_type]]) | (uint16_t(raw[data_offset[frame_type] + data_len[frame_type] + 1]) << 8);

    if (computed_crc != remote_crc) {
          if (this->disable_crc_) {
            ESP_LOGD(TAG, "Modbus CRC Check failed, but ignored! %02X!=%02X", computed_crc, remote_crc);
          } else {
            ESP_LOGW(TAG, "Modbus CRC Check failed! %02X!=%02X", computed_crc, remote_crc);
            return false;
          }
    }


  uint16_t start_reg= uint16_t(raw[3]) | (uint16_t(raw[2]) << 8);
  uint16_t num_regs= uint16_t(raw[5]) | (uint16_t(raw[4]) << 8);

  std::vector<uint8_t> data(this->rx_buffer_.begin() + data_offset[frame_type], this->rx_buffer_.begin() + data_offset[frame_type] + data_len[frame_type]);
    ESP_LOGD(TAG, "Found addr: 0x%02x function 0x%02x frame_type %d start_reg %x num_regs %d data size %d",address, function_code, frame_type,start_reg,num_regs,data.size());

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
      else
      {
          device->on_modbus_data(is_response[frame_type],address,function_code,start_reg,num_regs,remote_crc,data);
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
  //this->flush();

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
  //this->flush();
  }
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
  waiting_for_response = payload[0];
  ESP_LOGV(TAG, "Modbus write raw: %s", format_hex_pretty(payload).c_str());
  last_send_ = millis();
}

}  // namespace modbus
}  // namespace esphome

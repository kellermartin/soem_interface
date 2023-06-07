/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Markus Staeuble, Jonas Junger, Johannes Pankert, Philipp Leemann,
** Tom Lankhorst, Samuel Bachmann, Gabriel Hottiger, Lennert Nachtigall,
** Mario Mauerer, Remo Diethelm
**
** This file is part of the soem_interface.
**
** The soem_interface is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The seom_interface is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the soem_interface.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <sstream>
#include <iomanip>

#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>

namespace soem_interface {

EthercatBusBase::EthercatBusBase(const std::string& name) : name_(name), wkc_(0) {
  // Initialize all SOEM context data pointers that are not used with null.
  ecatContext_.elist->head = 0;
  ecatContext_.elist->tail = 0;
  ecatContext_.port->stack.sock = nullptr;
  ecatContext_.port->stack.txbuf = nullptr;
  ecatContext_.port->stack.txbuflength = nullptr;
  ecatContext_.port->stack.tempbuf = nullptr;
  ecatContext_.port->stack.rxbuf = nullptr;
  ecatContext_.port->stack.rxbufstat = nullptr;
  ecatContext_.port->stack.rxsa = nullptr;
  ecatContext_.port->redport = nullptr;
  //  ecatContext_.idxstack->data = nullptr; // This does not compile since SOEM uses a fixed size array of void pointers.
  ecatContext_.FOEhook = nullptr;
}

bool EthercatBusBase::busIsAvailable(const std::string& name) {
  ec_adaptert* adapter = ec_find_adapters();
  while (adapter != nullptr) {
    if (name == std::string(adapter->name)) {
      return true;
    }
    adapter = adapter->next;
  }
  return false;
}

void EthercatBusBase::printAvailableBusses() {
  spdlog::info("Available adapters:");
  ec_adaptert* adapter = ec_find_adapters();
  while (adapter != nullptr) {
    std::stringstream ss;
    ss << "- Name: '" << adapter->name << "', description: '" << adapter->desc << "'";
    spdlog::info(ss.str());
    adapter = adapter->next;
  }
}

bool EthercatBusBase::busIsAvailable() const { return busIsAvailable(name_); }

int EthercatBusBase::getNumberOfSlaves() const {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  return *ecatContext_.slavecount;
}

bool EthercatBusBase::addSlave(const EthercatSlaveBasePtr& slave) {
  for (const auto& existingSlave : slaves_) {
    if (slave->getAddress() == existingSlave->getAddress()) {
      std::stringstream ss;
      ss << "[" << getName() << "] "
                            << "Slave '" << existingSlave->getName() << "' and slave '" << slave->getName()
                            << "' have identical addresses (" << slave->getAddress() << ").";
      spdlog::error(ss.str());
      return false;
    }
  }

  slaves_.push_back(slave);
  return true;
}

bool EthercatBusBase::startup(const bool sizeCheck) {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  /*
   * Followed by start of the application we need to set up the NIC to be used as
   * EtherCAT Ethernet interface. In a simple setup we call ec_init(ifname) and if
   * SOEM comes with support for cable redundancy we call ec_init_redundant that
   * will open a second port as backup. You can send NULL as ifname if you have a
   * dedicated NIC selected in the nicdrv.c. It returns >0 if succeeded.
   */
  
  if (!busIsAvailable()) {
    std::stringstream logStr;
    logStr << "[" << getName() << "] "
                          << "Bus is not available.";
    spdlog::error(logStr.str());
    printAvailableBusses();
    return false;
  }
  if (ecx_init(&ecatContext_, name_.c_str()) <= 0) {
    std::stringstream logStr;
    logStr <<  "[" << getName() << "] "
                          << "No socket connection. Execute as root.";

    spdlog::error(logStr.str());
    return false;
  }

  // Initialize SOEM.
  // Note: ecx_config_init(..) requests the slaves to go to PRE-OP.
  for (unsigned int retry = 0; retry <= ecatConfigMaxRetries_; retry++) {
    if (ecx_config_init(&ecatContext_, FALSE) > 0) {
      // Successful initialization.
      break;
    } else if (retry == ecatConfigMaxRetries_) {
      // Too many failed attempts.
      std::stringstream logStr;
      logStr << "[" << getName() << "] "
                            << "No slaves have been found.";
      spdlog::error(logStr.str());
      return false;
    }
    // Sleep and retry.
    soem_interface::threadSleep(ecatConfigRetrySleep_);
    std::stringstream ss;
    ss << "No slaves have been found, retrying " << retry + 1 << "/" << ecatConfigMaxRetries_ << " ...";
    spdlog::info(ss.str());
  }

  // Print the slaves which have been detected.
  //MELO_INFO_STREAM("The following " << getNumberOfSlaves() << " slaves have been found and configured:");
  for (int slave = 1; slave <= getNumberOfSlaves(); slave++) {
    std::stringstream ss;
    ss << "Address: " << slave << " - Name: '" << std::string(ecatContext_.slavelist[slave].name) << "'";
    spdlog::info(ss.str());
  }

  // Check if the given slave addresses are valid.
  bool slaveAddressesAreOk = true;
  for (const auto& slave : slaves_) {
    auto address = static_cast<int>(slave->getAddress());
    if (address == 0) {
      std::stringstream ss;
      ss << "[" << getName() << "] "
                            << "Slave '" << slave->getName() << "': Invalid address " << address << ".";
      spdlog::error(ss.str());
      slaveAddressesAreOk = false;
    }
    if (address > getNumberOfSlaves()) {
      std::stringstream ss;
      ss << "[" << getName() << "] "
                            << "Slave '" << slave->getName() << "': Invalid address " << address << ", "
                            << "only " << getNumberOfSlaves() << " slave(s) found.";
      spdlog::error(ss.str());
      slaveAddressesAreOk = false;
    }
  }
  if (!slaveAddressesAreOk) {
    return false;
  }

  // Disable symmetrical transfers.
  ecatContext_.grouplist[0].blockLRW = 1;

  // Initialize the communication interfaces of all slaves.
  for (auto& slave : slaves_) {
    if (!slave->startup()) {
      std::stringstream ss;
      ss << "[" << getName() << "] "
                            << "Slave '" << slave->getName() << "' was not initialized successfully.";
      spdlog::error(ss.str());
      return false;
    }
  }

  // Set up the communication IO mapping.
  // Note: ecx_config_map_group(..) requests the slaves to go to SAFE-OP.
  ecx_config_map_group(&ecatContext_, &ioMap_, 0);

  // Check if the size of the IO mapping fits our slaves.
  bool ioMapIsOk = true;
  // do this check only if 'sizeCheck' is true
  if (sizeCheck) {
    for (const auto& slave : slaves_) {
      const EthercatSlaveBase::PdoInfo pdoInfo = slave->getCurrentPdoInfo();
      if (pdoInfo.rxPdoSize_ != ecatContext_.slavelist[slave->getAddress()].Obytes) {
        std::stringstream ss;
        ss << "[" << getName() << "] "
                              << "RxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of " << pdoInfo.rxPdoSize_
                              << " bytes but the slave found at its address " << slave->getAddress() << " requests "
                              << ecatContext_.slavelist[slave->getAddress()].Obytes << " bytes).";
        spdlog::error(ss.str());
        ioMapIsOk = false;
      }
      if (pdoInfo.txPdoSize_ != ecatContext_.slavelist[slave->getAddress()].Ibytes) {
        std::stringstream ss;
        ss << "[" << getName() << "] "
                              << "TxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of " << pdoInfo.txPdoSize_
                              << " bytes but the slave found at its address " << slave->getAddress() << " requests "
                              << ecatContext_.slavelist[slave->getAddress()].Ibytes << " bytes).";
        spdlog::error(ss.str());
        ioMapIsOk = false;
      }
    }
  }
  if (!ioMapIsOk) {
    return false;
  }

  // Initialize the memory with zeroes.
  for (int slave = 1; slave <= getNumberOfSlaves(); slave++) {
    memset(ecatContext_.slavelist[slave].inputs, 0, ecatContext_.slavelist[slave].Ibytes);
    memset(ecatContext_.slavelist[slave].outputs, 0, ecatContext_.slavelist[slave].Obytes);
  }

  workingCounterTooLowCounter_ = 0;
  initlialized_ = true;

  return true;
}

void EthercatBusBase::updateRead() {
  if (!sentProcessData_) {
    spdlog::debug("No process data to read.");
    return;
  }

  //! Receive the EtherCAT data.
  updateReadStamp_ = std::chrono::high_resolution_clock::now();
  {
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
  }
  sentProcessData_ = false;

  //! Check the working counter.
  if (!workingCounterIsOk()) {
    ++workingCounterTooLowCounter_;
    std::stringstream ss;
    if (!busIsOk()) {
      ss << 1.0 << "Bus is not ok. Too many working counter too low in a row: " << workingCounterTooLowCounter_;
      spdlog::warn(ss.str());
      ss.clear();
    }
    ss << "Working counter too low counter: " << workingCounterTooLowCounter_;
    ss << 1.0 << "Update Read:" << this;
    ss << 1.0 << "Working counter is too low: " << wkc_.load() << " < " << getExpectedWorkingCounter();
    spdlog::warn(ss.str());
    return;
  }
  // Reset working counter too low counter.
  workingCounterTooLowCounter_ = 0;

  //! Each slave attached to this bus reads its data to the buffer.
  for (auto& slave : slaves_) {
    slave->updateRead();
  }
}

void EthercatBusBase::updateWrite() {
  if (sentProcessData_) {
    spdlog::debug("Sending new process data without reading the previous one.");
  }

  //! Each slave attached to this bus write its data to the buffer.
  for (auto& slave : slaves_) {
    slave->updateWrite();
  }

  //! Send the EtherCAT data.
  updateWriteStamp_ = std::chrono::high_resolution_clock::now();
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  ecx_send_processdata(&ecatContext_);
  sentProcessData_ = true;
}

void EthercatBusBase::shutdown() {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  // Set the slaves to state Init.
  if (getNumberOfSlaves() > 0) {
    setState(EC_STATE_INIT);
    waitForState(EC_STATE_INIT);
  }

  for (auto& slave : slaves_) {
    slave->shutdown();
  }

  // Close the port.
  if (ecatContext_.port != nullptr) {
    spdlog::info("Closing socket ...");
    ecx_close(&ecatContext_);
    // Sleep to make sure the socket is closed, because ecx_close is non-blocking.
    soem_interface::threadSleep(0.5);
  }

  initlialized_ = false;
}

void EthercatBusBase::setState(const uint16_t state, const uint16_t slave) {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  if(!initlialized_) {
    std::stringstream ss;
    ss << "Bus " << name_ << " was not successfully initialized, skipping operation";
    spdlog::error(ss.str());
    return;
  }
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  ecatContext_.slavelist[slave].state = state;
  ecx_writestate(&ecatContext_, slave);
  std::stringstream ss;
  ss << "Slave " << slave << ": State " << state << " has been set.";
  spdlog::debug(ss.str());
}

bool EthercatBusBase::waitForState(const uint16_t state, const uint16_t slave, const unsigned int maxRetries, const double retrySleep) {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  for (unsigned int retry = 0; retry <= maxRetries; retry++) {
    if (ecx_statecheck(&ecatContext_, slave, state, static_cast<int>(1e6 * retrySleep)) == state) {
      std::stringstream ss;
      ss << "Slave " << slave << ": State " << state << " has been reached.";
      spdlog::debug(ss.str());
      return true;
    }
    // TODO: Do this for all states?
    ecx_send_processdata(&ecatContext_);
    wkc_ = ecx_receive_processdata(&ecatContext_, EC_TIMEOUTRET);
  }
  std::stringstream ss;
  ss << "Slave " << slave << ": State " << state << " has not been reached.";
  spdlog::warn(ss.str());
  return false;
}

int EthercatBusBase::getExpectedWorkingCounter(const uint16_t slave) const {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  return ecatContext_.grouplist[slave].outputsWKC * 2 + ecatContext_.grouplist[slave].inputsWKC;
}

std::string EthercatBusBase::getErrorString(ec_errort error) {
  std::stringstream stream;
  stream << "Time: " << (static_cast<double>(error.Time.sec) + (static_cast<double>(error.Time.usec) / 1000000.0));

  switch (error.Etype) {
    case EC_ERR_TYPE_SDO_ERROR:
      stream << " SDO slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.Index << "."
             << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(error.SubIdx) << " error: 0x" << std::setfill('0')
             << std::setw(8) << std::hex << static_cast<unsigned>(error.AbortCode) << " " << ec_sdoerror2string(error.AbortCode);
      break;
    case EC_ERR_TYPE_EMERGENCY:
      stream << " EMERGENCY slave: " << error.Slave << " error: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.ErrorCode;
      break;
    case EC_ERR_TYPE_PACKET_ERROR:
      stream << " PACKET slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.Index << "."
             << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(error.SubIdx) << " error: 0x" << std::setfill('0')
             << std::setw(8) << std::hex << error.ErrorCode;
      break;
    case EC_ERR_TYPE_SDOINFO_ERROR:
      stream << " SDO slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.Index << "."
             << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(error.SubIdx) << " error: 0x" << std::setfill('0')
             << std::setw(8) << std::hex << static_cast<unsigned>(error.AbortCode) << " " << ec_sdoerror2string(error.AbortCode);
      break;
    case EC_ERR_TYPE_SOE_ERROR:
      stream << " SoE slave: " << error.Slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.Index
             << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex << static_cast<unsigned>(error.AbortCode) << " "
             << ec_soeerror2string(error.ErrorCode);
      break;
    case EC_ERR_TYPE_MBX_ERROR:
      stream << " MBX slave: " << error.Slave << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex << error.ErrorCode << " "
             << ec_mbxerror2string(error.ErrorCode);
      break;
    default:
      stream << " MBX slave: " << error.Slave << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex
             << static_cast<unsigned>(error.AbortCode);
      break;
  }
  return stream.str();
}

void EthercatBusBase::printALStatus(const uint16_t slave) {
  std::lock_guard<std::recursive_mutex> guard(contextMutex_);
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  std::stringstream ss;
  ss << " slave: " << slave << " alStatusCode: 0x" << std::setfill('0') <<
                   std::setw(8) << std::hex << ecatContext_.slavelist[slave].ALstatuscode <<
                   " " << ec_ALstatuscode2string(ecatContext_.slavelist[slave].ALstatuscode);
  spdlog::info(ss.str());
}

bool EthercatBusBase::checkForSdoErrors(const uint16_t slave, const uint16_t index) {
  while (ecx_iserror(&ecatContext_)) {
    ec_errort error;
    if (ecx_poperror(&ecatContext_, &error)) {
      std::string errorStr = getErrorString(error);
      if (error.Slave == slave && error.Index == index) {
        spdlog::error(errorStr);
        return true;
      }
    }
  }
  return false;
}

bool EthercatBusBase::workingCounterIsOk() const { return wkc_ >= getExpectedWorkingCounter(); }

bool EthercatBusBase::busIsOk() const { return workingCounterTooLowCounter_ < maxWorkingCounterTooLow_; }

void EthercatBusBase::syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime, const double cycleShift) {
  std::stringstream ss;
  ss << "Bus '" << name_ << "', slave " << slave << ":  " << (activate ? "Activating" : "Deactivating")
                           << " distributed clock synchronization...";
  spdlog::info(ss.str());

  ss.clear();
  ecx_dcsync0(&ecatContext_, slave, static_cast<uint8_t>(activate), static_cast<uint32_t>(cycleTime * 1e9),
              static_cast<int32_t>(1e9 * cycleShift));

  ss << "Bus '" << name_ << "', slave " << slave << ":  " << (activate ? "Activated" : "Deactivated")
                           << " distributed clock synchronization.";
}

EthercatBusBase::PdoSizeMap EthercatBusBase::getHardwarePdoSizes() {
  PdoSizeMap pdoMap;

  for (const auto& slave : slaves_) {
    pdoMap.insert(std::make_pair(slave->getName(), getHardwarePdoSizes(slave->getAddress())));
  }

  return pdoMap;
}

EthercatBusBase::PdoSizePair EthercatBusBase::getHardwarePdoSizes(const uint16_t slave) {
  return std::make_pair(ecatContext_.slavelist[slave].Obytes, ecatContext_.slavelist[slave].Ibytes);
}

template<>
bool EthercatBusBase::sendSdoRead<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, std::string& value) {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    //Expected length of the string. String needs to be preallocated
    int size = value.length();
    //Store for check at the end
    int expected_size = size;
    //Create buffer with the length of the string
    char buffer[size];
    int wkc = 0;
    {
      std::lock_guard<std::recursive_mutex> guard(contextMutex_);
      wkc = ecx_SDOread(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), &size, buffer, EC_TIMEOUTRXM);
      //Convert read data to a std::string
      value = std::string(buffer,size);
    }
    if (wkc <= 0) {
      std::stringstream ss;
      ss << "Slave " << slave << ": Working counter too low (" << wkc << ") for reading SDO (ID: 0x" << std::setfill('0')
                                 << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                                 << static_cast<uint16_t>(subindex) << ").";
      spdlog::error(ss.str());
      return false;
    }

    if (size != (int)expected_size) {
      std::stringstream ss;
      ss << "Slave " << slave << ": Size mismatch (expected " << expected_size << " bytes, read " << size
                                 << " bytes) for reading SDO (ID: 0x" << std::setfill('0') << std::setw(4) << std::hex << index
                                 << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(subindex) << ").";
      spdlog::error(ss.str());
      return false;
    }
    return true;
}

template<>
bool EthercatBusBase::sendSdoWrite<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, const std::string value) {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    const int size = value.length();
    const char* dataPtr = value.data();
    int wkc = 0;
    {
        std::lock_guard<std::recursive_mutex> guard(contextMutex_);
        wkc = ecx_SDOwrite(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), size, &dataPtr, EC_TIMEOUTRXM);
    }
    if (wkc <= 0) {
      std::stringstream ss;
        ss << "Slave " << slave << ": Working counter too low (" << wkc << ") for writing SDO (ID: 0x" << std::setfill('0')
                                   << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                                   << static_cast<uint16_t>(subindex) << ").";
        spdlog::error(ss.str());
        return false;
    }
    return true;
}
}  // namespace soem_interface

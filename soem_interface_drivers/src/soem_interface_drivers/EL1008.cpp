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

#include "soem_interface_drivers/EL1008.hpp"


namespace EtherCAT::Drivers {

EL1008::EL1008(const std::string& name, soem_interface::EthercatBusBase* bus, const uint32_t address) :
soem_interface::EthercatSlaveBase(bus, address), 
name_(name)
{
  pdoInfo_.rxPdoId_ = RX_PDO_ID;
  pdoInfo_.txPdoId_ = TX_PDO_ID;
  pdoInfo_.rxPdoSize_ = 0;//sizeof(command_);
  pdoInfo_.txPdoSize_ = sizeof(reading_);
  pdoInfo_.moduleId_ = 0x03ec3052;
}

bool EL1008::startup() {
  // Do nothing else
  return true;
}

void EL1008::updateRead() {
  bus_->readTxPdo(address_, reading_);
  inputs = reading_.inputs;
  spdlog::debug(inputs.to_string());
  
}

void EL1008::updateWrite() {
  //bus_->writeRxPdo(address_, command_);
}

void EL1008::shutdown() {
  // Do nothing
}

}
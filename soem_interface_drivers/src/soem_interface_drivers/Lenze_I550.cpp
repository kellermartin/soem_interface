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

#include "soem_interface_drivers/Lenze_I550.hpp"


namespace EtherCAT::Drivers {

Lenze_I550::Lenze_I550(const std::string& name, soem_interface::EthercatBusBase* bus, const uint32_t address) :
soem_interface::EthercatSlaveBase(bus, address), 
name_(name)
{
  pdoInfo_.rxPdoId_ = RX_PDO_ID;
  pdoInfo_.txPdoId_ = TX_PDO_ID;
  pdoInfo_.rxPdoSize_ = sizeof(command_);
  pdoInfo_.txPdoSize_ = sizeof(reading_);
  pdoInfo_.moduleId_ = 0x00123456;
}

bool Lenze_I550::startup() {
  // Do nothing else
  return true;
}

void Lenze_I550::updateRead() {
  bus_->readTxPdo(address_, reading_);
  std::stringstream ss;
  ss << "LENZE I550 PDO RECEIVED " << std::endl;
  ss << "Statusword: " << reading_.statusWord << std::endl;
  ss << "Actualvelocity: " << reading_.actualVelocity << std::endl;
  ss << "Errorcode: " << reading_.errorCode << std::endl; 
  spdlog::debug(ss.str());
  pdoCb_(reading_);
}

void Lenze_I550::updateWrite() {
  bus_->writeRxPdo(address_, command_);
  std::stringstream ss;
  ss << "LENZE I550 PDO SENT " << std::endl;
  ss << "Statusword: " << reading_.statusWord << std::endl;
  ss << "Actualvelocity: " << reading_.actualVelocity << std::endl;
  ss << "Errorcode: " << reading_.errorCode << std::endl; 
  spdlog::debug(ss.str());
}

void Lenze_I550::shutdown() {
  // Do nothing
}

void Lenze_I550::attachPDOCallback(Lenze_I550_Cb cb){
  pdoCb_ = cb;
}

}
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

#include "soem_interface_drivers/EL1004.hpp"
#include "soem_interface_drivers/EL1008.hpp"
#include "soem_interface_drivers/EL2008.hpp"
#include "soem_interface_drivers/Lenze_I550.hpp"
#include "soem_interface/EthercatBusBase.hpp"

#include "spdlog/spdlog.h"
// This shows a minimal example on how to use the soem_interface library. 
// Keep in mind that this is non-working example code, with only minimal error handling


int main(int argc, char** argv) {

  spdlog::info("Starting...");

  //const std::string busName = "enp2s0";
  const std::string busName = "eth0";
 
  spdlog::info("Create EtherCAT master...");
  std::unique_ptr<soem_interface::EthercatBusBase> bus = std::make_unique<soem_interface::EthercatBusBase> (
    busName);

  bus->printAvailableBusses();

  auto noOfSlaves = bus->getNumberOfSlaves();
  spdlog::debug("Found slaves = %i",noOfSlaves);

  const std::string slaveName = "EL1008";
  const uint32_t slaveAddress = 2;
  std::shared_ptr<EtherCAT::Drivers::EL1008> el1008 = std::make_shared<EtherCAT::Drivers::EL1008> (
    slaveName, bus.get(), slaveAddress);
  bus->addSlave(el1008);

  const std::string slave2Name = "EL2008";
  const uint32_t slave2Address = 3;
  std::shared_ptr<EtherCAT::Drivers::EL2008> el2008 = std::make_shared<EtherCAT::Drivers::EL2008> (
    slave2Name, bus.get(), slave2Address); 
  bus->addSlave(el2008);

  /*const std::string slave3Name = "Lenze_I550";
  const uint32_t slave3Address = 3;
  std::shared_ptr<EtherCAT::Drivers::Lenze_I550> lenzeI550 = std::make_shared<EtherCAT::Drivers::Lenze_I550> (
    slave3Name, bus.get(), slave3Address); 
  bus->addSlave(lenzeI550);

  auto onNewLenzePDOReceived = [&](const EtherCAT::Drivers::Lenze_I550_TxPdo& pdo){
    spdlog::debug("New PDO received ... ");
  };
  
  lenzeI550->attachPDOCallback(onNewLenzePDOReceived);
*/

  //start ethercat master
  bus->startup();
  //request operational state
  bus->setState(EC_STATE_OPERATIONAL);

  
  //wait for all slaves to be operational
  if(!bus->waitForState(EC_STATE_OPERATIONAL, slaveAddress)) {
    spdlog::error("Slaves not operational");
    return 1;
  }
  
  //start tight control loop
  while(bus->busIsOk()) {
    bus->updateRead();
    bus->updateWrite();
  }

  bus->shutdown();
  return 0;
}
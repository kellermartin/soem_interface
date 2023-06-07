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

#pragma once

#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>
#include <bitset>

#define RX_PDO_ID 0x6000
#define TX_PDO_ID 0x7000

namespace EtherCAT::Drivers {

// PDO's are seen from slave side
struct EL2008_TxPdo {
} __attribute__((packed));

struct EL2008_RxPdo {
  uint8_t outputs = 0;
} __attribute__((packed));

// idea create method to set outputs in process image
class EL2008 : public soem_interface::EthercatSlaveBase {
public:
  EL2008(const std::string& name, soem_interface::EthercatBusBase* bus, const uint32_t address);
  ~EL2008() override = default;

  std::string getName() const override {
    return name_;
  }

  bool startup() override;
  void updateRead() override;
  void updateWrite() override;
  void shutdown() override;

  PdoInfo getCurrentPdoInfo() const override {
    return pdoInfo_;
  }

private:
  const std::string name_;
  PdoInfo pdoInfo_;
  EL2008_TxPdo reading_;
  EL2008_RxPdo command_;
  std::bitset<8> outputs_;

};

}
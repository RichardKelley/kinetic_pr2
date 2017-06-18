
#ifndef HARDWARE_INTERFACE_INTERFACE_RESOURCES_H
#define HARDWARE_INTERFACE_INTERFACE_RESOURCES_H

#include <set>
#include <string>
#include <vector>

namespace hardware_interface
{

/**
 * \brief Structure for storing resource identifiers belonging to a specific
 * hardware interface.
 */
struct InterfaceResources
{
  InterfaceResources() {}

  InterfaceResources(const std::string& hw_iface, const std::set<std::string>& res)
    : hardware_interface(hw_iface),
      resources(res)
  {}

  /** Hardware interface type. */
  std::string hardware_interface;

  /** Resources belonging to the hardware interface. */
  std::set<std::string> resources;
};

}

#endif

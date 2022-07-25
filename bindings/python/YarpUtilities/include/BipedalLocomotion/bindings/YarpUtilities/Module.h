/**
 * @file Module.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_YARP_UTILITES_MODULE_H
#define BIPEDAL_LOCOMOTION_BINDINGS_YARP_UTILITES_MODULE_H

#include <pybind11/pybind11.h>

namespace BipedalLocomotion
{
namespace bindings
{
namespace YarpUtilities
{

void CreateModule(pybind11::module& module);

} // namespace YarpUtilities
} // namespace bindings
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_BINDINGS_YARP_UTILITES_MODULE_H

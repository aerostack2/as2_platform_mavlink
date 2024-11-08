// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//    of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file mavlink_platform_test.cpp
 *
 * Pixhawk test
 *
 * @author Rafael Perez-Segui <r.psegui@upm.es>
 */

#include <iostream>
#include "as2_platform_mavlink/thrust_map.hpp"

int main(int argc, char * argv[])
{
  ThrustMap thrust_map = ThrustMap(4);
  thrust_map.set_parameters(
    -18414.433600449644,
    829.3300577713844,
    2453.1637496766352,
    -19.827039781458588,
    -36.0309786395156,
    -77.3083750576221);

  double input_thrust = 10.0;
  auto thrust_new = std::clamp(
    input_thrust,
    0.0,
    2.0);
  double voltage = 14.0;

  for (double i = 0; i <= 20; i = i + 5) {
    auto val = thrust_map.mapThrust(i / 4.0, voltage);
    auto thrust_normalized_new = thrust_map.getThrottle_normalized(
      i, voltage);
    std::cout << "Thrust: " << i << std::endl;
    std::cout << "Throttle normalized: " << thrust_normalized_new << std::endl;
  }

  // auto thrust_normalized_new = thrust_map.getThrottle_normalized(
  //   thrust_new, voltage);
  return 0;
}

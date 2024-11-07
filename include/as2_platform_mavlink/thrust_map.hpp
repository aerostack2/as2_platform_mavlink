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
 * @file thrust_map.hpp
 *
 * ThrustMap class declaration
 *
 * @author Miguel Fernández Cortizas
 *
 */

#ifndef THRUST_MAP_HPP_
#define THRUST_MAP_HPP_

#include <algorithm>
#include <cstdint>
#include <ostream>
#include <string>

class ThrustMap
{
public:
  explicit ThrustMap(unsigned int n_motors)
  : a(0.0), b(0.0), c(0.0), d(0.0), e(0.0), f(0.0), n_motors(n_motors) {}
  explicit ThrustMap(
    unsigned int n_motors, double a, double b, double c,
    double d, double e, double f)
  : a(a), b(b), c(c), d(d), e(e), f(f) {}

  void set_parameters(
    double a, double b, double c, double d, double e,
    double f)
  {
    this->a = a;
    this->b = b;
    this->c = c;
    this->d = d;
    this->e = e;
    this->f = f;
  }

  friend std::ostream & operator<<(std::ostream & os, const ThrustMap & tm)
  {
    os << "ThrustMap: " << tm.a << " " << tm.b << " " << tm.c << " " << tm.d
       << " " << tm.e << " " << tm.f;
    return os;
  }

  std::string to_string() const
  {
    return "ThrustMap: " + std::to_string(a) + " " + std::to_string(b) + " " +
           std::to_string(c) + " " + std::to_string(d) + " " +
           std::to_string(e) + " " + std::to_string(f);
  }

  double mapThrust(double thrust, double voltage)
  {
    double x = thrust;
    double y = voltage;

    return a + b * x + c * y + d * x * x + e * y * y + f * x * y;
  }

  uint16_t getThrottle_useconds(double thrust, double voltage)
  {
    double thrust_per_motor = thrust / static_cast<double>(n_motors);
    uint16_t throttle =
      static_cast<uint16_t>(mapThrust(thrust_per_motor, voltage));
    throttle = (throttle < 1000) ? 1000 : throttle;
    throttle = (throttle > 2000) ? 2000 : throttle;
    return throttle;
  }

  double getThrottle_normalized(double thrust, double voltage)
  {
    double thrust_per_motor = thrust / static_cast<double>(n_motors);
    double throttle = (mapThrust(thrust_per_motor, voltage) - 1000.0) / 1000.0;
    return std::clamp(throttle, 0.0, 1.0);
  }

private:
  double a, b, c, d, e, f;
  uint n_motors;
};

#endif  // THRUST_MAP_HPP_

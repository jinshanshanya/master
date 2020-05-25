/******************************************************************************
 * Copyright 2017 The glb_auto Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "util/util.h"

#include <cmath>
#include <vector>

namespace glb_auto_perception_sensorfusion {
namespace util {


glb_auto_perception_sensorfusion::Point MakePerceptionPoint(const double x, const double y,
                                              const double z) {
  glb_auto_perception_sensorfusion::Point point3d;
  point3d.set_x(x);
  point3d.set_y(y);
  point3d.set_z(z);
  return point3d;
}

}  // namespace util
}  // namespace glb_auto

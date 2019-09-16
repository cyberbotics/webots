# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from geometry.vec3 import Vec3
from geometry.vec2 import Vec2


def distributionGGX(N, H, roughness):
    assert isinstance(N, Vec3)
    assert isinstance(H, Vec3)
    assert isinstance(roughness, float)

    a = roughness * roughness
    a2 = a * a
    NdotH = max(N * H, 0.0)
    NdotH2 = NdotH * NdotH

    nom = a2
    denom = (NdotH2 * (a2 - 1.0) + 1.0)
    denom = math.pi * denom * denom

    ret = nom / denom
    assert isinstance(ret, float)
    return ret


# http://holger.dammertz.org/stuff/notes_HammersleyOnHemisphere.html
# efficient VanDerCorpus calculation.
def vanDerCorpusRadicalInverse(bits):
    assert isinstance(bits, int)
    bits = (bits << 16) | (bits >> 16)
    bits = ((bits & 0x55555555) << 1) | ((bits & 0xAAAAAAAA) >> 1)
    bits = ((bits & 0x33333333) << 2) | ((bits & 0xCCCCCCCC) >> 2)
    bits = ((bits & 0x0F0F0F0F) << 4) | ((bits & 0xF0F0F0F0) >> 4)
    bits = ((bits & 0x00FF00FF) << 8) | ((bits & 0xFF00FF00) >> 8)
    ret = float(bits) * 2.3283064365386963e-10  # / 0x100000000
    assert isinstance(ret, float)
    return ret


def hammersley(i, N):
    assert isinstance(i, int)
    assert isinstance(N, int)
    ret = Vec2(float(i) / float(N), vanDerCorpusRadicalInverse(i))
    assert isinstance(ret, Vec2)
    return ret


def importanceSampleGGX(Xi, N, roughness):
    assert isinstance(Xi, Vec2)
    assert isinstance(N, Vec3)
    assert isinstance(roughness, float)

    a = roughness * roughness

    phi = 2.0 * math.pi * Xi.x
    cosTheta = math.sqrt((1.0 - Xi.y) / (1.0 + (a * a - 1.0) * Xi.y))
    sinTheta = math.sqrt(1.0 - cosTheta * cosTheta)

    # from spherical coordinates to cartesian coordinates - halfway vector
    H = Vec3()
    H.x = math.cos(phi) * sinTheta
    H.y = math.sin(phi) * sinTheta
    H.z = cosTheta

    # from tangent-space H vector to world-space sample vector
    up = Vec3(0.0, 0.0, 1.0) if abs(N.z) < 0.999 else Vec3(1.0, 0.0, 0.0)
    tangent = up.cross(N).normalize()
    bitangent = N.cross(tangent)

    sampleVec = tangent * H.x + bitangent * H.y + N * H.z
    sampleVec = sampleVec.normalize()
    assert isinstance(sampleVec, Vec3)
    return sampleVec

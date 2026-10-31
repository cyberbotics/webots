// Copyright 1996-2025 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VERTEX_INDEX_HPP
#define VERTEX_INDEX_HPP

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <vector>

namespace wren {
  namespace vertexindex {

    // An array of per-vertex values: vertex i starts at data + i * stride.
    struct Attribute {
      const void *data;  // NULL for an absent attribute
      size_t stride;
    };

    // Merges the vertices whose attributes are byte-identical. Every vertex used by the indices gets a new number, in the
    // order in which the indices first use it, and identical vertices get the same number. The indices are rewritten with
    // the new numbers, and the returned vector gives, for each new number, the original vertex it stands for (see gather).
    // Unused vertices are dropped. Comparing bytes merges only vertices that every shader would process identically.
    inline std::vector<unsigned int> mergeIdenticalVertices(std::vector<unsigned int> &indices, size_t vertexCount,
                                                            std::initializer_list<Attribute> attributes) {
      std::vector<Attribute> present;
      for (const Attribute &attribute : attributes)
        if (attribute.data)
          present.push_back(attribute);

      const auto hash = [&present](size_t vertex) {
        uint64_t h = 0x9e3779b97f4a7c15ull;
        for (const Attribute &attribute : present) {
          const unsigned char *bytes = static_cast<const unsigned char *>(attribute.data) + vertex * attribute.stride;
          for (size_t offset = 0; offset < attribute.stride; offset += 8) {
            uint64_t word = 0;
            memcpy(&word, bytes + offset, std::min<size_t>(8, attribute.stride - offset));
            h = (h ^ word) * 0xbf58476d1ce4e5b9ull;
            h ^= h >> 31;
          }
        }
        return h;
      };
      const auto equal = [&present](size_t a, size_t b) {
        for (const Attribute &attribute : present) {
          const unsigned char *bytes = static_cast<const unsigned char *>(attribute.data);
          if (memcmp(bytes + a * attribute.stride, bytes + b * attribute.stride, attribute.stride))
            return false;
        }
        return true;
      };

      const unsigned int none = ~0u;
      std::vector<unsigned int> source;                     // original vertex of each new number
      std::vector<unsigned int> number(vertexCount, none);  // new number of each original vertex, once used
      // open addressing table of original vertices, one per distinct value
      size_t capacity = 16;
      while (capacity < 2 * std::min(vertexCount, indices.size()))
        capacity *= 2;
      std::vector<unsigned int> table(capacity, none);

      for (unsigned int &index : indices) {
        assert(index < vertexCount);
        if (number[index] == none) {
          size_t slot = hash(index) & (capacity - 1);
          while (table[slot] != none && !equal(table[slot], index))
            slot = (slot + 1) & (capacity - 1);
          if (table[slot] == none) {
            table[slot] = index;
            number[index] = static_cast<unsigned int>(source.size());
            source.push_back(index);
          } else
            number[index] = number[table[slot]];
        }
        index = number[index];
      }
      return source;
    }

    // Keeps the values of the vertices listed by mergeIdenticalVertices, in their new order.
    template<typename T> void gather(std::vector<T> &values, const std::vector<unsigned int> &source) {
      if (values.empty())
        return;
      if (std::is_sorted(source.begin(), source.end())) {  // in place: source[i] >= i
        for (size_t i = 0; i < source.size(); ++i)
          values[i] = values[source[i]];
        values.resize(source.size());
      } else {
        std::vector<T> gathered;
        gathered.reserve(source.size());
        for (unsigned int vertex : source)
          gathered.push_back(values[vertex]);
        values.swap(gathered);
      }
    }

  }  // namespace vertexindex
}  // namespace wren

#endif  // VERTEX_INDEX_HPP

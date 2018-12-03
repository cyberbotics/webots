local M = {}

-- assumption: a vector 3 is a table with x, y and z ids
--             e.g. v = { x = 1; y = 2; z = 3 }

function M.equal(vA, vB)
  return vA.x == vB.x and vA.y == vB.y and vA.z == vB.z
end

function M.add(vA, vB)
  return { x = vA.x + vB.x; y = vA.y + vB.y; z = vA.z + vB.z }
end

function M.minus(vA, vB)
  return { x = vA.x - vB.x; y = vA.y - vB.y; z = vA.z - vB.z }
end

function M.multiply(v, s)
  return { x = s * v.x; y = s * v.y; z = s * v.z }
end

function M.norm(v)
  return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
end

function M.distance(vA, vB)
  return M.norm(M.minus(vA, vB))
end

function M.cross(vA, vB)
  return { x = vA.y * vB.z - vA.z * vB.y, y = vA.z * vB.x - vA.x * vB.z, z = vA.x * vB.y - vA.y * vB.x}
end

function M.dot(vA, vB)
  return vA.x * vB.x + vA.y * vB.y + vA.z * vB.z
end

function M.normalize(v)
  local norm = M.norm(v)
  if norm == 0 then
    error()
  end
  return { x = v.x / norm; y = v.y / norm; z = v.z / norm }
end

return M

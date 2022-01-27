local M = {}

-- assumption: a vector 2 is a table with x and y ids
--             e.g. v = { x = 1; y = 2 }

function M.equal(vA, vB)
  return vA.x == vB.x and vA.y == vB.y
end

function M.add(vA, vB)
  return { x = vA.x + vB.x; y = vA.y + vB.y }
end

function M.minus(vA, vB)
  return { x = vA.x - vB.x; y = vA.y - vB.y }
end

function M.multiply(v, s)
  return { x = s * v.x; y = s * v.y }
end

function M.norm(v)
  return math.sqrt(v.x * v.x + v.y * v.y)
end

function M.atan2(v)
  return math.atan2(v.x, v.y)
end

function M.distance(vA, vB)
  return M.norm(M.minus(vA, vB))
end

function M.angle(vA, vB)
  return M.atan2(M.minus(vA, vB))
end

function M.cross(vA, vB)
  return vA.x * vB.y - vA.y * vB.x
end

function M.dot(vA, vB)
  return vA.x * vB.x + vA.y * vB.y
end

function M.normalize(v)
  local norm = M.norm(v)
  if norm == 0 then
    error()
  end
  return { x = v.x / norm; y = v.y / norm }
end

-- return the intersection point between segment 1 (p1->p2)
-- and segment 2 (p3->p4), a point is a table with x and y ids
-- if no intersections are found return nil
function M.intersection(p1, p2, p3, p4)
  --  check that the interval exists
  if (math.max(p1.x,p2.x) < math.min(p3.x,p4.x)) then
    return nil
  end

  -- check that point 1 and 2 are not equal
  if p1.x == p2.x and p1.y == p2.y then
    return nil
  end

  -- check that point 3 and 4 are not equal
  if p3.x == p4.x and p3.y == p4.y then
    return nil
  end

  -- check for parallel segments
  if (p1.x == p2.x and p3.x == p4.x) or (p1.y == p2.y and p3.y == p4.y) then
    return nil
  end


  local Xa = 0
  local Ya = 0
  -- compute the intersection of the two lines
  if p1.x ~= p2.x and p3.x ~= p4.x then
    local A1 = (p1.y - p2.y) / (p1.x - p2.x)
    local A2 = (p3.y - p4.y) / (p3.x - p4.x)
    local b1 = p1.y - A1 * p1.x
    local b2 = p3.y - A2 * p3.x

    if (A1 == A2) then
      return nil -- Parallel segments
    end

    Xa = (b2 - b1) / (A1 - A2)
    Ya = A2 * Xa + b2
  else
    local A1 = (p1.x - p2.x) / (p1.y - p2.y)
    local A2 = (p3.x - p4.x) / (p3.y - p4.y)
    local b1 = p1.x - A1 * p1.y
    local b2 = p3.x - A2 * p3.y

    if (A1 == A2) then
      return nil -- Parallel segments
    end

    Ya = (b2 - b1) / (A1 - A2)
    Xa = A2 * Ya + b2
  end

  if (Xa < math.max(math.min(p1.x,p2.x), math.min(p3.x,p4.x))) or (Xa > math.min(math.max(p1.x,p2.x), math.max(p3.x,p4.x))) then
    return nil -- intersection is out of bound
  else
    return { x = Xa; y = Ya }
  end
end

return M

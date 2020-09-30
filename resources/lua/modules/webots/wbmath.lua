local M = {}

function M.round(num)
  return math.floor(num + 0.5)
end

function M.degrees2radians(angle)
  return angle * math.pi / 180.0
end

function M.radians2degrees(angle)
  return angle * 180.0 / math.pi
end

return M

local M = {}

local seed = 0

function M.seed(s)
  seed = s
end

-- Linear congruential generator (LCG) algorithm.
-- With the coefficients used in Microsoft Visual Basic.
-- https://en.wikipedia.org/wiki/Linear_congruential_generator
local updateseed = function()
  seed = (1140671485 * seed + 12820163) % 2^24
end

function M.real(min, max)
  if min == nil then
    min = 0
    max = 1
  elseif max == nil then
    max = min
    min = 0
  end
  updateseed()
  return min + seed * (max - min) / (2^24 - 1)
end

function M.integer(min, max)
  if min == nil then
    min = 0
    max = 2^24 - 1
  elseif max == nil then
    max = min
    min = 1
  end
  updateseed()
  return min + seed % (1 + max - min)
end

return M

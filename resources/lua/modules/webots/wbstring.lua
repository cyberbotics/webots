local M = {}

-- split the string according to the pattern and return an array
function M.split(string, pattern) -- cf. http://lua-users.org/wiki/SplitJoin
  local result = {}  -- NOTE: use {n = 0} in Lua-5.0
  local findPattern = "(.-)" .. pattern
  local last_end = 1
  local s, e, cap = string:find(findPattern, 1)
  while s do
    if s ~= 1 or cap ~= "" then
      table.insert(result,cap)
    end
    last_end = e+1
    s, e, cap = string:find(findPattern, last_end)
  end
  if last_end <= #string then
    cap = string:sub(last_end)
    table.insert(result, cap)
  end
  return result
end

return M

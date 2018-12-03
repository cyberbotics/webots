local M = {}

-- copy by value any variable
function M.deepcopy(orig) -- cf. http://lua-users.org/wiki/CopyTable
  local orig_type = type(orig)
  local copy
  if orig_type == 'table' then
    copy = {}
    for orig_key, orig_value in next, orig, nil do
      copy[M.deepcopy(orig_key)] = M.deepcopy(orig_value)
    end
    setmetatable(copy, M.deepcopy(getmetatable(orig)))
  else -- number, string, boolean, etc
    copy = orig
  end
  return copy
end

-- count array length. O(n) algorithm
function M.tablelength(a)
  local count = 0
  for i, el in ipairs(a) do
    count = count + 1
  end
  return count
end

return M

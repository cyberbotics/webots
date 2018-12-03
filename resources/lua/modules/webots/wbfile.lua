local M = {}

-- remove the filename and extension from the file path
function M.getpathwithoutfilename(filePath)
  -- make sure to use '/' as separator
  filePath = string.gsub(filePath, "\\", "/")
  -- reverse the string in order to find the 'first' separator
  filePath = string.reverse(filePath)
  local index = string.find(filePath, "/")
  -- remove everything that is 'before the first' separator and re-reverse the string
  filePath = string.sub(filePath, index + 1)
  filePath = string.reverse(filePath)
  return filePath
end

return M

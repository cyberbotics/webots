-- template script to parse a procedural node

local io = require('io')
local liluat = require('liluat')
stderrString = ""
stdoutString = ""


-- redirect 'io.stderr:write' and 'io.stdout:write' to the 'stderrString' and 'stdoutString' variables
local stderr = {}
function stderr:write(...)
  local arg_strings = {}
  for _, v in ipairs{...} do
    stderrString = stderrString .. tostring(v)
  end
end
io.stderr = stderr

local stdout = {}
function stdout:write(...)
  local arg_strings = {}
  for _, v in ipairs{...} do
    stdoutString = stdoutString .. tostring(v)
  end
end
io.stdout = stdout

local _print = print
print = function(...)
  local arg_strings = {}
  for _, v in ipairs{...} do
    stdoutString = stdoutString .. tostring(v)
  end
end

-- update package.cpath to be able to load '*.dll' and '*.dylib'
%cpath%

-- webots parameters
local fields = {
%fields%
}

-- webots context (world path, PROTO path)
local context = {
%context%
}

-- use liluat to generate the procedural node
local template = liluat.compile('%templateContent%', {start_tag='%openingToken%', end_tag='%closingToken%'}, '%templateFileName%')
content = liluat.render(template, {fields = fields, context = context, require = require, io = io, print = print })


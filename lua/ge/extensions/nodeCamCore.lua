-- nodeCamCore: settings, saved cameras and console commands for nodeCam.
-- Holds everything that must survive a vehicle switch or be reachable from the
-- console and UI. The camera mode is in core/cameraModes/nodeCam.lua.

local M = {}

local SETTINGS_PATH = 'settings/nodeCam.json'

-- ---------------------------------------------------------------------------
-- settings
-- ---------------------------------------------------------------------------

M.defaults = {
  enabled          = true,  -- false holds a steady view, picking still works
  quiet            = false, -- suppress info logging, errors always get through

  fov              = 65,
  moveSpeed        = 1.1,   -- anchor move speed, m/s
  fastMultiplier   = 3.0,
  boundsMargin     = 3.0,   -- how far past the vehicle the anchor may go, metres
  boundsEnabled    = true,  -- false removes the leash entirely

  lookSensitivity  = 0.25,  -- mouse look scaling, raw deltas are far too hot
  keyLookSpeed     = 1.2,   -- look speed for analog/keyboard look, rad/s
  invertYaw        = false,
  invertPitch      = false,
  invertPadYaw     = false, -- pad/keyboard look is a separate path from the
  invertPadPitch   = false, -- mouse, so it gets its own two inversion flags

  slotCount        = 3,     -- saved cameras per vehicle, 1-6

  softness         = 0.0,   -- 0 = rigid, above 0 blends in the virtual springs
  stiffness        = 900.0,
  damping          = 26.0,
  maxSag           = 0.25,  -- hard leash on how far soft mode may drift, metres

  outlierResidual  = 0.09,  -- absolute floor for dropping a node, metres
  outlierMedianScale = 3.0, -- and drop anything this many times the median
  maxDropFraction  = 0.4,   -- never discard more than this share at once

  picker           = false, -- draw nearby nodes and the crosshair pick
  pickRadius       = 2.5,   -- live distance from the camera, metres
  pickRadiusNear   = 0.08,  -- always hittable within this radius, however close
  pickSpread       = 0.045, -- crosshair tolerance, larger is more forgiving
  maxDrawnNodes    = 220,   -- cap, a T-series has well over a thousand nodes
}

M.settings = {}
for k, v in pairs(M.defaults) do M.settings[k] = v end

local LIMITS = {
  fov             = { 10, 140 },
  moveSpeed       = { 0.05, 20 },
  fastMultiplier  = { 1, 20 },
  boundsMargin    = { 0, 50 },
  lookSensitivity = { 0.01, 3.0 },
  keyLookSpeed    = { 0.05, 10 },
  slotCount       = { 1, 6 },
  softness        = { 0, 1 },
  stiffness       = { 1, 5000 },
  damping         = { 0, 200 },
  maxSag          = { 0, 2 },
  pickRadius      = { 0.3, 8 },
  pickSpread      = { 0.005, 0.3 },
  maxDrawnNodes   = { 10, 2000 },
}

-- ---------------------------------------------------------------------------
-- logging
-- ---------------------------------------------------------------------------

-- All info-level output funnels through here so one toggle silences it.
function M.logi(m)
  if not M.settings.quiet then log('I', 'nodeCam', m) end
end

function M.logw(m) log('W', 'nodeCam', m) end
function M.loge(m) log('E', 'nodeCam', m) end

function M.msg(text, ttl)
  ttl = ttl or 2
  local shown = pcall(function() ui_message(text, ttl, 'nodeCam', 'videocam') end)
  if not shown then
    shown = pcall(function()
      guihooks.trigger('Message',
        { ttl = ttl, msg = text, category = 'nodeCam', icon = 'videocam' })
    end)
  end
  M.logi(text)
  return shown
end

-- ---------------------------------------------------------------------------
-- persistence
-- ---------------------------------------------------------------------------

function M.save()
  local ok = pcall(function()
    jsonWriteFile(SETTINGS_PATH, { settings = M.settings }, true)
  end)
  if not ok then M.logw('could not write ' .. SETTINGS_PATH) end
  return ok
end

function M.load()
  local ok, saved = pcall(function() return jsonReadFile(SETTINGS_PATH) end)
  if not ok or type(saved) ~= 'table' or type(saved.settings) ~= 'table' then
    return false
  end
  -- only keys we know about, so a stale file cannot inject junk
  for k, v in pairs(saved.settings) do
    if M.defaults[k] ~= nil and type(v) == type(M.defaults[k]) then
      M.settings[k] = v
    end
  end
  return true
end

function M.resetSettings()
  for k, v in pairs(M.defaults) do M.settings[k] = v end
  M.save()
  M.msg('nodeCam: settings reset to defaults')
end

-- Single setter for the UI, console and keybinds: clamping and persistence
-- happen in one place.
function M.set(key, value)
  local def = M.defaults[key]
  if def == nil then M.logw('unknown setting: ' .. tostring(key)); return false end

  if type(def) == 'boolean' then
    if type(value) == 'number' then value = (value ~= 0) end
    M.settings[key] = (value == true)
  else
    local v = tonumber(value)
    if v == nil then M.logw('setting ' .. key .. ' needs a number'); return false end
    local lim = LIMITS[key]
    if lim then
      if v < lim[1] then v = lim[1] end
      if v > lim[2] then v = lim[2] end
    end
    if key == 'attachCount' or key == 'slotCount' or key == 'maxDrawnNodes' then
      v = math.floor(v)
    end
    M.settings[key] = v
  end

  M.save()
  return true
end

function M.get(key) return M.settings[key] end

function M.toggle(key)
  if type(M.defaults[key]) ~= 'boolean' then return false end
  M.set(key, not M.settings[key])
  M.msg('nodeCam: ' .. key .. ' ' .. (M.settings[key] and 'on' or 'off'))
  return M.settings[key]
end

-- ---------------------------------------------------------------------------
-- pending actions
-- ---------------------------------------------------------------------------

-- The camera mode only runs inside core_camera's update, so keybind, console
-- and UI actions are parked here and picked up next frame.

local pending = {}

local function queue(key, value)
  pending[key] = (value == nil) and true or value
end

function M.consume(key)
  local v = pending[key]
  pending[key] = nil
  return v
end

-- ---------------------------------------------------------------------------
-- actions
-- ---------------------------------------------------------------------------

function M.togglePicker()
  M.toggle('picker')
end

function M.toggleEnabled()
  M.toggle('enabled')
end

function M.toggleNode()
  queue('toggleNode')
end

function M.clearNodes()
  queue('clearNodes')
end

function M.cycleSlot()
  queue('cycleSlot')
end

function M.nudge(dir, amount)
  amount = tonumber(amount) or 0.15
  local n = { x = 0, y = 0, z = 0 }
  if dir == 'forward' then n.y = -amount
  elseif dir == 'back' then n.y = amount
  elseif dir == 'left' then n.x = amount
  elseif dir == 'right' then n.x = -amount
  elseif dir == 'up' then n.z = amount
  elseif dir == 'down' then n.z = -amount
  else M.logw('nudge: use forward, back, left, right, up or down'); return end
  queue('nudge', n)
  M.logi(string.format('nudging anchor %s by %.2f m', dir, amount))
end

function M.look(yawDeg, pitchDeg)
  queue('look', { yaw = math.rad(tonumber(yawDeg) or 0),
                  pitch = math.rad(tonumber(pitchDeg) or 0) })
end

function M.resetLook()
  queue('resetLook')
end

-- ---------------------------------------------------------------------------
-- anchors and camera slots
-- ---------------------------------------------------------------------------

local anchors = {}
local slots, activeSlot = {}, {}
local states = {}

function M.getAnchor(vid) return anchors[vid] end

function M.setAnchor(vid, x, y, z)
  local a = anchors[vid]
  if a then a.x, a.y, a.z = x, y, z
  else anchors[vid] = { x = x, y = y, z = z } end
end

function M.slotIndex(vid)
  local i = activeSlot[vid] or 1
  if i > M.settings.slotCount then i = 1 end
  return i
end

function M.setSlotIndex(vid, i) activeSlot[vid] = i end

function M.saveSlot(vid, i, d)
  slots[vid] = slots[vid] or {}
  slots[vid][i] = d
end

function M.getSlot(vid, i)
  return slots[vid] and slots[vid][i]
end

-- The live camera for a vehicle, parked when you tab away and handed back when
-- you tab in again. Keyed by the same shape signature the camera mode uses, so
-- a replaced body never inherits the old one's nodes.
function M.saveState(vid, st)
  if vid and st then states[vid] = st end
end

function M.getState(vid)
  return states[vid]
end

-- ---------------------------------------------------------------------------
-- state published by the camera mode, for diag and the UI
-- ---------------------------------------------------------------------------

M.live = false
M.liveInfo = nil
M.moveSource = nil
M.lookSource = nil
M.lookSeen = nil

-- Everything the UI needs in one call.
function M.requestUIState()
  local li = M.liveInfo or {}
  return {
    active = M.settings.enabled,
    running = M.live == true,
    settings = M.settings,
    attached = li.nSel or 0,
    totalNodes = li.nNodes or 0,
    slot = li.vid and M.slotIndex(li.vid) or 1,
    slotCount = M.settings.slotCount,
    steady = (li.nSel or 0) < 4,
    fallback = li.fallback,
  }
end

function M.status()
  local s = M.settings
  M.logi(string.format(
    'enabled=%s fov=%d moveSpeed=%.2f look=%.2f slots=%d picker=%s',
    tostring(s.enabled), s.fov, s.moveSpeed, s.lookSensitivity,
    s.slotCount, tostring(s.picker)))
end

function M.diag()
  local out = {}
  local function add(fmt, ...)
    local ok, line = pcall(string.format, fmt, ...)
    out[#out + 1] = ok and line or (fmt .. '  <bad args>')
  end

  add('--- nodeCam diagnostics ---')
  add('enabled / running   : %s / %s', tostring(M.settings.enabled), tostring(M.live == true))
  add('move source seen    : %s', tostring(M.moveSource or 'none yet'))
  add('look source seen    : %s', tostring(M.lookSource or 'none yet'))
  local ls = M.lookSeen
  if ls then
    add('  mouse delta path  : %s', tostring(ls.delta or 'nothing yet'))
    add('  pad/analog path   : %s', tostring(ls.pair or 'nothing yet'))
  end
  add('invert y/p, pad y/p : %s/%s, %s/%s',
    tostring(M.settings.invertYaw), tostring(M.settings.invertPitch),
    tostring(M.settings.invertPadYaw), tostring(M.settings.invertPadPitch))

  local li = M.liveInfo
  if li then
    add('state               : %s',
      li.fallback and ('FALLBACK - ' .. tostring(li.fallback))
      or (((li.nSel or 0) >= 4) and 'attached' or 'steady, no nodes picked'))
    add('node map built      : %s', tostring(li.shapeReady == true))
    if li.shapeFail then add('node map failure    : %s', tostring(li.shapeFail)) end
    add('vehicle frame from  : %s', tostring(li.frameSource or 'not resolved'))
    add('vehicle / nodes     : %s / %s total, %s attached',
      tostring(li.vid), tostring(li.nNodes), tostring(li.nSel))
    add('picker drawn / hover: %s / %s',
      tostring(li.nPick or 0), tostring(li.hover or 'none'))
    add('camera slot         : %d of %d',
      M.slotIndex(li.vid), M.settings.slotCount)
    add('anchor              : %.3f %.3f %.3f',
      li.anchorX or 0, li.anchorY or 0, li.anchorZ or 0)
    add('anchor clamped      : %s', tostring(li.clamped == true))
    add('yaw / pitch         : %.3f / %.3f rad', li.yaw or 0, li.pitch or 0)
    if li.inputSeen and #li.inputSeen > 0 then
      add('input fields moving : %s', table.concat(li.inputSeen, ', '))
    end
  else
    add('camera mode has not run yet, press C until you reach nodeCam')
  end

  M.status()
  for _, line in ipairs(out) do log('I', 'nodeCam', line) end
  return table.concat(out, '\n')
end

-- ---------------------------------------------------------------------------
-- lifecycle
-- ---------------------------------------------------------------------------

local function forgetVehicle(vid)
  if not vid then return end
  anchors[vid] = nil
  slots[vid] = nil
  activeSlot[vid] = nil
  states[vid] = nil
end

function M.onVehicleDestroyed(vid)
  pcall(function() forgetVehicle(vid) end)
end

-- A replaced vehicle usually keeps its ID, so its saved camera would otherwise
-- carry over to a body it was never picked on.
function M.onVehicleSpawned(vid)
  pcall(function()
    forgetVehicle(vid)
    queue('invalidate')
  end)
end

-- Deliberately does nothing. Tabbing between vehicles is not a vehicle change:
-- the camera mode compares a shape signature and restores each vehicle's own
-- saved camera. Forcing an invalidate here wiped the node set on every tab.

function M.onExtensionLoaded()
  M.load()
  M.logi('nodeCamCore 3.0 loaded. Press C to cycle to nodeCam.')
  return true
end

return M

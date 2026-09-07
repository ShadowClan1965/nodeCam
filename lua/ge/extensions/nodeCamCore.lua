-- nodeCamCore: state and controls for the nodeCam camera mode.

local M = {}

local function logi(m) log('I', 'nodeCam', m) end

-- On-screen notice.
function M.msg(text, ttl)
  ttl = ttl or 2
  local shown = pcall(function() ui_message(text, ttl, 'nodeCam', 'videocam') end)
  if not shown then
    shown = pcall(function()
      guihooks.trigger('Message',
        { ttl = ttl, msg = text, category = 'nodeCam', icon = 'videocam' })
    end)
  end
  log('I', 'nodeCam', text)
  return shown
end
local function logw(m) log('W', 'nodeCam', m) end

-- ---------------------------------------------------------------------------
-- settings
-- ---------------------------------------------------------------------------

M.settings = {
  nodeCount        = 10,    -- how many nodes the camera welds itself to
  softness         = 0.0,   -- 0 = rigid. Above 0 blends in the virtual beams
  stiffness        = 900.0, -- virtual beam spring rate
  damping          = 26.0,  -- virtual beam damping
  maxSag           = 0.25,  -- hard leash on how far soft mode may drift, metres
  fov              = 65,
  outlierResidual  = 0.09,  -- absolute floor for dropping a node, metres
  outlierMedianScale = 3.0, -- and drop anything this many times the median residual
  maxDropFraction  = 0.4,   -- never discard more than this share of the set at once
  strainAbsTol     = 0.03,  -- virtual beam length change tolerated, metres
  strainRelTol     = 0.12,  -- plus this share of the beam's rest length
  reattachDist     = 0.12,  -- move the anchor this far and we re-pick nodes
  reattachInterval = 0.15,  -- fastest allowed re-pick rate, seconds
  minSeparation    = 0.05,  -- refuse nodes closer together than this
  searchRadius     = 1.2,   -- initial node search radius around the anchor
  moveSpeed        = 1.1,   -- anchor move speed, m/s
  fastMultiplier   = 3.0,
  lookSensitivity  = 0.25,  -- mouse look scaling. Raw deltas are far too hot
  keyLookSpeed     = 1.2,   -- look speed for analog/keyboard look, rad/s
  invertYaw        = false, -- flip if looking left and right feels backwards
  invertPitch      = false, -- flip if looking up and down feels backwards
  nativeMove       = true,  -- also read the game's own camera movement keys
  lock             = false, -- freeze the attached node set
  bypass           = false, -- ignore the nodes, hold a steady view
  pickRadius       = 2.5,   -- how far around the camera nodes are shown, metres
  pickRadiusNear   = 0.08,  -- always hittable within this radius, however close
  maxDrawnNodes    = 220,   -- cap, a T-series has well over a thousand nodes
  pickSpread       = 0.045, -- crosshair tolerance, larger is more forgiving
  banStrikes       = 8,     -- failures before a node is struck off for good
  strikeDecay      = 2.0,   -- failures forgiven per second while a node behaves
  debug            = false, -- draw the virtual beams
}

-- live movement flags, driven by keybinds
M.move = { fast = false }

-- ---------------------------------------------------------------------------
-- internal state
-- ---------------------------------------------------------------------------

local anchors = {}         -- vid -> last anchor position, vehicle local
local pendingPreset = nil
local pendingReattach = false
local pendingLookReset = false
local pendingClearBans = false
local pendingLook = nil

local PRESETS = { 'dash', 'hood', 'bumper', 'roof', 'tail', 'wheelLeft', 'wheelRight' }
local presetIndex = 1

-- ---------------------------------------------------------------------------
-- anchor persistence
-- ---------------------------------------------------------------------------

function M.getAnchor(vid) return anchors[vid] end

function M.setAnchor(vid, x, y, z)
  local a = anchors[vid]
  if a then
    a.x, a.y, a.z = x, y, z
  else
    anchors[vid] = { x = x, y = y, z = z }
  end
end

-- ---------------------------------------------------------------------------
-- things the camera mode polls each frame
-- ---------------------------------------------------------------------------

function M.consumePreset()
  local p = pendingPreset
  pendingPreset = nil
  return p
end

function M.preset(name)
  pendingPreset = name or 'dash'
  M.msg('nodeCam: ' .. tostring(pendingPreset))
end

function M.cyclePreset(step)
  presetIndex = presetIndex + (step or 1)
  while presetIndex > #PRESETS do presetIndex = presetIndex - #PRESETS end
  while presetIndex < 1 do presetIndex = presetIndex + #PRESETS end
  M.preset(PRESETS[presetIndex])
end

-- Move the anchor straight from the console.
local pendingNudge = nil

function M.nudge(dir, amount)
  amount = tonumber(amount) or 0.15
  local n = { x = 0, y = 0, z = 0 }
  if dir == 'forward' then n.y = -amount
  elseif dir == 'back' then n.y = amount
  elseif dir == 'left' then n.x = amount
  elseif dir == 'right' then n.x = -amount
  elseif dir == 'up' then n.z = amount
  elseif dir == 'down' then n.z = -amount
  else logw("nudge: use forward, back, left, right, up or down"); return end
  pendingNudge = n
  M.hits = M.hits + 1
  logi(string.format('nudging anchor %s by %.2f m', dir, amount))
end

function M.consumeNudge()
  local n = pendingNudge
  pendingNudge = nil
  return n
end

-- Turn the view from the console, same idea for the look path.
function M.look(yawDeg, pitchDeg)
  pendingLook = { yaw = math.rad(tonumber(yawDeg) or 0),
                  pitch = math.rad(tonumber(pitchDeg) or 0) }
  logi(string.format('turning view by %s deg yaw, %s deg pitch',
    tostring(yawDeg), tostring(pitchDeg)))
end

function M.consumeLook()
  local l = pendingLook
  pendingLook = nil
  return l
end

function M.setSoftness(v)
  v = tonumber(v) or 0
  M.settings.softness = math.max(0, math.min(1, v))
  M.msg(string.format('nodeCam: softness %.2f', M.settings.softness))
end

function M.setNodes(n)
  n = tonumber(n) or 10
  M.settings.nodeCount = math.max(4, math.min(32, math.floor(n)))
  pendingReattach = true
  M.msg('nodeCam: using ' .. M.settings.nodeCount .. ' nodes')
end

function M.setFov(f)
  M.settings.fov = math.max(10, math.min(140, tonumber(f) or 65))
  M.msg(string.format('nodeCam: fov %d', M.settings.fov))
end

-- BeamNG's zoom filter runs after camera modes and can overwrite fov.
function M.forceFov(on)
  on = (on ~= false)
  local ok = pcall(function() core_camera.setSkipFovModifier(0, on) end)
  if not ok then ok = pcall(function() core_camera.setSkipFovModifier(on) end) end
  M.msg('nodeCam: fov override ' .. (ok and 'on' or 'unavailable'))
  return ok
end

-- If the vehicle frame had to be guessed from the node cloud, front and back
-- are a coin toss.
local pendingFlip = false

local pendingClear, pendingSlot = false, false

function M.toggleBypass()
  M.settings.bypass = not M.settings.bypass
  M.msg('nodeCam: ' .. (M.settings.bypass and 'nodes ignored, steady view'
    or 'following nodes again'))
end

-- Empties the attached set.
function M.clearNodes()
  M.settings.lock = true
  pendingClear = true
  M.msg('nodeCam: nodes cleared, locked, pick your own')
end

function M.consumeClear()
  local c = pendingClear
  pendingClear = false
  return c
end

-- Three independent camera and node sets per vehicle.
local slots, activeSlot = {}, {}

function M.cycleSlot()
  pendingSlot = true
end

function M.consumeCycleSlot()
  local c = pendingSlot
  pendingSlot = false
  return c
end

function M.slotIndex(vid) return activeSlot[vid] or 1 end
function M.setSlotIndex(vid, i) activeSlot[vid] = i end

function M.saveSlot(vid, i, d)
  slots[vid] = slots[vid] or {}
  slots[vid][i] = d
end

function M.getSlot(vid, i)
  return slots[vid] and slots[vid][i]
end

function M.toggleLock()
  M.settings.lock = not M.settings.lock
  M.msg(M.settings.lock and 'nodeCam: nodes LOCKED' or 'nodeCam: nodes unlocked')
end

local pendingToggleNode = false

function M.toggleNode()
  pendingToggleNode = true
end

function M.consumeToggleNode()
  local r = pendingToggleNode
  pendingToggleNode = false
  return r
end

function M.toggleInvertYaw()
  M.settings.invertYaw = not M.settings.invertYaw
  M.msg('nodeCam: yaw ' .. (M.settings.invertYaw and 'inverted' or 'normal'))
end

function M.setNativeMove(v)
  M.settings.nativeMove = (v ~= false)
  logi('reading the game camera movement keys: ' .. tostring(M.settings.nativeMove))
end

function M.setPickRadius(r)
  M.settings.pickRadius = math.max(0.3, math.min(8, tonumber(r) or 2.5))
  logi(string.format('showing nodes within %.2f m', M.settings.pickRadius))
end

function M.flipForward()
  pendingFlip = true
  M.msg('nodeCam: flipped front/back')
end

function M.consumeFlip()
  local f = pendingFlip
  pendingFlip = false
  return f
end

function M.clearBans()
  pendingClearBans = true
  M.msg('nodeCam: cleared all node choices')
end

function M.consumeClearBans()
  local r = pendingClearBans
  pendingClearBans = false
  return r
end

function M.setBanStrikes(n)
  M.settings.banStrikes = math.max(1, math.min(200, tonumber(n) or 8))
  logi('nodes struck off after ' .. M.settings.banStrikes .. ' failures')
end

function M.setLookSensitivity(v)
  M.settings.lookSensitivity = math.max(0.01, math.min(3.0, tonumber(v) or 0.25))
  logi(string.format('look sensitivity %.2f', M.settings.lookSensitivity))
end

function M.toggleInvertPitch()
  M.settings.invertPitch = not M.settings.invertPitch
  M.msg('nodeCam: pitch ' .. (M.settings.invertPitch and 'inverted' or 'normal'))
end

function M.resetLook()
  pendingLookReset = true
  M.msg('nodeCam: view recentred')
end

function M.consumeLookReset()
  local r = pendingLookReset
  pendingLookReset = false
  return r
end

function M.setMoveSpeed(s)
  M.settings.moveSpeed = math.max(0.05, math.min(20, tonumber(s) or 1.1))
  M.msg(string.format('nodeCam: move speed %.2f m/s', M.settings.moveSpeed))
end

function M.toggleDebug()
  M.settings.debug = not M.settings.debug
  M.msg('nodeCam: node picker ' .. (M.settings.debug and 'on' or 'off'))
end

function M.status()
  local s = M.settings
  logi(string.format(
    'nodes=%d softness=%.2f fov=%d moveSpeed=%.2f look=%.2f lock=%s debug=%s',
    s.nodeCount, s.softness, s.fov, s.moveSpeed, s.lookSensitivity,
    tostring(s.lock), tostring(s.debug)))
end

-- ---------------------------------------------------------------------------
-- diagnostics
-- ---------------------------------------------------------------------------

-- Dump what the mod can actually see.
function M.diag()
  local out = {}
  local function add(fmt, ...) out[#out + 1] = string.format(fmt, ...) end

  add('--- nodeCam diagnostics ---')
  add('camera mode running : %s', tostring(M.live == true))
  add('binding hits so far : %d  (press a movement key, then run this again)', M.hits)
  add('look source seen    : %s', tostring(M.lookSource or 'none yet'))

  local li = M.liveInfo
  if li then
    add('state               : %s', li.fallback and ('FALLBACK - ' .. tostring(li.fallback))
      or 'attached and running')
    add('node map built      : %s', tostring(li.shapeReady == true))
    if li.shapeFail then add('node map failure    : %s', tostring(li.shapeFail)) end
    add('refNodes provided   : %s', tostring(li.hasRefNodes))
    add('vehicle frame from  : %s', tostring(li.frameSource or 'not resolved'))
    add('node set            : %s', li.locked and 'LOCKED' or 'auto')
    add('nodes drawn / hover : %s / %s', tostring(li.nPick or 0), tostring(li.hover or 'none'))
    add('movement source     : %s', tostring(M.moveSource or 'none seen'))
    add('camera slot         : %d of 3', M.slotIndex(li.vid))
    add('fov / move speed    : %d / %.2f  (asked for, may be overridden)',
      M.settings.fov, M.settings.moveSpeed)
    add('bypass              : %s', tostring(M.settings.bypass))
    if li.inputSeen then
      add('input fields moving : %s',
        (#li.inputSeen > 0) and table.concat(li.inputSeen, ', ')
        or '(none seen yet - move the mouse, then run this again)')
    end
    add('vehicle %s, %s nodes total, %s attached',
      tostring(li.vid), tostring(li.nNodes), tostring(li.nSel))
    add('anchor  %.3f %.3f %.3f', li.anchorX or 0, li.anchorY or 0, li.anchorZ or 0)
    add('yaw %.3f  pitch %.3f rad', li.yaw or 0, li.pitch or 0)
    if li.dataKeys then
      add('camera data fields  : %s', table.concat(li.dataKeys, ', '))
    end
  else
    add('camera mode has not run yet, press C until you reach nodeCam')
  end

  -- which methods does the vehicle object actually expose to us
  local veh
  pcall(function() veh = be:getPlayerVehicle(0) end)
  if veh then
    local probe = { 'getNodeCount', 'getNodePosition', 'getPosition', 'getRotation',
                    'getRefNodeRotation', 'getDirectionVector', 'getDirectionVectorUp' }
    local have, missing = {}, {}
    for _, name in ipairs(probe) do
      local ok, v = pcall(function() return veh[name] end)
      if ok and v ~= nil then have[#have + 1] = name else missing[#missing + 1] = name end
    end
    add('vehicle has          : %s', table.concat(have, ', '))
    add('vehicle lacks        : %s', (#missing > 0) and table.concat(missing, ', ') or '(none)')
    local okc, cnt = pcall(function() return veh:getNodeCount() end)
    add('getNodeCount()       : %s', okc and tostring(cnt) or 'CALL FAILED')
  else
    add('no player vehicle object available')
  end

  -- what does MoveManager actually offer
  local mm = MoveManager
  add('MoveManager type    : %s', type(mm))
  if type(mm) == 'table' then
    local names, nonzero = {}, {}
    for k, v in pairs(mm) do
      if type(v) == 'number' then
        names[#names + 1] = k
        if v ~= 0 then nonzero[#nonzero + 1] = string.format('%s=%.4f', k, v) end
      end
    end
    table.sort(names)
    add('MoveManager numbers : %s', table.concat(names, ', '))
    add('currently non-zero  : %s',
      (#nonzero > 0) and table.concat(nonzero, ', ') or '(none)')
  end

  for _, line in ipairs(out) do logi(line) end
  return table.concat(out, '\n')
end

-- ---------------------------------------------------------------------------
-- lifecycle
-- ---------------------------------------------------------------------------

-- Every hook below is wrapped.
function M.onVehicleDestroyed(vid)
  pcall(function()
    if vid then anchors[vid] = nil end
  end)
end

function M.onExtensionLoaded()
  logi('nodeCamCore 2.1 loaded. Press C to cycle to nodeCam.')
  return true
end

return M

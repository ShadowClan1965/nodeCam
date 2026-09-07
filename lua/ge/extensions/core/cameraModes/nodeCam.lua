-- nodeCam: a camera mode that welds the view to the nearest vehicle nodes and
-- re-picks which nodes it uses as you move the anchor around, live, no reload.

-- Printed the moment core_camera requires this file.
if log then log('I', 'nodeCam', 'nodeCam 2.1 camera mode loaded') end

local C = {}
C.__index = C

local abs, sqrt, min, max, floor = math.abs, math.sqrt, math.min, math.max, math.floor
local cos, sin = math.cos, math.sin
local huge = math.huge
local huge = math.huge

--[[NODECAM_SOLVER_BEGIN]]
-- Pure math below.

local S = {}

-- Orthogonal (rotation) factor of a 3x3 matrix, by Higham's iteration:   M <-
-- 0.5 * (M + inverse(transpose(M))) converges quadratically to the rotation
-- nearest to M.
function S.polar(m11, m12, m13, m21, m22, m23, m31, m32, m33, iters, tol)
  iters = iters or 12
  tol = tol or 1e-10

  local det = m11 * (m22 * m33 - m23 * m32)
            - m12 * (m21 * m33 - m23 * m31)
            + m13 * (m21 * m32 - m22 * m31)

  -- NaN, near-singular (nodes collinear/coplanar) or mirrored: caller falls back
  if det ~= det or det < 1e-12 then return nil end

  -- Normalise scale first so the iteration converges in 3-4 steps instead of 20
  local s = 1.0 / (det ^ (1.0 / 3.0))
  m11, m12, m13 = m11 * s, m12 * s, m13 * s
  m21, m22, m23 = m21 * s, m22 * s, m23 * s
  m31, m32, m33 = m31 * s, m32 * s, m33 * s

  for _ = 1, iters do
    -- inverse(transpose(M)) == cofactor(M) / det(M)
    local c11 =  (m22 * m33 - m23 * m32)
    local c12 = -(m21 * m33 - m23 * m31)
    local c13 =  (m21 * m32 - m22 * m31)
    local c21 = -(m12 * m33 - m13 * m32)
    local c22 =  (m11 * m33 - m13 * m31)
    local c23 = -(m11 * m32 - m12 * m31)
    local c31 =  (m12 * m23 - m13 * m22)
    local c32 = -(m11 * m23 - m13 * m21)
    local c33 =  (m11 * m22 - m12 * m21)

    local d = m11 * c11 + m12 * c12 + m13 * c13
    if d ~= d or abs(d) < 1e-12 then return nil end
    local inv = 1.0 / d

    local n11, n12, n13 = c11 * inv, c12 * inv, c13 * inv
    local n21, n22, n23 = c21 * inv, c22 * inv, c23 * inv
    local n31, n32, n33 = c31 * inv, c32 * inv, c33 * inv

    local o11, o12, o13 = m11, m12, m13
    local o21, o22, o23 = m21, m22, m23
    local o31, o32, o33 = m31, m32, m33

    m11, m12, m13 = 0.5 * (m11 + n11), 0.5 * (m12 + n12), 0.5 * (m13 + n13)
    m21, m22, m23 = 0.5 * (m21 + n21), 0.5 * (m22 + n22), 0.5 * (m23 + n23)
    m31, m32, m33 = 0.5 * (m31 + n31), 0.5 * (m32 + n32), 0.5 * (m33 + n33)

    local delta = abs(m11 - o11) + abs(m12 - o12) + abs(m13 - o13)
                + abs(m21 - o21) + abs(m22 - o22) + abs(m23 - o23)
                + abs(m31 - o31) + abs(m32 - o32) + abs(m33 - o33)
    if delta < tol then break end
  end

  if m11 ~= m11 then return nil end
  return m11, m12, m13, m21, m22, m23, m31, m32, m33
end

-- Weighted rigid fit (Kabsch).
function S.fit(sel, nSel, live, qx, qy, qz, px, py, pz, w)
  local wsum = 0.0
  local qcx, qcy, qcz = 0.0, 0.0, 0.0
  local pcx, pcy, pcz = 0.0, 0.0, 0.0
  local used = 0

  for s = 1, nSel do
    local i = sel[s]
    if live[s] then
      local wi = w[s]
      wsum = wsum + wi
      qcx = qcx + qx[i] * wi; qcy = qcy + qy[i] * wi; qcz = qcz + qz[i] * wi
      pcx = pcx + px[s] * wi; pcy = pcy + py[s] * wi; pcz = pcz + pz[s] * wi
      used = used + 1
    end
  end

  if used < 3 or wsum < 1e-9 then return false end

  local iw = 1.0 / wsum
  qcx, qcy, qcz = qcx * iw, qcy * iw, qcz * iw
  pcx, pcy, pcz = pcx * iw, pcy * iw, pcz * iw

  -- Cross-covariance C = sum(w * b * a^T)
  local c11, c12, c13 = 0.0, 0.0, 0.0
  local c21, c22, c23 = 0.0, 0.0, 0.0
  local c31, c32, c33 = 0.0, 0.0, 0.0

  for s = 1, nSel do
    local i = sel[s]
    if live[s] then
      local wi = w[s]
      local ax, ay, az = qx[i] - qcx, qy[i] - qcy, qz[i] - qcz
      local bx, by, bz = px[s] - pcx, py[s] - pcy, pz[s] - pcz
      c11 = c11 + wi * bx * ax; c12 = c12 + wi * bx * ay; c13 = c13 + wi * bx * az
      c21 = c21 + wi * by * ax; c22 = c22 + wi * by * ay; c23 = c23 + wi * by * az
      c31 = c31 + wi * bz * ax; c32 = c32 + wi * bz * ay; c33 = c33 + wi * bz * az
    end
  end

  local r11, r12, r13, r21, r22, r23, r31, r32, r33 =
    S.polar(c11, c12, c13, c21, c22, c23, c31, c32, c33)

  if r11 == nil then return false end

  return true, qcx, qcy, qcz, pcx, pcy, pcz,
         r11, r12, r13, r21, r22, r23, r31, r32, r33
end

-- Strike accounting.
function S.updateStrikes(sel, nSel, live, strikes, banned, banAt, decay, dt)
  local dec = decay * dt
  local liveCount, newBans = 0, 0
  for s = 1, nSel do
    local i = sel[s]
    if live[s] then
      liveCount = liveCount + 1
      local v = (strikes[i] or 0) - dec
      strikes[i] = (v > 0) and v or 0
    else
      local v = (strikes[i] or 0) + 1
      strikes[i] = v
      if v >= banAt and not banned[i] then
        banned[i] = true
        newBans = newBans + 1
      end
    end
  end
  return liveCount, newBans
end

-- Turn a camera-relative movement request into vehicle-local axes.
function S.moveToLocal(fwdAmt, rightAmt, upAmt,
                       cfx, cfy, cfz, cux, cuy, cuz,
                       r11, r12, r13, r21, r22, r23, r31, r32, r33)
  local crx = cfy * cuz - cfz * cuy
  local cry = cfz * cux - cfx * cuz
  local crz = cfx * cuy - cfy * cux

  local wx = cfx * fwdAmt + crx * rightAmt + cux * upAmt
  local wy = cfy * fwdAmt + cry * rightAmt + cuy * upAmt
  local wz = cfz * fwdAmt + crz * rightAmt + cuz * upAmt

  return r11 * wx + r21 * wy + r31 * wz,
         r12 * wx + r22 * wy + r32 * wz,
         r13 * wx + r23 * wy + r33 * wz
end

-- Closest node to a ray from the camera.
function S.pickAlongRay(n, px, py, pz, ox, oy, oz, dx, dy, dz, spread, hitRadius)
  hitRadius = hitRadius or 0.08
  local best, bestScore = nil, huge
  for i = 1, n do
    local vx, vy, vz = px[i] - ox, py[i] - oy, pz[i] - oz
    local along = vx * dx + vy * dy + vz * dz
    if along > 0.004 then
      local ex = vx - dx * along
      local ey = vy - dy * along
      local ez = vz - dz * along
      local off = sqrt(ex * ex + ey * ey + ez * ez)
      local tol = spread * along
      if hitRadius > tol then tol = hitRadius end
      local score = off / tol
      if score < 1 and score < bestScore then bestScore, best = score, i end
    end
  end
  return best
end

-- Rotate a vector about a unit axis (Rodrigues).
function S.rotAbout(vx, vy, vz, kx, ky, kz, angle)
  local c, sn = cos(angle), sin(angle)
  local t = (kx * vx + ky * vy + kz * vz) * (1 - c)
  return vx * c + (ky * vz - kz * vy) * sn + kx * t,
         vy * c + (kz * vx - kx * vz) * sn + ky * t,
         vz * c + (kx * vy - ky * vx) * sn + kz * t
end

-- Apply look yaw and pitch to a forward/up pair without leaking roll.
function S.applyLook(fx, fy, fz, ux, uy, uz, yaw, pitch)
  if yaw ~= 0 then
    fx, fy, fz = S.rotAbout(fx, fy, fz, ux, uy, uz, yaw)
  end
  if pitch ~= 0 then
    -- right = forward x up, already unit length since the pair is orthonormal
    local rx = fy * uz - fz * uy
    local ry = fz * ux - fx * uz
    local rz = fx * uy - fy * ux
    local rl = sqrt(rx * rx + ry * ry + rz * rz)
    if rl > 1e-6 then
      rx, ry, rz = rx / rl, ry / rl, rz / rl
      fx, fy, fz = S.rotAbout(fx, fy, fz, rx, ry, rz, pitch)
      ux, uy, uz = S.rotAbout(ux, uy, uz, rx, ry, rz, pitch)
    end
  end
  return fx, fy, fz, ux, uy, uz
end

-- Rest length of every virtual beam between the attached nodes.
function S.buildRestDistances(sel, nSel, qx, qy, qz, restD)
  for a = 1, nSel do
    local ia = sel[a]
    for b = a + 1, nSel do
      local ib = sel[b]
      local dx, dy, dz = qx[ia] - qx[ib], qy[ia] - qy[ib], qz[ia] - qz[ib]
      restD[(a - 1) * nSel + b] = sqrt(dx * dx + dy * dy + dz * dz)
    end
  end
end

-- Virtual beam strain test.
function S.strainReject(nSel, live, px, py, pz, restD, absTol, relTol, votes)
  local liveCount = 0
  for a = 1, nSel do
    votes[a] = 0
    if live[a] then liveCount = liveCount + 1 end
  end
  -- with too few partners the vote is meaningless, leave it to the fit
  if liveCount < 5 then return 0 end

  for a = 1, nSel do
    if live[a] then
      for b = a + 1, nSel do
        if live[b] then
          local dx, dy, dz = px[a] - px[b], py[a] - py[b], pz[a] - pz[b]
          local d = sqrt(dx * dx + dy * dy + dz * dz)
          local d0 = restD[(a - 1) * nSel + b] or d
          if abs(d - d0) > (absTol + relTol * d0) then
            votes[a] = votes[a] + 1
            votes[b] = votes[b] + 1
          end
        end
      end
    end
  end

  local majority = (liveCount - 1) * 0.5
  local dropped = 0
  for a = 1, nSel do
    if live[a] and votes[a] > majority then
      live[a] = false
      dropped = dropped + 1
    end
  end
  return dropped
end

-- How far each node sits from where the fitted rigid frame says it should be.
function S.rejectOutliers(sel, nSel, live, qx, qy, qz, px, py, pz,
                          qcx, qcy, qcz, pcx, pcy, pcz,
                          r11, r12, r13, r21, r22, r23, r31, r32, r33,
                          absFloor, medianScale, maxDropFrac, resid, sorted)
  medianScale = medianScale or 3.0
  maxDropFrac = maxDropFrac or 0.4

  local m = 0
  for s = 1, nSel do
    if live[s] then
      local i = sel[s]
      local ax, ay, az = qx[i] - qcx, qy[i] - qcy, qz[i] - qcz
      local ex = (r11 * ax + r12 * ay + r13 * az) - (px[s] - pcx)
      local ey = (r21 * ax + r22 * ay + r23 * az) - (py[s] - pcy)
      local ez = (r31 * ax + r32 * ay + r33 * az) - (pz[s] - pcz)
      local d = sqrt(ex * ex + ey * ey + ez * ez)
      resid[s] = d
      m = m + 1
      -- insertion sort as we go, the set is never bigger than 32
      local j = m
      while j > 1 and sorted[j - 1] > d do
        sorted[j] = sorted[j - 1]
        j = j - 1
      end
      sorted[j] = d
    else
      resid[s] = -1
    end
  end

  if m < 4 then return 0 end

  local median = sorted[floor((m + 1) * 0.5)]
  local thr = max(absFloor, medianScale * median)

  -- never gut more than a fraction of the set in one go
  local maxDrop = floor(m * maxDropFrac)
  if maxDrop < 1 then return 0 end
  local cutoffIdx = m - maxDrop
  if cutoffIdx >= 1 and sorted[cutoffIdx] > thr then
    thr = sorted[cutoffIdx]
  end

  local dropped = 0
  for s = 1, nSel do
    if live[s] and resid[s] > thr then
      live[s] = false
      dropped = dropped + 1
    end
  end
  return dropped
end

-- Optional virtual-beam softness.
function S.springStep(cx, cy, cz, vx, vy, vz,
                      sel, nSel, live, px, py, pz, restLen, w,
                      stiffness, damping, dt, substeps)
  substeps = substeps or 2
  local h = dt / substeps
  if h <= 0 or h ~= h then return cx, cy, cz, vx, vy, vz end

  for _ = 1, substeps do
    local fx, fy, fz = 0.0, 0.0, 0.0
    for s = 1, nSel do
      if live[s] then
        local dx, dy, dz = px[s] - cx, py[s] - cy, pz[s] - cz
        local d2 = dx * dx + dy * dy + dz * dz
        if d2 > 1e-10 then
          local d = sqrt(d2)
          local invd = 1.0 / d
          local ux, uy, uz = dx * invd, dy * invd, dz * invd
          local stretch = d - restLen[s]
          local k = stiffness * w[s]
          fx = fx + k * stretch * ux
          fy = fy + k * stretch * uy
          fz = fz + k * stretch * uz
        end
      end
    end
    -- Global damper.
    fx = fx - damping * vx
    fy = fy - damping * vy
    fz = fz - damping * vz

    vx = vx + fx * h
    vy = vy + fy * h
    vz = vz + fz * h
    cx = cx + vx * h
    cy = cy + vy * h
    cz = cz + vz * h
  end

  if cx ~= cx or cy ~= cy or cz ~= cz then return nil end
  return cx, cy, cz, vx, vy, vz
end
--[[NODECAM_SOLVER_END]]

-- ---------------------------------------------------------------------------
-- helpers
-- ---------------------------------------------------------------------------

local function cfg()
  -- Settings live in the GE extension so they survive vehicle switches and can
  -- be poked from the console.
  if nodeCamCore and nodeCamCore.settings then return nodeCamCore.settings end
  return {
    nodeCount = 10,
    softness = 0.0,
    stiffness = 900.0,
    damping = 26.0,
    maxSag = 0.25,
    fov = 65,
    outlierResidual = 0.09,
    reattachDist = 0.12,
    reattachInterval = 0.15,
    minSeparation = 0.05,
    searchRadius = 1.2,
    moveSpeed = 1.1,
    fastMultiplier = 3.0,
    lookSensitivity = 0.25,
    keyLookSpeed = 1.2,
    invertYaw = false,
    invertPitch = false,
    banStrikes = 8,
    strikeDecay = 2.0,
    debug = false,
  }
end

local function moveState()
  if nodeCamCore and nodeCamCore.move then return nodeCamCore.move end
  return nil
end

-- ---------------------------------------------------------------------------
-- camera mode
-- ---------------------------------------------------------------------------

function C:init()
  self.baseMode = true      -- we set pos, rot and fov ourselves
  self.register = true      -- show up in the C-key camera cycle
  self.hidden = false

  self.fov = self.fov or 65

  -- vehicle-local rest shape, filled in on attach.
  self.qx, self.qy, self.qz = {}, {}, {}
  self.cid = {}
  self.nNodes = 0
  self.aabb = nil
  self.shapeVid = nil
  self.shapeReady = false

  -- the currently attached node set
  self.sel, self.selLive, self.selW, self.selRest = {}, {}, {}, {}
  self.nSel = 0
  self.px, self.py, self.pz = {}, {}, {}
  -- scratch buffers, reused so the hot path allocates nothing
  self.resid, self.residSorted = {}, {}
  self.restD, self.votes = {}, {}
  self.rawX, self.rawY, self.rawZ = {}, {}, {}

  -- nodes the user picked by hand, and the picker's working set
  self.forced = {}
  self.inSet = {}
  self.pickList, self.pickPX, self.pickPY, self.pickPZ = {}, {}, {}, {}
  self.pickDist = {}
  self.nPick, self.pickTimer, self.hoverNode = 0, 0, nil

  -- last frame's camera basis and fitted rotation, used to move relative to
  -- where the camera is looking
  self.camF = { x = 0, y = 1, z = 0 }
  self.camU = { x = 0, y = 0, z = 1 }
  self.R = nil

  -- Nodes that keep failing the strain test get struck off.
  self.strikes, self.banned = {}, {}
  self.lockedSet = nil
  self.pendingRepick = false

  -- anchor, in vehicle-local coordinates (+X left, +Y back, +Z up, fwd = -Y)
  self.anchor = { x = 0, y = 0, z = 0 }
  self.anchorValid = false
  self.lastAttachAnchor = { x = 0, y = 0, z = 0 }
  self.reattachTimer = 0

  -- look offsets
  self.yaw = 0
  self.pitch = 0

  -- soft-mode state
  self.softPos = nil
  self.softVel = { x = 0, y = 0, z = 0 }

  -- warm-started rotation
  self.haveRot = false

end

function C:reset()
  self.yaw = 0
  self.pitch = 0
  self.debugBroken = false
  self.softPos = nil
  self.softVel.x, self.softVel.y, self.softVel.z = 0, 0, 0
  self.anchorValid = false
  self.shapeReady = false
  self.nSel = 0
end

function C:reloaded()
  self.strikes, self.banned = {}, {}
  self.shapeReady = false
  self.shapeVid = nil
  self.nSel = 0
  self.softPos = nil
end

function C:setFOV(fov)
  if fov then self.fov = fov end
end

function C:setOffset(offset) end

local function normalize3(x, y, z)
  local l = sqrt(x * x + y * y + z * z)
  if l < 1e-6 then return nil end
  return x / l, y / l, z / l
end

-- Work out the vehicle's local axes in world space: +X left, +Y back, +Z up.
function C:resolveFrame(veh, rawX, rawY, rawZ, n)
  local fx, fy, fz, ux, uy, uz, src

  -- 1. the reference node triad, if this build does provide it
  local rn = self.refNodes
  if rn and rn.ref and rn.back and rn.up then
    local ok, r = pcall(function()
      return { o = vec3(veh:getNodePosition(rn.ref)),
               b = vec3(veh:getNodePosition(rn.back)),
               u = vec3(veh:getNodePosition(rn.up)) }
    end)
    if ok and r then
      fx, fy, fz = normalize3(r.o.x - r.b.x, r.o.y - r.b.y, r.o.z - r.b.z)
      ux, uy, uz = normalize3(r.u.x - r.o.x, r.u.y - r.o.y, r.u.z - r.o.z)
      if fx and ux then src = 'refNodes' end
    end
  end

  -- 2. the vehicle object's own direction vectors
  if not src then
    local ok, r = pcall(function()
      return { f = vec3(veh:getDirectionVector()), u = vec3(veh:getDirectionVectorUp()) }
    end)
    if ok and r and r.f and r.u then
      fx, fy, fz = normalize3(r.f.x, r.f.y, r.f.z)
      ux, uy, uz = normalize3(r.u.x, r.u.y, r.u.z)
      if fx and ux then src = 'getDirectionVector' end
    end
  end

  -- 3. its rotation quaternion, turning the object axes into world space
  if not src then
    local ok, r = pcall(function()
      local q = veh:getRotation()
      return { f = q * vec3(0, 1, 0), u = q * vec3(0, 0, 1) }
    end)
    if ok and r and r.f and r.u then
      fx, fy, fz = normalize3(r.f.x, r.f.y, r.f.z)
      ux, uy, uz = normalize3(r.u.x, r.u.y, r.u.z)
      if fx and ux then src = 'getRotation' end
    end
  end

  -- 4.
  if not src and n and n >= 8 then
    local cx, cy = 0, 0
    for i = 1, n do cx = cx + rawX[i]; cy = cy + rawY[i] end
    cx, cy = cx / n, cy / n
    local sxx, syy, sxy = 0, 0, 0
    for i = 1, n do
      local dx, dy = rawX[i] - cx, rawY[i] - cy
      sxx = sxx + dx * dx; syy = syy + dy * dy; sxy = sxy + dx * dy
    end
    local theta = 0.5 * math.atan2(2 * sxy, sxx - syy)
    fx, fy, fz = cos(theta), sin(theta), 0
    ux, uy, uz = 0, 0, 1
    src = 'node cloud shape (front/back may be reversed)'
  end

  if not src then return nil end

  -- the user can spin it 180 degrees if the geometric guess faced backwards
  if self.frameFlip then fx, fy, fz = -fx, -fy, -fz end

  -- make up perpendicular to forward
  local d = ux * fx + uy * fy + uz * fz
  ux, uy, uz = normalize3(ux - d * fx, uy - d * fy, uz - d * fz)
  if not ux then return nil end

  -- vehicle local: +Y is back, so it is the reverse of forward, +Z is up, and
  -- +X = Y cross Z which comes out as left
  local Yx, Yy, Yz = -fx, -fy, -fz
  local Zx, Zy, Zz = ux, uy, uz
  local Xx = Yy * Zz - Yz * Zy
  local Xy = Yz * Zx - Yx * Zz
  local Xz = Yx * Zy - Yy * Zx

  return Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz, src
end

-- Vehicle axes for this frame, cheaply.
function C:frameAxes(veh)
  local a = { self:resolveFrame(veh) }
  if a[1] then self.lastAxes = a; return a end
  return self.lastAxes
end

-- Sweep every node once, resolve the vehicle frame, and store the rest shape
-- in vehicle-local coordinates.
function C:buildShapeGE(veh)
  local okCount, count = pcall(function() return veh:getNodeCount() end)
  if not okCount or type(count) ~= 'number' or count < 4 then
    self.shapeFail = 'veh:getNodeCount() unavailable or returned ' .. tostring(count)
    return false
  end

  local rawX, rawY, rawZ = self.rawX, self.rawY, self.rawZ
  local n = 0
  local okAll = pcall(function()
    for i = 0, count - 1 do
      local p = vec3(veh:getNodePosition(i))
      n = n + 1
      rawX[n], rawY[n], rawZ[n] = p.x, p.y, p.z
    end
  end)
  if not okAll or n < 4 then
    self.shapeFail = string.format('node sweep failed after %d of %d nodes', n, count)
    return false
  end

  local Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz, src =
    self:resolveFrame(veh, rawX, rawY, rawZ, n)
  if not Xx then
    self.shapeFail = 'could not establish the vehicle orientation by any means'
    return false
  end

  if self.frameSource ~= src then
    self.frameSource = src
    if log then log('I', 'nodeCam', 'vehicle frame from: ' .. tostring(src)) end
  end

  local qx, qy, qz, cid = self.qx, self.qy, self.qz, self.cid
  local minx, miny, minz = huge, huge, huge
  local maxx, maxy, maxz = -huge, -huge, -huge

  for i = 1, n do
    local px, py, pz = rawX[i], rawY[i], rawZ[i]
    local lx = px * Xx + py * Xy + pz * Xz
    local ly = px * Yx + py * Yy + pz * Yz
    local lz = px * Zx + py * Zy + pz * Zz
    qx[i], qy[i], qz[i] = lx, ly, lz
    cid[i] = i - 1
    if lx < minx then minx = lx end
    if ly < miny then miny = ly end
    if lz < minz then minz = lz end
    if lx > maxx then maxx = lx end
    if ly > maxy then maxy = ly end
    if lz > maxz then maxz = lz end
  end

  self.lastAxes = { Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz }
  self.shapeFail = nil
  self.nNodes = n
  self.aabb = { minx = minx, miny = miny, minz = minz,
                maxx = maxx, maxy = maxy, maxz = maxz }
  self.shapeReady = true
  return true
end

-- Default anchor: roughly where a dash cam sits, derived from the node cloud
-- so it lands sensibly on anything from a hatchback to a semi.
function C:defaultAnchor()
  local a = self.aabb
  if not a then return 0, -0.2, 0.8 end
  local ry = a.maxy - a.miny
  local rz = a.maxz - a.minz
  return 0.0, a.miny + ry * 0.34, a.minz + rz * 0.62
end

function C:applyPreset(name)
  local a = self.aabb
  if not a then return end
  local rx, ry, rz = a.maxx - a.minx, a.maxy - a.miny, a.maxz - a.minz
  local cx = (a.maxx + a.minx) * 0.5
  local x, y, z
  if name == 'hood' then
    x, y, z = cx, a.miny + ry * 0.14, a.minz + rz * 0.58
  elseif name == 'bumper' then
    x, y, z = cx, a.miny + ry * 0.03, a.minz + rz * 0.22
  elseif name == 'roof' then
    x, y, z = cx, a.miny + ry * 0.42, a.maxz
  elseif name == 'tail' then
    x, y, z = cx, a.maxy - ry * 0.03, a.minz + rz * 0.45
  elseif name == 'wheelLeft' then
    -- pulled in and up a little so it starts on the arch rather than the tyre
    x, y, z = a.maxx - rx * 0.05, a.miny + ry * 0.24, a.minz + rz * 0.34
  elseif name == 'wheelRight' then
    x, y, z = a.minx + rx * 0.05, a.miny + ry * 0.24, a.minz + rz * 0.34
  else -- dash
    x, y, z = self:defaultAnchor()
  end
  self.anchor.x, self.anchor.y, self.anchor.z = x, y, z
  self.anchorValid = true
  self.nSel = 0
  self.softPos = nil
end

-- Pick the node set.
function C:selectNodes()
  local c = cfg()
  local want = max(4, min(32, floor(c.nodeCount or 10)))
  local radius = c.searchRadius or 1.2
  local minSep = c.minSeparation or 0.05
  local minSep2 = minSep * minSep

  local ax, ay, az = self.anchor.x, self.anchor.y, self.anchor.z
  local qx, qy, qz = self.qx, self.qy, self.qz


  -- gather candidates inside the radius, growing it if the car is sparse
  local cand, candD = {}, {}
  local nc = 0
  for attempt = 1, 4 do
    local r2 = (radius * attempt) * (radius * attempt)
    nc = 0
    for i = 1, self.nNodes do
      if not self.banned[i] then
        local dx, dy, dz = qx[i] - ax, qy[i] - ay, qz[i] - az
        local d2 = dx * dx + dy * dy + dz * dz
        if d2 <= r2 then
          nc = nc + 1
          cand[nc] = i
          candD[i] = d2
        end
      end
    end
    if nc >= want * 2 then break end
  end

  if nc < 4 then return false end

  -- the radius only ever grows, so nc cannot shrink, but clear the tail anyway
  -- so table.sort can never see a leftover from a previous pass
  for k = nc + 1, #cand do cand[k] = nil end
  table.sort(cand, function(m, n) return candD[m] < candD[n] end)
  -- table.sort may leave stale tail entries, so only read the first nc
  local sel, selW, selRest, selLive = self.sel, self.selW, self.selRest, self.selLive
  local picked = 0

  -- anything you picked by hand goes in first, whatever the distance
  for i in pairs(self.forced) do
    if not self.banned[i] and picked < want then
      picked = picked + 1
      sel[picked] = i
      local dx, dy, dz = qx[i] - ax, qy[i] - ay, qz[i] - az
      local d2 = dx * dx + dy * dy + dz * dz
      selRest[picked] = sqrt(d2)
      selW[picked] = 1.0 / (d2 + 0.02)
      selLive[picked] = true
    end
  end
  local forcedCount = picked

  for k = 1, nc do
    if picked >= want then break end
    local i = cand[k]
    local ok = not self.forced[i]
    for s = 1, picked do
      local j = sel[s]
      local dx, dy, dz = qx[i] - qx[j], qy[i] - qy[j], qz[i] - qz[j]
      if (dx * dx + dy * dy + dz * dz) < minSep2 then ok = false; break end
    end
    if ok then
      picked = picked + 1
      sel[picked] = i
      local d2 = candD[i] or 0
      local d = sqrt(d2)
      selRest[picked] = d
      -- inverse square falloff: nodes right under the camera dominate the fit
      selW[picked] = 1.0 / (d2 + 0.02)
      selLive[picked] = true
    end
  end

  if picked < 4 and forcedCount < picked then return false end
  if picked < 4 then return false end

  self.nSel = picked
  self.pendingRepick = false
  S.buildRestDistances(sel, picked, qx, qy, qz, self.restD)
  self.lastAttachAnchor.x, self.lastAttachAnchor.y, self.lastAttachAnchor.z = ax, ay, az
  self.softPos = nil
  return true
end

-- Rebuild the weights and beam rest lengths after the set changes by hand.
function C:refreshSet()
  local ax, ay, az = self.anchor.x, self.anchor.y, self.anchor.z
  local qx, qy, qz = self.qx, self.qy, self.qz
  for s = 1, self.nSel do
    local i = self.sel[s]
    local dx, dy, dz = qx[i] - ax, qy[i] - ay, qz[i] - az
    local d2 = dx * dx + dy * dy + dz * dz
    self.selRest[s] = sqrt(d2)
    self.selW[s] = 1.0 / (d2 + 0.02)
    self.selLive[s] = true
  end
  S.buildRestDistances(self.sel, self.nSel, qx, qy, qz, self.restD)
  self.softPos = nil
end

-- Toggle one node in or out of the attached set.
function C:saveSlot(vid)
  if not (nodeCamCore and nodeCamCore.saveSlot) then return end
  local nodes = {}
  for i = 1, self.nSel do nodes[i] = self.sel[i] end
  nodeCamCore.saveSlot(vid, nodeCamCore.slotIndex(vid), {
    ax = self.anchor.x, ay = self.anchor.y, az = self.anchor.z,
    nodes = nodes, locked = cfg().lock == true,
    yaw = self.yaw, pitch = self.pitch, fov = cfg().fov,
  })
end

function C:loadSlot(vid, i)
  local d = nodeCamCore and nodeCamCore.getSlot and nodeCamCore.getSlot(vid, i)
  if not d then return false end
  self.anchor.x, self.anchor.y, self.anchor.z = d.ax, d.ay, d.az
  self.anchorValid = true
  self.yaw, self.pitch = d.yaw or 0, d.pitch or 0
  local n = 0
  for k, v in ipairs(d.nodes or {}) do
    if v <= self.nNodes then n = n + 1; self.sel[n] = v end
  end
  self.nSel = n
  cfg().lock = d.locked == true
  if d.fov then cfg().fov = d.fov end
  self.lockedSet = nil
  if n >= 4 then self:refreshSet() end
  if d.locked then self:snapshotLock() end
  return true
end

function C:cycleSlot(vid)
  local cur = nodeCamCore.slotIndex(vid)
  self:saveSlot(vid)
  local nxt = (cur % 3) + 1
  nodeCamCore.setSlotIndex(vid, nxt)
  if not self:loadSlot(vid, nxt) then self:saveSlot(vid) end
  nodeCamCore.msg(string.format('nodeCam: cam %d of 3, nodes %s',
    nxt, cfg().lock and 'locked' or 'auto'), 2)
end

function C:snapshotLock()
  local t = {}
  for s = 1, self.nSel do t[s] = self.sel[s] end
  self.lockedSet = t
end

function C:restoreLock()
  local t = self.lockedSet
  if not t or #t < 4 then return false end
  for _, i in ipairs(t) do
    if i > self.nNodes then self.lockedSet = nil; return false end
  end
  for s = 1, #t do self.sel[s] = t[s] end
  self.nSel = #t
  self:refreshSet()
  return true
end

function C:toggleNode(i)
  if not i or i < 1 or i > self.nNodes then return end

  local at = nil
  for s = 1, self.nSel do if self.sel[s] == i then at = s; break end end

  local locked = cfg().lock == true

  if at then
    for s = at, self.nSel - 1 do self.sel[s] = self.sel[s + 1] end
    self.nSel = self.nSel - 1
    if not locked then
      self.forced[i] = nil
      self.banned[i] = true
    end
  else
    self.nSel = self.nSel + 1
    self.sel[self.nSel] = i
    if not locked then
      self.forced[i] = true
      self.banned[i] = nil
      self.strikes[i] = 0
    end
  end

  if self.nSel >= 4 then self:refreshSet() end
  if locked then self:snapshotLock() end
  if nodeCamCore then
    nodeCamCore.msg(string.format('nodeCam: %s node, %d attached',
      at and 'removed' or 'added', self.nSel))
  end
end

-- Keep a running tally of which nodes keep failing.
function C:updateStrikes(dt, live)
  local c = cfg()

  -- Locked: nothing is struck off.
  if c.lock then return end

  local liveCount, newBans = S.updateStrikes(self.sel, self.nSel, live,
    self.strikes, self.banned, c.banStrikes or 8, c.strikeDecay or 2.0, dt)
  local newBan = newBans > 0

  -- A newly banned node leaves a hole in the set, and any dead node is weight
  -- we are not using, so line up a fresh pick.
  if newBan or liveCount < self.nSel then
    self.pendingRepick = true
  end
  if liveCount < 4 then
    self.nSel = 0
    self.reattachTimer = 1e9
  end
end

-- The game's own camera movement bindings write into MoveManager.
local MOVE_SOURCES = {
  { 'forward', 'backward', 'left', 'right', 'up', 'down' },
  { 'moveForward', 'moveBackward', 'moveLeft', 'moveRight', 'moveUp', 'moveDown' },
  { 'movementForward', 'movementBackward', 'movementLeft', 'movementRight',
    'movementUp', 'movementDown' },
}

local function readMove(src)
  if src == nil then return 0, 0, 0, nil end
  for _, e in ipairs(MOVE_SOURCES) do
    local f = (tonumber(src[e[1]]) or 0) - (tonumber(src[e[2]]) or 0)
    local r = (tonumber(src[e[4]]) or 0) - (tonumber(src[e[3]]) or 0)
    local u = (tonumber(src[e[5]]) or 0) - (tonumber(src[e[6]]) or 0)
    if f ~= 0 or r ~= 0 or u ~= 0 then return f, r, u, e[1] end
  end
  return 0, 0, 0, nil
end

-- Silently swallowing an error here is how mouse look died once already: a
-- refactor deleted the reader, the pcall caught the nil call, and look simply
-- returned zero with nothing in the log to say why.
function C:lookFailed(err)
  local msg = tostring(err)
  if self.lastLookErr ~= msg then
    self.lastLookErr = msg
    if log then log('E', 'nodeCam', 'look input read failed: ' .. msg) end
  end
end

function C:handleMovement(dt)
  local mv = moveState()
  if not mv then return false end
  local c = cfg()

  -- console nudge, lets you prove anchor movement works without a keybind
  if nodeCamCore and nodeCamCore.consumeNudge then
    local n = nodeCamCore.consumeNudge()
    if n then
      self.anchor.x = self.anchor.x + (n.x or 0)
      self.anchor.y = self.anchor.y + (n.y or 0)
      self.anchor.z = self.anchor.z + (n.z or 0)
      if not c.lock then self.pendingRepick = true end
      return true
    end
  end

  local fwdAmt, rightAmt, upAmt = 0, 0, 0
  if c.nativeMove ~= false and MoveManager ~= nil then
    local ok, f, r, u, label = pcall(readMove, MoveManager)
    if ok and (f ~= 0 or r ~= 0 or u ~= 0) then
      -- clamped, so moveSpeed means the same thing whatever the source carries
      fwdAmt = f > 1 and 1 or (f < -1 and -1 or f)
      rightAmt = r > 1 and 1 or (r < -1 and -1 or r)
      upAmt = u > 1 and 1 or (u < -1 and -1 or u)
      if nodeCamCore then nodeCamCore.moveSource = 'MoveManager.' .. tostring(label) end
    end
  end

  if fwdAmt == 0 and rightAmt == 0 and upAmt == 0 then return false end

  local speed = (c.moveSpeed or 1.1) * (mv.fast and (c.fastMultiplier or 3.0) or 1.0) * dt
  fwdAmt, rightAmt, upAmt = fwdAmt * speed, rightAmt * speed, upAmt * speed

  local dx, dy, dz
  if self.R then
    -- fly style: forward is wherever the camera is pointing, pitch included
    local R = self.R
    dx, dy, dz = S.moveToLocal(fwdAmt, rightAmt, upAmt,
      self.camF.x, self.camF.y, self.camF.z,
      self.camU.x, self.camU.y, self.camU.z,
      R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8], R[9])
  else
    -- before the first fit lands, fall back to the vehicle's own axes
    dx, dy, dz = -rightAmt, -fwdAmt, upAmt
  end

  self.anchor.x = self.anchor.x + dx
  self.anchor.y = self.anchor.y + dy
  self.anchor.z = self.anchor.z + dz

  local a = self.aabb
  if a then
    local m = 0.9
    self.anchor.x = max(a.minx - m, min(a.maxx + m, self.anchor.x))
    self.anchor.y = max(a.miny - m, min(a.maxy + m, self.anchor.y))
    self.anchor.z = max(a.minz - m, min(a.maxz + m, self.anchor.z))
  end
  return true
end

-- Where mouse look arrives is not something I could pin down from outside the
-- game, so rather than bet on one field name we read every source that
-- plausibly carries it and take whichever is actually moving.
local LOOK_SOURCES = {
  -- { yaw field, pitch field, per-frame delta or a rate }
  { 'yawRelative',    'pitchRelative',    'delta' },
  { 'yawRelativeRaw', 'pitchRelativeRaw', 'delta' },
  { 'camx',           'camy',             'delta' },
  { 'yaw',            'pitch',            'rate'  },
}

local LOOK_PAIRS = {
  { 'yawLeftSpeed',  'yawRightSpeed',  'yaw'   },
  { 'yawLeft',       'yawRight',       'yaw'   },
  { 'pitchUpSpeed',  'pitchDownSpeed', 'pitch' },
  { 'pitchUp',       'pitchDown',      'pitch' },
}

-- The input source may be userdata rather than a table, and reading a field it
-- does not have is not guaranteed to be safe, so every access is guarded on
-- its own.
local function num(src, k)
  local ok, v = pcall(function() return src[k] end)
  if ok then return tonumber(v) end
  return nil
end

local function readLook(src, dt, sens, keyRate)
  if src == nil then return 0, 0, nil end
  local yawIn, pitchIn, used = 0, 0, nil

  for _, e in ipairs(LOOK_SOURCES) do
    local y, p = num(src, e[1]), num(src, e[2])
    if (y and y ~= 0) or (p and p ~= 0) then
      local scale = (e[3] == 'delta') and sens or (keyRate * 60)
      yawIn = yawIn + (y or 0) * scale
      pitchIn = pitchIn + (p or 0) * scale
      used = e[1]
      -- a delta is consumed, or it would be applied again next frame
      if e[3] == 'delta' then
        if y then pcall(function() src[e[1]] = 0 end) end
        if p then pcall(function() src[e[2]] = 0 end) end
      end
      break
    end
  end

  for _, e in ipairs(LOOK_PAIRS) do
    local a, b = num(src, e[1]), num(src, e[2])
    if (a and a ~= 0) or (b and b ~= 0) then
      local v = ((a or 0) - (b or 0)) * keyRate
      if e[3] == 'yaw' then yawIn = yawIn + v else pitchIn = pitchIn + v end
      used = used or e[1]
    end
  end

  return yawIn, pitchIn, used
end

function C:handleLook(dt, data)
  local c = cfg()

  if nodeCamCore and nodeCamCore.consumeLookReset and nodeCamCore.consumeLookReset() then
    self.yaw, self.pitch = 0, 0
  end

  local sens = c.lookSensitivity or 0.25
  local keyRate = (c.keyLookSpeed or 1.2) * dt

  local yawIn, pitchIn = 0, 0
  if nodeCamCore and nodeCamCore.consumeLook then
    local l = nodeCamCore.consumeLook()
    if l then yawIn, pitchIn = yawIn + (l.yaw or 0), pitchIn + (l.pitch or 0) end
  end
  local src
  -- the camera context's own move table first, then the global
  if data and type(data.move) == 'table' then src = data.move end
  local ok1, y1, p1, u1 = pcall(readLook, src, dt, sens, keyRate)
  if not ok1 then self:lookFailed(y1); y1, p1, u1 = 0, 0, nil end
  local y2, p2, u2 = 0, 0, nil
  if MoveManager ~= nil then
    local ok2, a, b, c2 = pcall(readLook, MoveManager, dt, sens, keyRate)
    if ok2 then y2, p2, u2 = a, b, c2 else self:lookFailed(a) end
  end
  yawIn = yawIn + (y1 or 0) + (y2 or 0)
  pitchIn = pitchIn + (p1 or 0) + (p2 or 0)

  if nodeCamCore then
    nodeCamCore.lookSource = u1 and ('data.move.' .. u1)
      or (u2 and ('MoveManager.' .. u2)) or nodeCamCore.lookSource
  end

  -- Both axes came in reversed on the input source that turned out to be live,
  -- so both are negated by default.
  local yawSign = (c.invertYaw and 1) or -1
  local pitchSign = (c.invertPitch and -1) or 1

  self.yaw = self.yaw + yawIn * yawSign
  self.pitch = self.pitch + pitchIn * pitchSign

  local lim = 1.45
  if self.pitch > lim then self.pitch = lim end
  if self.pitch < -lim then self.pitch = -lim end
end

-- Fallback view when we have no usable node fit yet.
function C:fallback(data, reason)
  reason = reason or 'unknown'
  if self.fallbackReason ~= reason then
    self.fallbackReason = reason
    if nodeCamCore then nodeCamCore.msg('nodeCam: ' .. reason, 4) end
  end

  local veh = data.veh
  local Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz = nil
  if veh then Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz = self:resolveFrame(veh) end

  if Xx then
    local fwd = vec3(-Yx, -Yy, -Yz)
    local up = vec3(Zx, Zy, Zz)
    data.res.pos = data.pos + up * 1.4 - fwd * 4.0
    local okq, q = pcall(quatFromDir, fwd, up)
    if okq and q then data.res.rot = q end
  else
    data.res.pos = data.pos + vec3(0, 0, 3)
  end
  data.res.fov = self.fov
  return true
end

-- Camera placed straight off the vehicle axes, ignoring the nodes entirely.
function C:steady(data, veh, why)
  local a = self:frameAxes(veh)
  if not a or not a[1] then return self:fallback(data, why or 'no vehicle axes') end

  local Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz = a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9]
  local ax, ay, az = self.anchor.x, self.anchor.y, self.anchor.z
  local ox = Xx * ax + Yx * ay + Zx * az
  local oy = Xy * ax + Yy * ay + Zy * az
  local oz = Xz * ax + Yz * ay + Zz * az

  local fx, fy, fz, ux, uy, uz =
    S.applyLook(-Yx, -Yy, -Yz, Zx, Zy, Zz, self.yaw, self.pitch)

  local fwd, up = vec3(fx, fy, fz), vec3(ux, uy, uz)
  local okq, q = pcall(quatFromDir, fwd, up)
  data.res.pos = data.pos + vec3(ox, oy, oz)
  if okq and q then data.res.rot = q end
  data.res.fov = self.fov

  self.camF.x, self.camF.y, self.camF.z = fx, fy, fz
  self.camU.x, self.camU.y, self.camU.z = ux, uy, uz
  self.R = self.R or {}
  self.R[1], self.R[2], self.R[3] = Xx, Yx, Zx
  self.R[4], self.R[5], self.R[6] = Xy, Yy, Zy
  self.R[7], self.R[8], self.R[9] = Xz, Yz, Zz

  if cfg().debug then
    self:drawPicker(data, veh, ox, oy, oz, fx, fy, fz, self.selLive)
  end
  self.fallbackReason = nil
  return true
end

function C:update(data)
  local dt = data.dt or 0.016
  local veh = data.veh
  if not veh then return false end

  -- Pull up the state module the first time we run, in case the mod loader
  -- never got around to it.
  if not self.coreChecked then
    self.coreChecked = true
    if not nodeCamCore then
      pcall(function() extensions.load('nodeCamCore') end)
    end
  end

  self.vid = data.vid
  local c = cfg()

  if nodeCamCore then
    nodeCamCore.live = true
    nodeCamCore.liveInfo = nodeCamCore.liveInfo or {}
    local li = nodeCamCore.liveInfo
    li.dt, li.vid, li.nSel, li.nNodes = dt, data.vid, self.nSel, self.nNodes
    li.yaw, li.pitch = self.yaw, self.pitch
    li.fallback, li.shapeFail = self.fallbackReason, self.shapeFail
    li.hasRefNodes = (self.refNodes ~= nil)
    li.shapeReady = self.shapeReady
    li.frameSource = self.frameSource
    li.locked = (cfg().lock == true)
    li.hover = self.hoverNode
    li.nPick = self.nPick

    -- Watch every numeric field on both candidate input tables and remember
    -- which ones actually move.
    self.seen = self.seen or {}
    local function watch(tbl, label)
      if type(tbl) ~= 'table' then return end
      for k, v in pairs(tbl) do
        if type(v) == 'number' and v ~= 0 then
          local key = label .. '.' .. tostring(k)
          local a = math.abs(v)
          if not self.seen[key] or a > self.seen[key] then self.seen[key] = a end
        end
      end
    end
    pcall(watch, data.move, 'data.move')
    pcall(watch, MoveManager, 'MoveManager')
    local list = {}
    for k, v in pairs(self.seen) do list[#list + 1] = string.format('%s(max %.3f)', k, v) end
    table.sort(list)
    li.inputSeen = list
    li.anchorX, li.anchorY, li.anchorZ = self.anchor.x, self.anchor.y, self.anchor.z
    if not li.dataKeys then
      li.dataKeys = {}
      for k, v in pairs(data) do li.dataKeys[#li.dataKeys + 1] = k .. '(' .. type(v) .. ')' end
      table.sort(li.dataKeys)
    end
  end
  self.fov = c.fov or self.fov

  -- (re)build the rest shape when the vehicle changes
  if not self.shapeReady or self.shapeVid ~= data.vid then
    self.shapeVid = data.vid
    self.shapeReady = false
    self.anchorValid = false
    self.nSel = 0
    self.strikes, self.banned = {}, {}
  end

  if not self.shapeReady then
    -- Retry on a timer rather than every frame.
    self.shapeRetry = (self.shapeRetry or 0) - dt
    if self.shapeRetry <= 0 then
      self.shapeRetry = 1.0
      self:buildShapeGE(veh)
    end
    if not self.shapeReady then
      return self:fallback(data, self.shapeFail or 'could not map the vehicle nodes')
    end
  end

  if not self.anchorValid then
    local ax, ay, az = self:defaultAnchor()
    -- restore a previously parked anchor for this vehicle if there is one
    if nodeCamCore and nodeCamCore.getAnchor then
      local saved = nodeCamCore.getAnchor(data.vid)
      if saved then ax, ay, az = saved.x, saved.y, saved.z end
    end
    self.anchor.x, self.anchor.y, self.anchor.z = ax, ay, az
    self.anchorValid = true
    self.nSel = 0
  end

  -- pending preset from a keybind or the console
  if nodeCamCore and nodeCamCore.consumePreset then
    local p = nodeCamCore.consumePreset()
    if p then self:applyPreset(p) end
  end

  self:handleLook(dt, data)
  local moved = self:handleMovement(dt)

  -- decide whether to re-pick the node set
  self.reattachTimer = self.reattachTimer + dt
  local locked = c.lock == true

  if locked then
    if self.lockedSet == nil then self:snapshotLock() end
    if self.nSel < 4 then self:restoreLock() end
  end

  -- Bypass skips selection entirely. An empty set does not: it still needs a
  -- chance to re-pick further down, or unlocking with no nodes would strand you
  -- in the steady view forever.
  if c.bypass then return self:steady(data, veh) end

  local needReattach = (self.nSel < 4) and not locked or (self.pendingRepick and not locked)
  if moved and not needReattach and not locked then
    local dx = self.anchor.x - self.lastAttachAnchor.x
    local dy = self.anchor.y - self.lastAttachAnchor.y
    local dz = self.anchor.z - self.lastAttachAnchor.z
    local moveD = sqrt(dx * dx + dy * dy + dz * dz)
    if moveD > (c.reattachDist or 0.12) then needReattach = true end
  end
  if nodeCamCore and nodeCamCore.consumeClear and nodeCamCore.consumeClear() then
    self.nSel = 0
    self.lockedSet = {}
    self.forced = {}
  end
  if nodeCamCore and nodeCamCore.consumeCycleSlot and nodeCamCore.consumeCycleSlot() then
    self:cycleSlot(data.vid)
  end
  if nodeCamCore and nodeCamCore.consumeToggleNode and nodeCamCore.consumeToggleNode() then
    if self.hoverNode then
      self:toggleNode(self.hoverNode)
    elseif log then
      log('W', 'nodeCam', 'nothing under the crosshair, turn on debug and aim at a node')
    end
  end
  if nodeCamCore and nodeCamCore.consumeFlip and nodeCamCore.consumeFlip() then
    self.frameFlip = not self.frameFlip
    self.shapeReady = false      -- rebuild the whole map in the new orientation
    self.anchorValid = false
    self.nSel = 0
  end
  if nodeCamCore and nodeCamCore.consumeClearBans and nodeCamCore.consumeClearBans() then
    self.strikes, self.banned = {}, {}
    needReattach = true
  end

  if needReattach and self.reattachTimer >= (c.reattachInterval or 0.15) then
    self.reattachTimer = 0
    if not self:selectNodes() then
      return self:steady(data, veh)
    end
  end

  if self.nSel < 4 then return self:steady(data, veh) end

  -- read the live node positions, as offsets from the vehicle origin
  local sel, px, py, pz, live = self.sel, self.px, self.py, self.pz, self.selLive
  local cidMap = self.cid
  local okRead = pcall(function()
    for s = 1, self.nSel do
      local p = vec3(veh:getNodePosition(cidMap[sel[s]]))
      px[s], py[s], pz[s] = p.x, p.y, p.z
      live[s] = true
    end
  end)
  if not okRead then
    self.nSel = 0
    return self:steady(data, veh)
  end

  -- First pass: drop nodes whose virtual beams to the rest of the set have
  -- gone obviously wrong.
  S.strainReject(self.nSel, live, px, py, pz, self.restD,
    c.strainAbsTol or 0.03, c.strainRelTol or 0.12, self.votes)

  -- Second pass: fit, drop whatever is still off, fit again on what is left
  local ok, qcx, qcy, qcz, pcx, pcy, pcz,
        r11, r12, r13, r21, r22, r23, r31, r32, r33 =
    S.fit(sel, self.nSel, live, self.qx, self.qy, self.qz, px, py, pz, self.selW)

  if ok then
    local dropped = S.rejectOutliers(sel, self.nSel, live,
      self.qx, self.qy, self.qz, px, py, pz,
      qcx, qcy, qcz, pcx, pcy, pcz,
      r11, r12, r13, r21, r22, r23, r31, r32, r33,
      c.outlierResidual or 0.09, c.outlierMedianScale or 3.0,
      c.maxDropFraction or 0.4, self.resid, self.residSorted)

    if dropped > 0 then
      ok, qcx, qcy, qcz, pcx, pcy, pcz,
      r11, r12, r13, r21, r22, r23, r31, r32, r33 =
        S.fit(sel, self.nSel, live, self.qx, self.qy, self.qz, px, py, pz, self.selW)
    end
  end

  self:updateStrikes(dt, live)

  if not ok then
    self.nSel = 0
    self.reattachTimer = 1e9  -- re-pick on the very next frame, skip the throttle
    return self:steady(data, veh)
  end

  -- where the anchor lands once the fitted frame is applied
  local lax, lay, laz = self.anchor.x - qcx, self.anchor.y - qcy, self.anchor.z - qcz
  local tx = pcx + (r11 * lax + r12 * lay + r13 * laz)
  local ty = pcy + (r21 * lax + r22 * lay + r23 * laz)
  local tz = pcz + (r31 * lax + r32 * lay + r33 * laz)

  local finalX, finalY, finalZ = tx, ty, tz

  -- optional virtual-beam softness
  local softness = c.softness or 0
  if softness > 0.001 then
    if not self.softPos then
      self.softPos = { x = tx, y = ty, z = tz }
      self.softVel.x, self.softVel.y, self.softVel.z = 0, 0, 0
      -- rest lengths measured against the rigid target, so the springs sit at
      -- equilibrium exactly where the rigid solution puts the camera
      for s = 1, self.nSel do
        local dx, dy, dz = px[s] - tx, py[s] - ty, pz[s] - tz
        self.selRest[s] = sqrt(dx * dx + dy * dy + dz * dz)
      end
    end

    local ncx, ncy, ncz, nvx, nvy, nvz = S.springStep(
      self.softPos.x, self.softPos.y, self.softPos.z,
      self.softVel.x, self.softVel.y, self.softVel.z,
      sel, self.nSel, live, px, py, pz, self.selRest, self.selW,
      c.stiffness or 900.0, c.damping or 26.0, min(dt, 0.05), 3)

    if ncx then
      -- leash: never let the soft camera wander far from the rigid answer
      local maxSag = c.maxSag or 0.25
      local dx, dy, dz = ncx - tx, ncy - ty, ncz - tz
      local d = sqrt(dx * dx + dy * dy + dz * dz)
      if d > maxSag then
        local k = maxSag / d
        ncx, ncy, ncz = tx + dx * k, ty + dy * k, tz + dz * k
        nvx, nvy, nvz = nvx * 0.5, nvy * 0.5, nvz * 0.5
      end
      self.softPos.x, self.softPos.y, self.softPos.z = ncx, ncy, ncz
      self.softVel.x, self.softVel.y, self.softVel.z = nvx, nvy, nvz
      finalX = tx + (ncx - tx) * softness
      finalY = ty + (ncy - ty) * softness
      finalZ = tz + (ncz - tz) * softness
    else
      self.softPos = nil
    end
  else
    self.softPos = nil
  end

  -- Orientation straight out of the fitted frame.
  local fx, fy, fz = -r12, -r22, -r32
  local ux, uy, uz = r13, r23, r33

  -- Look offsets applied as two explicit axis rotations, never as one euler
  -- triple, so no combination of yaw and pitch can roll the horizon.
  fx, fy, fz, ux, uy, uz = S.applyLook(fx, fy, fz, ux, uy, uz, self.yaw, self.pitch)

  local fwd = vec3(fx, fy, fz)
  local up = vec3(ux, uy, uz)
  local flen = fwd:length()
  if flen < 1e-5 then return self:fallback(data, 'fitted frame has no forward axis') end
  fwd = fwd / flen
  local ulen = up:length()
  if ulen < 1e-5 then up = vec3(0, 0, 1) else up = up / ulen end

  local okRot, rot = pcall(quatFromDir, fwd, up)
  if not okRot or not rot then
    okRot, rot = pcall(quatFromDir, fwd)
    if not okRot or not rot then return self:fallback(data, 'quatFromDir unavailable') end
  end

  if self.fallbackReason then
    if nodeCamCore then nodeCamCore.msg('nodeCam: attached') end
    self.fallbackReason = nil
  end

  data.res.pos = data.pos + vec3(finalX, finalY, finalZ)
  data.res.rot = rot
  data.res.fov = self.fov

  -- remember the basis so next frame's movement can be relative to the view
  self.camF.x, self.camF.y, self.camF.z = fwd.x, fwd.y, fwd.z
  self.camU.x, self.camU.y, self.camU.z = up.x, up.y, up.z
  self.R = self.R or {}
  self.R[1], self.R[2], self.R[3] = r11, r12, r13
  self.R[4], self.R[5], self.R[6] = r21, r22, r23
  self.R[7], self.R[8], self.R[9] = r31, r32, r33

  if nodeCamCore and nodeCamCore.setAnchor then
    nodeCamCore.setAnchor(data.vid, self.anchor.x, self.anchor.y, self.anchor.z)
  end

  if c.debug then
    self:drawPicker(data, veh, finalX, finalY, finalZ, fwd.x, fwd.y, fwd.z, live)
  else
    self.hoverNode = nil
    if nodeCamCore then nodeCamCore.hoverNode = nil end
  end

  return true
end

-- Build the list of nodes near enough to be worth drawing and aiming at.
function C:buildPickList(dt)
  local c = cfg()
  self.pickTimer = self.pickTimer - dt
  if self.nPick > 0 and self.pickTimer > 0 then return end
  self.pickTimer = 0.2

  local r2 = (c.pickRadius or 2.5) ^ 2
  local maxN = c.maxDrawnNodes or 220
  local ax, ay, az = self.anchor.x, self.anchor.y, self.anchor.z
  local qx, qy, qz = self.qx, self.qy, self.qz
  local list, dist = self.pickList, self.pickDist
  local n = 0

  for i = 1, self.nNodes do
    local dx, dy, dz = qx[i] - ax, qy[i] - ay, qz[i] - az
    local d = dx * dx + dy * dy + dz * dz
    if d <= r2 then
      n = n + 1
      list[n] = i
      dist[i] = d
    end
  end
  for k = n + 1, #list do list[k] = nil end

  -- Sort before capping.
  if n > maxN then
    table.sort(list, function(a, b) return dist[a] < dist[b] end)
    for k = maxN + 1, n do list[k] = nil end
    n = maxN
  end
  self.nPick = n
end

-- Draw every nearby node, aim at one, and report which is under the crosshair.
function C:drawPicker(data, veh, camX, camY, camZ, fx, fy, fz, live)
  if not debugDrawer or self.debugBroken then return end
  local c = cfg()

  self:buildPickList(data.dt or 0.016)
  if self.nPick == 0 then return end

  local list, px, py, pz = self.pickList, self.pickPX, self.pickPY, self.pickPZ
  local cidMap = self.cid

  local okRead = pcall(function()
    for k = 1, self.nPick do
      local p = vec3(veh:getNodePosition(cidMap[list[k]]))
      px[k], py[k], pz[k] = p.x, p.y, p.z
    end
  end)
  if not okRead then return end

  -- which of these are currently attached, and are they pulling their weight
  local inSet = self.inSet
  for k in pairs(inSet) do inSet[k] = nil end
  for s = 1, self.nSel do
    inSet[self.sel[s]] = live[s] and 2 or 1
  end

  -- crosshair pick, straight down the camera's own forward axis
  local hit = S.pickAlongRay(self.nPick, px, py, pz, camX, camY, camZ,
    fx, fy, fz, c.pickSpread or 0.045, c.pickRadiusNear or 0.08)
  self.hoverNode = hit and list[hit] or nil
  if nodeCamCore then nodeCamCore.hoverNode = self.hoverNode end

  local ok = pcall(function()
    local base = data.pos
    local camW = base + vec3(camX, camY, camZ)

    for k = 1, self.nPick do
      local i = list[k]
      local nodeW = base + vec3(px[k], py[k], pz[k])
      local state = inSet[i]
      local col, rad
      if k == hit then
        col, rad = ColorF(1.0, 1.0, 1.0, 1.0), 0.034
      elseif state == 2 then
        col, rad = ColorF(0.15, 1.0, 0.35, 0.9), 0.022
      elseif state == 1 then
        col, rad = ColorF(1.0, 0.65, 0.1, 0.85), 0.022
      else
        col, rad = ColorF(0.85, 0.15, 0.15, 0.45), 0.012
      end
      debugDrawer:drawSphere(nodeW:toPoint3F(), rad, col)
      if state == 2 then
        debugDrawer:drawLine(camW:toPoint3F(), nodeW:toPoint3F(),
          ColorF(0.15, 1.0, 0.35, 0.5))
      end
    end
  end)

  if not ok then
    self.debugBroken = true
    log('W', 'nodeCam', 'debug drawing failed on this build, turning it off')
  end
end

-- Guard against exactly the edit mistake that broke mouse look: if an internal
-- the update loop depends on is not here, say so at load rather than behaving
-- like the feature simply does nothing.
for name, fn in pairs({ readLook = readLook, readMove = readMove,
                        normalize3 = normalize3, num = num }) do
  if type(fn) ~= 'function' then
    if log then log('E', 'nodeCam', 'internal function missing: ' .. name) end
  end
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
  local o = ... or {}
  setmetatable(o, C)
  o:init()
  return o
end

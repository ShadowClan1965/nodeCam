-- nodeCam: a camera mode that welds the view to vehicle nodes you pick by hand.
-- Starts steady on the vehicle axes; four or more picked nodes switches it to a
-- rigid fit against those nodes.

if log then log('I', 'nodeCam', 'nodeCam 3.0 camera mode loaded') end

local C = {}
C.__index = C

local abs, sqrt, min, max, floor = math.abs, math.sqrt, math.min, math.max, math.floor
local cos, sin = math.cos, math.sin
local huge = math.huge

--[[NODECAM_SOLVER_BEGIN]]
-- Pure maths. No game globals, no vec3, no vehicle object: this block runs
-- standalone under plain lua, which is where it should be tested.

local S = {}

-- Orthogonal (rotation) factor of a 3x3 matrix, by Higham's iteration:
-- M <- 0.5 * (M + inverse(transpose(M))) converges quadratically to the
-- rotation nearest to M.
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

-- Weighted rigid fit (Kabsch). q is the rest shape, p the live positions.
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

-- Residual against the fitted frame. The one safety net left: stops the view
-- flying off when a picked node is torn away in a crash.
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

-- Optional virtual-beam softness.
function S.springStep(cx, cy, cz, vx, vy, vz,
                      nSel, live, px, py, pz, restLen, w,
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

-- Settings live in the GE extension so they survive vehicle switches. This is
-- only a last-resort copy for when that module is missing.
local FALLBACK = {
  enabled = true, quiet = false, fov = 65, moveSpeed = 1.1, fastMultiplier = 3.0,
  boundsMargin = 3.0, boundsEnabled = true, lookSensitivity = 0.25,
  keyLookSpeed = 1.2, invertYaw = false, invertPitch = false,
  invertPadPitch = false, attachCount = 10, slotCount = 3, softness = 0.0,
  stiffness = 900.0, damping = 26.0, maxSag = 0.25, outlierResidual = 0.09,
  outlierMedianScale = 3.0, maxDropFraction = 0.4, picker = false,
  pickRadius = 2.5, pickRadiusNear = 0.08, pickSpread = 0.045,
  maxDrawnNodes = 220,
}

local function cfg()
  if nodeCamCore and nodeCamCore.settings then return nodeCamCore.settings end
  return FALLBACK
end

local function consume(key)
  if nodeCamCore and nodeCamCore.consume then return nodeCamCore.consume(key) end
  return nil
end

local function logi(m)
  if nodeCamCore and nodeCamCore.logi then nodeCamCore.logi(m)
  elseif log then log('I', 'nodeCam', m) end
end

local function msg(text, ttl)
  if nodeCamCore and nodeCamCore.msg then nodeCamCore.msg(text, ttl) else logi(text) end
end

local function normalize3(x, y, z)
  local l = sqrt(x * x + y * y + z * z)
  if l < 1e-6 then return nil end
  return x / l, y / l, z / l
end

-- ---------------------------------------------------------------------------
-- camera mode
-- ---------------------------------------------------------------------------

function C:init()
  self.baseMode = true      -- we set pos, rot and fov ourselves
  self.register = true      -- show up in the C-key camera cycle
  self.hidden = false

  self.fov = self.fov or 65

  -- vehicle-local rest shape, filled in on attach
  self.qx, self.qy, self.qz = {}, {}, {}
  self.nNodes = 0
  self.aabb = nil
  self.shapeVid = nil
  self.shapeReady = false

  -- the attached node set, all hand-picked
  self.sel, self.selLive, self.selW, self.selRest = {}, {}, {}, {}
  self.nSel = 0
  self.px, self.py, self.pz = {}, {}, {}

  -- scratch buffers, reused so the hot path allocates nothing
  self.resid, self.residSorted = {}, {}
  self.rawX, self.rawY, self.rawZ = {}, {}, {}

  -- picker working set, all in live world-relative coordinates
  self.pickList = {}
  self.pickPX, self.pickPY, self.pickPZ, self.pickD = {}, {}, {}, {}
  self.inSet = {}
  self.nPick, self.hoverNode = 0, nil

  -- last frame's camera basis and fitted rotation
  self.camF = { x = 0, y = 1, z = 0 }
  self.camU = { x = 0, y = 0, z = 1 }
  self.R = nil

  -- anchor, in vehicle-local coordinates (+X left, +Y back, +Z up, fwd = -Y)
  self.anchor = { x = 0, y = 0, z = 0 }
  self.anchorValid = false
  self.anchorClamped = false

  self.yaw, self.pitch = 0, 0

  self.softPos = nil
  self.softVel = { x = 0, y = 0, z = 0 }
end

-- Vehicle reset: rebuild the rest shape against the repaired body, but keep
-- the picked nodes, the anchor and the view. A reset invalidates none of them.
function C:reset()
  self.debugBroken = false
  self.softPos = nil
  self.softVel.x, self.softVel.y, self.softVel.z = 0, 0, 0
  self.shapeReady = false
  self.shapeRetry = 0
end

-- Drop indices the new node map cannot support, reweight against the new shape.
function C:revalidateSet()
  if self.nSel <= 0 then return end
  local n = 0
  for s = 1, self.nSel do
    local i = self.sel[s]
    if i and i <= self.nNodes then n = n + 1; self.sel[n] = i end
  end
  if n ~= self.nSel then
    logi(string.format('node map shrank, kept %d of %d picked nodes', n, self.nSel))
  end
  self.nSel = n
  if n >= 4 then self:refreshSet() else self.softPos = nil end
end

function C:reloaded()
  self.shapeReady = false
  self.shapeVid = nil
  self.nSel = 0
  self.softPos = nil
end

function C:setFOV(fov) if fov then self.fov = fov end end
function C:setOffset(offset) end

-- ---------------------------------------------------------------------------
-- vehicle frame and rest shape
-- ---------------------------------------------------------------------------

-- Work out the vehicle's local axes in world space: +X left, +Y back, +Z up.
function C:resolveFrame(veh)
  local fx, fy, fz, ux, uy, uz, src

  -- 1. the reference node triad, if this build provides it
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

  if not src then return nil end

  -- make up perpendicular to forward
  local d = ux * fx + uy * fy + uz * fz
  ux, uy, uz = normalize3(ux - d * fx, uy - d * fy, uz - d * fz)
  if not ux then return nil end

  -- vehicle local: +Y is back (reverse of forward), +Z is up,
  -- +X = Y cross Z, which comes out as left
  local Yx, Yy, Yz = -fx, -fy, -fz
  local Zx, Zy, Zz = ux, uy, uz
  local Xx = Yy * Zz - Yz * Zy
  local Xy = Yz * Zx - Yx * Zz
  local Xz = Yx * Zy - Yy * Zx

  return Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz, src
end

function C:frameAxes(veh)
  local a = { self:resolveFrame(veh) }
  if a[1] then self.lastAxes = a; return a end
  return self.lastAxes
end

-- Identifies which vehicle the current rest shape belongs to. Replacing a
-- vehicle frequently reuses the same vehicle ID, so the ID alone is not enough:
-- without this the old node indices survive into the new jbeam and select
-- essentially random nodes.
function C:shapeSignature(veh)
  local count = 0
  pcall(function() count = veh:getNodeCount() or 0 end)
  local name = ''
  pcall(function() name = tostring(veh:getJBeamFilename() or '') end)
  if name == '' then pcall(function() name = tostring(veh.jbeam or '') end) end
  return tostring(count) .. '|' .. name
end

-- Sweep every node once and store the rest shape in vehicle-local coordinates.
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

  local Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz, src = self:resolveFrame(veh)
  if not Xx then
    self.shapeFail = 'could not establish the vehicle orientation'
    return false
  end

  if self.frameSource ~= src then
    self.frameSource = src
    logi('vehicle frame from: ' .. tostring(src))
  end

  local qx, qy, qz = self.qx, self.qy, self.qz
  local minx, miny, minz = huge, huge, huge
  local maxx, maxy, maxz = -huge, -huge, -huge

  for i = 1, n do
    local px, py, pz = rawX[i], rawY[i], rawZ[i]
    local lx = px * Xx + py * Xy + pz * Xz
    local ly = px * Yx + py * Yy + pz * Yz
    local lz = px * Zx + py * Zy + pz * Zz
    qx[i], qy[i], qz[i] = lx, ly, lz
    if lx < minx then minx = lx end
    if ly < miny then miny = ly end
    if lz < minz then minz = lz end
    if lx > maxx then maxx = lx end
    if ly > maxy then maxy = ly end
    if lz > maxz then maxz = lz end
  end

  self.lastAxes = { Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz }
  self.shapeFail = nil
  self.debugBroken = false
  self.nNodes = n
  self.aabb = { minx = minx, miny = miny, minz = minz,
                maxx = maxx, maxy = maxy, maxz = maxz }
  self.shapeReady = true
  return true
end

-- Roughly a dash-cam position, derived from the node cloud so it scales from a
-- hatchback to a semi.
function C:defaultAnchor()
  local a = self.aabb
  if not a then return 0, -0.2, 0.8 end
  return 0.0, a.miny + (a.maxy - a.miny) * 0.34,
              a.minz + (a.maxz - a.minz) * 0.62
end

-- ---------------------------------------------------------------------------
-- the attached set
-- ---------------------------------------------------------------------------

-- Weights and spring rest lengths, recomputed when the set changes.
function C:refreshSet()
  local ax, ay, az = self.anchor.x, self.anchor.y, self.anchor.z
  local qx, qy, qz = self.qx, self.qy, self.qz
  for s = 1, self.nSel do
    local i = self.sel[s]
    local dx, dy, dz = qx[i] - ax, qy[i] - ay, qz[i] - az
    local d2 = dx * dx + dy * dy + dz * dz
    self.selRest[s] = sqrt(d2)
    -- inverse square falloff: nodes right under the camera dominate the fit
    self.selW[s] = 1.0 / (d2 + 0.02)
    self.selLive[s] = true
  end
  self.softPos = nil
end

function C:toggleNode(i)
  if not i or i < 1 or i > self.nNodes then return end

  local at = nil
  for s = 1, self.nSel do if self.sel[s] == i then at = s; break end end

  if at then
    for s = at, self.nSel - 1 do self.sel[s] = self.sel[s + 1] end
    self.nSel = self.nSel - 1
  else
    self.nSel = self.nSel + 1
    self.sel[self.nSel] = i
  end

  self:refreshSet()
  msg(string.format('nodeCam: %s node, %d attached',
    at and 'removed' or 'added', self.nSel))
end

function C:clearNodes()
  self.nSel = 0
  self.softPos = nil
  msg('nodeCam: nodes cleared, steady view')
end

-- ---------------------------------------------------------------------------
-- camera slots
-- ---------------------------------------------------------------------------

function C:saveSlot(vid)
  if not (nodeCamCore and nodeCamCore.saveSlot) then return end
  local nodes = {}
  for i = 1, self.nSel do nodes[i] = self.sel[i] end
  nodeCamCore.saveSlot(vid, nodeCamCore.slotIndex(vid), {
    ax = self.anchor.x, ay = self.anchor.y, az = self.anchor.z,
    nodes = nodes, yaw = self.yaw, pitch = self.pitch,
  })
end

function C:loadSlot(vid, i)
  local d = nodeCamCore and nodeCamCore.getSlot and nodeCamCore.getSlot(vid, i)
  if not d then return false end
  self.anchor.x, self.anchor.y, self.anchor.z = d.ax, d.ay, d.az
  self.anchorValid = true
  self.yaw, self.pitch = d.yaw or 0, d.pitch or 0
  local n = 0
  for _, v in ipairs(d.nodes or {}) do
    if v <= self.nNodes then n = n + 1; self.sel[n] = v end
  end
  self.nSel = n
  if n >= 4 then self:refreshSet() else self.softPos = nil end
  return true
end

-- Park the current camera under the outgoing vehicle so tabbing back returns
-- you to the nodes you picked on it.
function C:stashState()
  if not (nodeCamCore and nodeCamCore.saveState) then return end
  if not (self.shapeVid and self.shapeSig and self.shapeReady) then return end
  local nodes = {}
  for i = 1, self.nSel do nodes[i] = self.sel[i] end
  nodeCamCore.saveState(self.shapeVid, {
    sig = self.shapeSig, nodes = nodes,
    ax = self.anchor.x, ay = self.anchor.y, az = self.anchor.z,
    yaw = self.yaw, pitch = self.pitch,
  })
end

-- Hand it back, but only if the body is still the one those nodes were picked on.
function C:restoreState(vid, sig)
  local st = nodeCamCore and nodeCamCore.getState and nodeCamCore.getState(vid)
  if not st or st.sig ~= sig then return false end

  self.anchor.x, self.anchor.y, self.anchor.z = st.ax, st.ay, st.az
  self.anchorValid = true
  self.yaw, self.pitch = st.yaw or 0, st.pitch or 0

  local n = 0
  for _, v in ipairs(st.nodes or {}) do
    if v <= self.nNodes then n = n + 1; self.sel[n] = v end
  end
  self.nSel = n
  if n >= 4 then self:refreshSet() else self.softPos = nil end
  return true
end

function C:cycleSlot(vid)
  local total = max(1, min(6, floor(cfg().slotCount or 3)))
  local cur = nodeCamCore.slotIndex(vid)
  self:saveSlot(vid)
  local nxt = (cur % total) + 1
  nodeCamCore.setSlotIndex(vid, nxt)
  if not self:loadSlot(vid, nxt) then self:saveSlot(vid) end
  msg(string.format('nodeCam: cam %d of %d, %s',
    nxt, total, self.nSel >= 4 and (self.nSel .. ' nodes') or 'steady'), 2)
end

-- ---------------------------------------------------------------------------
-- input
-- ---------------------------------------------------------------------------

-- Confirmed against 0.39: movement arrives on MoveManager as
-- forward/backward/left/right/up/down.
local function readMove(src)
  if src == nil then return 0, 0, 0 end
  local f = (tonumber(src.forward) or 0) - (tonumber(src.backward) or 0)
  local r = (tonumber(src.right) or 0) - (tonumber(src.left) or 0)
  local u = (tonumber(src.up) or 0) - (tonumber(src.down) or 0)
  return f, r, u
end

function C:handleMovement(dt)
  local c = cfg()

  -- console nudge, proves anchor movement works without a keybind
  local n = consume('nudge')
  if n then
    self.anchor.x = self.anchor.x + (n.x or 0)
    self.anchor.y = self.anchor.y + (n.y or 0)
    self.anchor.z = self.anchor.z + (n.z or 0)
    self:clampAnchor()
    return true
  end

  local fwdAmt, rightAmt, upAmt = 0, 0, 0
  if MoveManager ~= nil then
    local ok, f, r, u = pcall(readMove, MoveManager)
    if ok and (f ~= 0 or r ~= 0 or u ~= 0) then
      -- clamped, so moveSpeed means the same thing whatever the source carries
      fwdAmt = f > 1 and 1 or (f < -1 and -1 or f)
      rightAmt = r > 1 and 1 or (r < -1 and -1 or r)
      upAmt = u > 1 and 1 or (u < -1 and -1 or u)
      if nodeCamCore then nodeCamCore.moveSource = 'MoveManager' end
    end
  end

  if fwdAmt == 0 and rightAmt == 0 and upAmt == 0 then return false end

  local fast = (MoveManager and tonumber(MoveManager.fast) or 0) ~= 0
  local speed = (c.moveSpeed or 1.1) * (fast and (c.fastMultiplier or 3.0) or 1.0) * dt
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
    dx, dy, dz = -rightAmt, -fwdAmt, upAmt
  end

  self.anchor.x = self.anchor.x + dx
  self.anchor.y = self.anchor.y + dy
  self.anchor.z = self.anchor.z + dz
  self:clampAnchor()
  return true
end

-- Leash on the anchor. Was a hardcoded 0.9 m, which made moveSpeed look broken
-- once you hit the wall.
function C:clampAnchor()
  self.anchorClamped = false
  local c = cfg()
  if c.boundsEnabled == false then return end
  local a = self.aabb
  if not a then return end

  local m = c.boundsMargin or 3.0
  local x = max(a.minx - m, min(a.maxx + m, self.anchor.x))
  local y = max(a.miny - m, min(a.maxy + m, self.anchor.y))
  local z = max(a.minz - m, min(a.maxz + m, self.anchor.z))
  if x ~= self.anchor.x or y ~= self.anchor.y or z ~= self.anchor.z then
    self.anchorClamped = true
  end
  self.anchor.x, self.anchor.y, self.anchor.z = x, y, z
end

-- Two input routes with different sign conventions: the mouse arrives on the
-- delta fields reversed, the pad and keyboard on the pair fields the right way
-- round. Each gets its own sign below.
local LOOK_DELTAS = {
  { 'yawRelative',    'pitchRelative'    },
  { 'yawRelativeRaw', 'pitchRelativeRaw' },
  { 'camx',           'camy'             },
}

local LOOK_PAIRS = {
  { 'yawLeftSpeed',  'yawRightSpeed',  'yaw'   },
  { 'yawLeft',       'yawRight',       'yaw'   },
  { 'pitchUpSpeed',  'pitchDownSpeed', 'pitch' },
  { 'pitchUp',       'pitchDown',      'pitch' },
}

-- The source may be userdata, and reading an absent field is not guaranteed
-- safe, so every access is guarded.
local function num(src, k)
  local ok, v = pcall(function() return src[k] end)
  if ok then return tonumber(v) end
  return nil
end

-- Returns delta yaw/pitch and pair yaw/pitch separately, so the caller can sign
-- them independently.
local function readLook(src, sens, keyRate)
  if src == nil then return 0, 0, 0, 0, nil end
  local dy, dp, py, pp, used = 0, 0, 0, 0, nil

  for _, e in ipairs(LOOK_DELTAS) do
    local y, p = num(src, e[1]), num(src, e[2])
    if (y and y ~= 0) or (p and p ~= 0) then
      dy = dy + (y or 0) * sens
      dp = dp + (p or 0) * sens
      used = e[1]
      -- a delta is consumed, or it would be applied again next frame
      if y then pcall(function() src[e[1]] = 0 end) end
      if p then pcall(function() src[e[2]] = 0 end) end
      break
    end
  end

  for _, e in ipairs(LOOK_PAIRS) do
    local a, b = num(src, e[1]), num(src, e[2])
    if (a and a ~= 0) or (b and b ~= 0) then
      local v = ((a or 0) - (b or 0)) * keyRate
      if e[3] == 'yaw' then py = py + v else pp = pp + v end
      used = used or e[1]
    end
  end

  return dy, dp, py, pp, used
end

function C:handleLook(dt, data)
  local c = cfg()

  if consume('resetLook') then
    self.yaw, self.pitch = 0, 0
    msg('nodeCam: view recentred')
  end

  local sens = c.lookSensitivity or 0.25
  local keyRate = (c.keyLookSpeed or 1.2) * dt

  local dYaw, dPitch, pYaw, pPitch = 0, 0, 0, 0

  local l = consume('look')
  if l then pYaw, pPitch = pYaw + (l.yaw or 0), pPitch + (l.pitch or 0) end

  local sources = {}
  if data and type(data.move) == 'table' then sources[#sources + 1] = { data.move, 'data.move' } end
  if MoveManager ~= nil then sources[#sources + 1] = { MoveManager, 'MoveManager' } end

  for _, entry in ipairs(sources) do
    local ok, a, b, cc, d, used = pcall(readLook, entry[1], sens, keyRate)
    if ok then
      dYaw, dPitch = dYaw + a, dPitch + b
      pYaw, pPitch = pYaw + cc, pPitch + d
      if used and nodeCamCore then
        nodeCamCore.lookSource = entry[2] .. '.' .. used
      end
    else
      -- Silently swallowing this is how mouse look died once already: a refactor
      -- deleted the reader, the pcall caught the nil call, and look returned
      -- zero with nothing in the log to say why.
      local m = tostring(a)
      if self.lastLookErr ~= m then
        self.lastLookErr = m
        if nodeCamCore then nodeCamCore.loge('look input read failed: ' .. m) end
      end
    end
  end

  local yawSign = c.invertYaw and 1 or -1
  local pitchSign = c.invertPitch and -1 or 1
  local padYawSign = c.invertPadYaw and -1 or 1
  local padPitchSign = c.invertPadPitch and -1 or 1

  self.yaw = self.yaw + (dYaw * yawSign) + (pYaw * padYawSign)
  self.pitch = self.pitch + (dPitch * pitchSign) + (pPitch * padPitchSign)

  -- diag reports which path carried input, so device quirks stay diagnosable
  if nodeCamCore then
    local seen = nodeCamCore.lookSeen or {}
    nodeCamCore.lookSeen = seen
    if dYaw ~= 0 or dPitch ~= 0 then
      seen.delta = string.format('yaw %+.4f pitch %+.4f', dYaw, dPitch)
    end
    if pYaw ~= 0 or pPitch ~= 0 then
      seen.pair = string.format('yaw %+.4f pitch %+.4f', pYaw, pPitch)
    end
  end

  local lim = 1.45
  if self.pitch > lim then self.pitch = lim end
  if self.pitch < -lim then self.pitch = -lim end
end

-- ---------------------------------------------------------------------------
-- views
-- ---------------------------------------------------------------------------

-- Last resort when we cannot even resolve the vehicle axes.
function C:fallback(data, reason)
  reason = reason or 'unknown'
  if self.fallbackReason ~= reason then
    self.fallbackReason = reason
    msg('nodeCam: ' .. reason, 4)
  end

  local veh = data.veh
  local Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz
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

-- Camera straight off the vehicle axes, ignoring nodes. The default state.
function C:steady(data, veh, why)
  local a = self:frameAxes(veh)
  if not a or not a[1] then return self:fallback(data, why or 'no vehicle axes') end

  local Xx, Xy, Xz, Yx, Yy, Yz, Zx, Zy, Zz =
    a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9]
  local ax, ay, az = self.anchor.x, self.anchor.y, self.anchor.z
  local ox = Xx * ax + Yx * ay + Zx * az
  local oy = Xy * ax + Yy * ay + Zy * az
  local oz = Xz * ax + Yz * ay + Zz * az

  local fx, fy, fz, ux, uy, uz =
    S.applyLook(-Yx, -Yy, -Yz, Zx, Zy, Zz, self.yaw, self.pitch)

  local okq, q = pcall(quatFromDir, vec3(fx, fy, fz), vec3(ux, uy, uz))
  data.res.pos = data.pos + vec3(ox, oy, oz)
  if okq and q then data.res.rot = q end
  data.res.fov = self.fov

  self.camF.x, self.camF.y, self.camF.z = fx, fy, fz
  self.camU.x, self.camU.y, self.camU.z = ux, uy, uz
  self.R = self.R or {}
  self.R[1], self.R[2], self.R[3] = Xx, Yx, Zx
  self.R[4], self.R[5], self.R[6] = Xy, Yy, Zy
  self.R[7], self.R[8], self.R[9] = Xz, Yz, Zz

  self.lastCam = self.lastCam or {}
  self.lastCam.x, self.lastCam.y, self.lastCam.z = ox, oy, oz
  self.lastCam.fx, self.lastCam.fy, self.lastCam.fz = fx, fy, fz

  if cfg().picker then self:drawPicker(data, veh, ox, oy, oz, fx, fy, fz) end
  self.fallbackReason = nil
  return true
end

-- ---------------------------------------------------------------------------
-- update
-- ---------------------------------------------------------------------------

function C:update(data)
  local dt = data.dt or 0.016
  local veh = data.veh
  if not veh then return false end

  -- Keep trying to pull up the state module: a Lua reload can drop it, and a
  -- one-shot check here left the keybinds dead until the next level load.
  if not nodeCamCore then
    self.coreRetry = (self.coreRetry or 0) - dt
    if self.coreRetry <= 0 then
      self.coreRetry = 1.0
      pcall(function() extensions.load('nodeCamCore') end)
    end
  end

  local c = cfg()
  self.fov = c.fov or self.fov
  self.vid = data.vid

  -- A reset makes the rest shape stale but keeps your nodes: same jbeam, same
  -- indices. A different vehicle invalidates them. Compare a signature rather
  -- than the vehicle ID, which gets reused when a vehicle is replaced.
  local sig = self:shapeSignature(veh)
  if consume('invalidate') then self.shapeSig = nil end

  if self.shapeVid ~= data.vid or self.shapeSig ~= sig then
    self:stashState()
    self.shapeVid, self.shapeSig = data.vid, sig
    self.shapeReady = false
    self.anchorValid = false
    self.debugBroken = false
    self.nSel = 0
    self.softPos = nil
    self.hoverNode, self.nPick = nil, 0
    self.pendingRestore = true
  end

  if not self.shapeReady then
    -- retry quickly: mid-reset the node positions are briefly unreadable
    self.shapeRetry = (self.shapeRetry or 0) - dt
    if self.shapeRetry <= 0 then
      self.shapeRetry = 0.25
      if self:buildShapeGE(veh) then
        self:revalidateSet()
        if self.pendingRestore then
          self.pendingRestore = false
          if self:restoreState(data.vid, sig) then
            msg(string.format('nodeCam: %s',
              self.nSel >= 4 and (self.nSel .. ' nodes restored') or 'steady'))
          end
        end
      end
    end
    if not self.shapeReady then
      return self:fallback(data, self.shapeFail or 'could not map the vehicle nodes')
    end
  end

  if not self.anchorValid then
    local ax, ay, az = self:defaultAnchor()
    if nodeCamCore and nodeCamCore.getAnchor then
      local saved = nodeCamCore.getAnchor(data.vid)
      if saved then ax, ay, az = saved.x, saved.y, saved.z end
    end
    self.anchor.x, self.anchor.y, self.anchor.z = ax, ay, az
    self.anchorValid = true
  end

  self:handleLook(dt, data)
  self:handleMovement(dt)

  -- Pending actions are consumed here, before any early return. In the old
  -- build the bypass check returned above this block, so every one of these
  -- keybinds silently did nothing in steady view.
  if consume('clearNodes') then self:clearNodes() end
  if consume('cycleSlot') then self:cycleSlot(data.vid) end
  if consume('toggleNode') then
    if self.hoverNode then
      self:toggleNode(self.hoverNode)
    else
      msg('nodeCam: nothing under the crosshair, turn the picker on and aim at a node')
    end
  end

  self:publish(data)

  -- Node following off, or nothing picked yet: steady view on the vehicle axes.
  -- We stay in nodeCam either way, so the picker and every keybind keep working.
  if c.enabled == false or self.nSel < 4 then return self:steady(data, veh) end

  -- read the live node positions, as offsets from the vehicle origin
  local sel, px, py, pz, live = self.sel, self.px, self.py, self.pz, self.selLive
  local okRead = pcall(function()
    for s = 1, self.nSel do
      local p = vec3(veh:getNodePosition(sel[s] - 1))
      px[s], py[s], pz[s] = p.x, p.y, p.z
      live[s] = true
    end
  end)
  if not okRead then return self:steady(data, veh, 'could not read node positions') end

  -- fit, drop whatever is obviously wrong, fit again on what is left
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

  if not ok then
    -- Almost always a flat or collinear set: the fit has no third axis to work
    -- with. Say so, once, rather than silently sitting in steady view.
    self.fitFailFor = (self.fitFailFor or 0) + dt
    if self.fitFailFor > 0.5 and not self.fitWarned then
      self.fitWarned = true
      msg('nodeCam: those nodes are too flat to fit, pick some spread in depth', 4)
    end
    return self:steady(data, veh)
  end
  self.fitFailFor, self.fitWarned = 0, false

  -- where the anchor lands once the fitted frame is applied
  local lax, lay, laz = self.anchor.x - qcx, self.anchor.y - qcy, self.anchor.z - qcz
  local tx = pcx + (r11 * lax + r12 * lay + r13 * laz)
  local ty = pcy + (r21 * lax + r22 * lay + r23 * laz)
  local tz = pcz + (r31 * lax + r32 * lay + r33 * laz)

  local finalX, finalY, finalZ = self:applySoftness(c, tx, ty, tz, dt, live, px, py, pz)

  -- orientation straight out of the fitted frame
  local fx, fy, fz = -r12, -r22, -r32
  local ux, uy, uz = r13, r23, r33

  -- Look offsets applied as two explicit axis rotations, never as one euler
  -- triple, so no combination of yaw and pitch can roll the horizon.
  fx, fy, fz, ux, uy, uz = S.applyLook(fx, fy, fz, ux, uy, uz, self.yaw, self.pitch)

  local fwd, up = vec3(fx, fy, fz), vec3(ux, uy, uz)
  local flen = fwd:length()
  if flen < 1e-5 then return self:steady(data, veh, 'fitted frame has no forward axis') end
  fwd = fwd / flen
  local ulen = up:length()
  up = (ulen < 1e-5) and vec3(0, 0, 1) or (up / ulen)

  local okRot, rot = pcall(quatFromDir, fwd, up)
  if not okRot or not rot then
    okRot, rot = pcall(quatFromDir, fwd)
    if not okRot or not rot then return self:fallback(data, 'quatFromDir unavailable') end
  end

  if self.fallbackReason then
    msg('nodeCam: attached')
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

  self.lastCam = self.lastCam or {}
  self.lastCam.x, self.lastCam.y, self.lastCam.z = finalX, finalY, finalZ

  if nodeCamCore and nodeCamCore.setAnchor then
    nodeCamCore.setAnchor(data.vid, self.anchor.x, self.anchor.y, self.anchor.z)
  end

  if c.picker then
    self:drawPicker(data, veh, finalX, finalY, finalZ, fwd.x, fwd.y, fwd.z)
  else
    self.hoverNode, self.nPick = nil, 0
  end

  return true
end

-- Optional virtual-beam softness, leashed to the rigid answer.
function C:applySoftness(c, tx, ty, tz, dt, live, px, py, pz)
  local softness = c.softness or 0
  if softness <= 0.001 then self.softPos = nil; return tx, ty, tz end

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
    self.nSel, live, px, py, pz, self.selRest, self.selW,
    c.stiffness or 900.0, c.damping or 26.0, min(dt, 0.05), 3)

  if not ncx then self.softPos = nil; return tx, ty, tz end

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

  return tx + (ncx - tx) * softness,
         ty + (ncy - ty) * softness,
         tz + (ncz - tz) * softness
end

function C:publish(data)
  if not nodeCamCore then return end
  nodeCamCore.live = true
  local li = nodeCamCore.liveInfo or {}
  nodeCamCore.liveInfo = li
  li.vid, li.nSel, li.nNodes = data.vid, self.nSel, self.nNodes
  li.yaw, li.pitch = self.yaw, self.pitch
  li.fallback, li.shapeFail = self.fallbackReason, self.shapeFail
  li.shapeReady, li.frameSource = self.shapeReady, self.frameSource
  li.hover, li.nPick = self.hoverNode, self.nPick
  li.anchorX, li.anchorY, li.anchorZ = self.anchor.x, self.anchor.y, self.anchor.z
  li.clamped = self.anchorClamped
end

-- ---------------------------------------------------------------------------
-- picker
-- ---------------------------------------------------------------------------

-- Draw nearby nodes and report which is under the crosshair. Everything here
-- works from live positions: filtering on rest positions is what made open
-- doors select the wrong nodes. Reads every node per frame, hence picker-only.
function C:drawPicker(data, veh, camX, camY, camZ, fx, fy, fz)
  if not debugDrawer or self.debugBroken then return end
  local c = cfg()

  local r2 = (c.pickRadius or 2.5) ^ 2
  local maxN = c.maxDrawnNodes or 220
  local list, dist = self.pickList, self.pickD
  local px, py, pz = self.pickPX, self.pickPY, self.pickPZ

  -- camera position relative to the vehicle origin, matching getNodePosition
  local n = 0
  local ok = pcall(function()
    for i = 1, self.nNodes do
      local p = vec3(veh:getNodePosition(i - 1))
      local dx, dy, dz = p.x - camX, p.y - camY, p.z - camZ
      local d2 = dx * dx + dy * dy + dz * dz
      if d2 <= r2 then
        n = n + 1
        list[n] = i
        dist[i] = d2
        px[n], py[n], pz[n] = p.x, p.y, p.z
      end
    end
  end)
  if not ok then
    self.debugBroken = true
    if nodeCamCore then nodeCamCore.logw('node read failed, turning the picker off') end
    return
  end
  for k = n + 1, #list do list[k] = nil end

  -- sort before capping, so the cap keeps the nearest rather than the first
  if n > maxN then
    table.sort(list, function(a, b) return dist[a] < dist[b] end)
    -- positions were gathered in scan order, so re-read them for the kept set
    local ok2 = pcall(function()
      for k = 1, maxN do
        local p = vec3(veh:getNodePosition(list[k] - 1))
        px[k], py[k], pz[k] = p.x, p.y, p.z
      end
    end)
    if not ok2 then return end
    n = maxN
  end
  self.nPick = n
  if n == 0 then self.hoverNode = nil; return end

  -- which of these are currently attached, and are they pulling their weight
  local inSet = self.inSet
  for k in pairs(inSet) do inSet[k] = nil end
  for s = 1, self.nSel do
    inSet[self.sel[s]] = self.selLive[s] and 2 or 1
  end

  local hit = S.pickAlongRay(n, px, py, pz, camX, camY, camZ,
    fx, fy, fz, c.pickSpread or 0.045, c.pickRadiusNear or 0.08)
  self.hoverNode = hit and list[hit] or nil

  local okDraw = pcall(function()
    local base = data.pos
    for k = 1, n do
      local nodeW = base + vec3(px[k], py[k], pz[k])
      local state = inSet[list[k]]
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
    end
  end)

  if not okDraw then
    self.debugBroken = true
    if nodeCamCore then
      nodeCamCore.logw('debug drawing failed on this build, turning it off')
    end
  end
end

-- DO NOT CHANGE CLASS IMPLEMENTATION BELOW

return function(...)
  local o = ... or {}
  setmetatable(o, C)
  o:init()
  return o
end

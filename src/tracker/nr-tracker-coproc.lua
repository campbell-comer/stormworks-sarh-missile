-- nr-tracker-coproc.lua -- multi-target track manager and mount control
--
-- Upstream of the EKF: maintains a coarse track table over all radar
-- contacts, lets the operator hook and lock one, seeds the EKF with the
-- locked track, and steers the sensor mount (sweep, manual, or track-follow).
--
-- NUMBER INPUTS: contact x/y/z triples per radar (1..24), 25/27/26 mount
-- position (raw physics-sensor order), 28-30 mount pitch / roll / heading,
-- 31/32 manual yaw / pitch stick.
-- BOOL INPUTS: 1..8 contact valid, 9 sweep mode, 10 lock toggle, 11 hold,
-- 12/13 previous/next hook step, 14/15 zoom out/in.
-- NUMBER OUTPUTS: 1-3 locked track position (EKF seed), 10/11 mount yaw
-- velocity / pitch position commands, 15-29 display slots (5 x/y/z triples,
-- locked or hooked first), 30 live track count, 31 zoom, 32 status word
-- (fault code + 8 * hooked track number).
-- BOOL OUTPUTS: 1 lock valid, 2 more than one live track.
-- PROPERTIES: Sweep Command, Manual Yaw/Pitch Rate, Rejection Range.

pgn = property.getNumber
getN = input.getNumber
getB = input.getBool
setN = output.setNumber
setB = output.setBool
m = math
sqrtf, sine, cosine, tanf, floorf = m.sqrt, m.sin, m.cos, m.tan, m.floor
atan2 = m.atan
minf, maxf = m.min, m.max
TAU = m.pi * 2

TRACK_GATE  = 60          -- m association gate, measurement to track
MERGE_M     = 40          -- m: contacts merged into one measurement
DROP_TICKS  = 180         -- track age-out
SHOW_MIN_N  = 3           -- detections before a track is shown/selectable
LOCK_CONE   = 0.80        -- min cos(angle to boresight) for a lock pick

FOV_CONE_K  = 2.0         -- widen the pick cone with the camera FOV

VEL_DT      = 12          -- ticks between velocity refreshes per track
TN_MAX      = 32          -- display numbers recycle in 1..TN_MAX

YAW_GAIN    = 4.0
PITCH_GAIN  = 0.05

ZOOM_RATE   = 1.05
ZOOM_MIN    = 0.29
ZOOM_MAX    = 28
REF_HALF    = m.pi / 180 * 35

SWEEP_CMD = pgn("Sweep Command")
MAN_YAW   = pgn("Manual Yaw Rate")
MAN_PITCH = pgn("Manual Pitch Rate")

REJECT_R = pgn("Rejection Range")
if REJECT_R <= 0 then REJECT_R = 1e9 end

T, nextId, lockId, hookId = 0, 0, 0, 0
lockEdge, lockHeld, lockDip = false, 0, 0
prevEdge, nextEdge = false, false
lockFault, lockFaultT = 0, -999
manPitch = 0
zoomX, zin = 1, 0
tracks, usedN = {}, {}

cx, cy, cz = {}, {}, {}
mx, my, mz, mc = {}, {}, {}, {}

function clamp(x, lo, hi) return x < lo and lo or (x > hi and hi or x) end
function dist3(ax, ay, az, bx, by, bz) return sqrtf((ax - bx) ^ 2 + (ay - by) ^ 2 + (az - bz) ^ 2) end
function putSlot(k, x, y, z) setN(12 + k * 3, x); setN(13 + k * 3, y); setN(14 + k * 3, z) end

function mountAxes(yaw, pitch, roll)
    mcy, msy, mcp, msp, mcr, msr = cosine(yaw), sine(yaw), cosine(pitch), sine(pitch), cosine(roll), sine(roll)
    fwdX, fwdY, fwdZ = msy * mcp, mcy * mcp, msp
    upX, upY, upZ = mcy * msr - msy * mcr * msp, -msy * msr - mcy * mcr * msp, mcr * mcp
end

function newTrack(x, y, z)
    nextId = nextId + 1
    -- assign the lowest free display number
    for k = 1, TN_MAX do usedN[k] = false end
    for id, tr in pairs(tracks) do usedN[tr.num] = true end
    tn = TN_MAX
    for k = 1, TN_MAX do if not usedN[k] then tn = k break end end
    tracks[nextId] = { x = x, y = y, z = z, vx = 0, vy = 0, vz = 0, px = x, py = y, pz = z, pt = T, t = T, n = 1, num = tn }
end

function refreshTrack(tr, x, y, z)
    tr.x, tr.y, tr.z = (tr.x + x) / 2, (tr.y + y) / 2, (tr.z + z) / 2
    dtv = T - tr.pt
    if dtv >= VEL_DT then
        tr.vx = 0.6 * tr.vx + 0.4 * (x - tr.px) / dtv
        tr.vy = 0.6 * tr.vy + 0.4 * (y - tr.py) / dtv
        tr.vz = 0.6 * tr.vz + 0.4 * (z - tr.pz) / dtv
        tr.px, tr.py, tr.pz, tr.pt = x, y, z, T
    end
    tr.t, tr.n = T, tr.n + 1
end

-- pick the shown track closest to the mount boresight, within cone
function pickBore(cone)
    bestId, bestDot = 0, cone
    for id, tr in pairs(tracks) do
        if tr.n >= SHOW_MIN_N then
            dx, dy, dz = tr.x - mountX, tr.y - mountY, tr.z - mountZ
            dl = maxf(sqrtf(dx * dx + dy * dy + dz * dz), 1)
            dot = (dx * fwdX + dy * fwdY + dz * fwdZ) / dl
            if dot > bestDot then bestDot, bestId = dot, id end
        end
    end
    return bestId
end

-- step the hook to the next/previous track by display number, wrapping
function stepHook(dir)
    cur = tracks[hookId] and tracks[hookId].num or (dir > 0 and 0 or TN_MAX + 1)
    stepId, stepN, wrapId, wrapN = 0, 0, 0, 0
    for id, tr in pairs(tracks) do
        if tr.n >= SHOW_MIN_N then
            if (tr.num - cur) * dir > 0 and (stepId == 0 or (tr.num - stepN) * dir < 0) then stepId, stepN = id, tr.num end
            if wrapId == 0 or (tr.num - wrapN) * dir < 0 then wrapId, wrapN = id, tr.num end
        end
    end
    return stepId ~= 0 and stepId or wrapId
end

function onTick()
    T = T + 1
    sweepMode, lockNow, holdNow = getB(9), getB(10), getB(11)

    mountX, mountY, mountZ = getN(25), getN(27), getN(26)
    mountPitch, mountHdg = getN(28), getN(30)
    mountAxes(-mountHdg * TAU, mountPitch * TAU, getN(29) * TAU)

    if getB(15) then zoomX = minf(zoomX * ZOOM_RATE, ZOOM_MAX)
    elseif getB(14) then zoomX = maxf(zoomX / ZOOM_RATE, ZOOM_MIN) end
    zin = clamp(1 - 2 * atan2(tanf(REF_HALF) / zoomX) * 180 / m.pi / (127 - 1.43), 0, 1)
    setN(31, zin)

    coneNow = maxf(LOCK_CONE, cosine(minf(((1 - zin) * 2.1893 + 0.025) / 2 * FOV_CONE_K, 1.5)))

    -- gather contacts, drop returns inside the rejection range
    cn = 0
    for b = 1, 8 do
        if getB(b) then
            cb = (b - 1) * 3
            kx, ky, kz = getN(cb + 1), getN(cb + 2), getN(cb + 3)
            if dist3(kx, ky, kz, mountX, mountY, mountZ) <= REJECT_R then
                cn = cn + 1
                cx[cn], cy[cn], cz[cn] = kx, ky, kz
            end
        end
    end

    -- merge contacts within MERGE_M into running-mean measurements
    mn = 0
    for i = 1, cn do
        mi = 0
        for j = 1, mn do
            if mi == 0 and dist3(cx[i], cy[i], cz[i], mx[j], my[j], mz[j]) < MERGE_M then mi = j end
        end

        if mi == 0 then
            mn, mi = mn + 1, mn + 1
            mx[mi], my[mi], mz[mi], mc[mi] = 0, 0, 0, 0
        end
        mc[mi] = mc[mi] + 1
        mx[mi] = mx[mi] + (cx[i] - mx[mi]) / mc[mi]
        my[mi] = my[mi] + (cy[i] - my[mi]) / mc[mi]
        mz[mi] = mz[mi] + (cz[i] - mz[mi]) / mc[mi]
    end

    -- associate measurements to velocity-predicted tracks
    for i = 1, mn do
        bestId, bestD = 0, TRACK_GATE
        for id, tr in pairs(tracks) do
            predDt = minf(T - tr.t, 60)
            d = dist3(mx[i], my[i], mz[i], tr.x + tr.vx * predDt, tr.y + tr.vy * predDt, tr.z + tr.vz * predDt)
            if d < bestD then bestD, bestId = d, id end
        end
        if bestId > 0 then refreshTrack(tracks[bestId], mx[i], my[i], mz[i])
        else newTrack(mx[i], my[i], mz[i]) end
    end

    liveCount = 0
    for id, tr in pairs(tracks) do
        if T - tr.t > DROP_TICKS and not (holdNow and id == lockId) then tracks[id] = nil
        elseif tr.n >= SHOW_MIN_N then liveCount = liveCount + 1 end
    end

    -- hook stepping (edge-triggered prev/next)
    prevNow, nextNow = getB(12), getB(13)
    stepDir = (nextNow and not nextEdge) and 1 or ((prevNow and not prevEdge) and -1 or 0)
    prevEdge, nextEdge = prevNow, nextNow
    if stepDir ~= 0 then

        hookNew = tracks[hookId] and stepHook(stepDir) or pickBore(-1)
        if hookNew > 0 then hookId = hookNew end
    end

    -- lock toggle: locks the hooked track, or the best boresight track
    if lockNow and not lockEdge then
        if lockId > 0 then
            lockId, hookId = 0, 0
        else
            hookId = (tracks[hookId] and tracks[hookId].n >= SHOW_MIN_N) and hookId or pickBore(coneNow)
            lockId = hookId
            -- fault codes: 3 = tracks exist but none in cone, 2 = no tracks
            if hookId == 0 then lockFault, lockFaultT = liveCount > 0 and 3 or 2, T end
        end
    end
    lockEdge = lockNow
    if not tracks[hookId] then hookId = 0 end
    if lockId > 0 and not tracks[lockId] then lockId = 0 end

    -- re-hooking while locked moves the lock (one-tick dip on bool 1 so the
    -- EKF re-seeds on the new track)
    if lockId > 0 and hookId > 0 and hookId ~= lockId then lockId, lockDip = hookId, 1 end

    if lockId > 0 then
        lt = tracks[lockId]
        setN(1, lt.x); setN(2, lt.y); setN(3, lt.z)
    else
        setN(1, 0); setN(2, 0); setN(3, 0)
    end
    setN(30, liveCount)
    setB(1, lockId > 0 and lockDip == 0)
    lockDip = 0
    setB(2, liveCount > 1)

    lockHeld = lockNow and lockHeld + 1 or 0
    if lockHeld > 300 then lockFault, lockFaultT = 1, T end
    hookNum = tracks[hookId] and tracks[hookId].num or 0
    if mountX * mountX + mountY * mountY + mountZ * mountZ < 1 then
        setN(32, 4 + 8 * hookNum)   -- fault 4: no mount position wired
    else
        setN(32, (T - lockFaultT < 180 and lockFault or 0) + 8 * hookNum)
    end

    -- mount steering: follow the lock, else sweep, else manual
    if lockId > 0 then
        dx, dy, dz = lt.x - mountX, lt.y - mountY, lt.z - mountZ

        yerr = atan2(-dx, dy) / TAU - mountHdg
        setN(10, clamp(YAW_GAIN * (yerr - floorf(yerr + 0.5)) * sqrtf(maxf(0, 1 - fwdZ * fwdZ)), -1, 1))

        dUp = dx * upX + dy * upY + dz * upZ
        manPitch = clamp(manPitch + PITCH_GAIN * atan2(dUp,
            sqrtf(maxf(dx * dx + dy * dy + dz * dz - dUp * dUp, 0))) * 4 / TAU, -1, 1)
        setN(11, manPitch)
    elseif sweepMode then

        setN(10, SWEEP_CMD)
        setN(11, 0)
        manPitch = 0
    else
        -- manual rates scale with zoom so fine aiming stays fine
        manScale = ((1 - zin) * 2.1893 + 0.025) / 2.2143
        manPitch = clamp(manPitch + getN(32) * MAN_PITCH * manScale / 60, -1, 1)
        setN(10, clamp(getN(31) * MAN_YAW * manScale, -1, 1))
        setN(11, manPitch)
    end

    -- display slots: slot 1 the lock, slot 2 the hook (when not locked),
    -- remaining slots filled with other shown tracks
    slot = 1
    hookBg = lockId == 0 and tracks[hookId] and tracks[hookId].n >= SHOW_MIN_N
    if hookBg then
        slot = 2
        putSlot(2, tracks[hookId].x, tracks[hookId].y, tracks[hookId].z)
    end
    for id, tk in pairs(tracks) do
        if slot < 5 and id ~= lockId and not (hookBg and id == hookId) and tk.n >= SHOW_MIN_N then
            slot = slot + 1
            putSlot(slot, tk.x, tk.y, tk.z)
        end
    end
    for k = slot + 1, 5 do putSlot(k, 0, 0, 0) end
end

-- prox-fuse.lua -- proximity fuse
--
-- Independent of the flight computer: reads the same datalink target and own
-- position, fires bool output 1 when the target is (or is about to be) inside
-- the detonation radius. Three tests, cheapest first:
--   1. current relative distance inside the radius,
--   2. the segment swept since last tick passed through the radius
--      (catches a fly-through between samples),
--   3. predicted sphere entry within COMP ticks, from a two-tick central
--      difference of the relative velocity (covers the fuse-to-warhead delay).
--
-- NUMBER INPUTS: 1-3 target position, 7-9 own position (raw physics-sensor
-- order), 30 LAUNCHED level (>= 0.5 armed).
-- PROPERTIES: Safety Ticks, Missile Lifetime, Fuse Tick Compensation,
-- Max Detonation Distance.

IN    = input.getNumber
ONB   = output.setBool
PROPN = property.getNumber
sqrt, max, huge = math.sqrt, math.max, math.huge

SAFETY_TICKS = max(0, PROPN("Safety Ticks"))
LIFE_T     = PROPN("Missile Lifetime")
LIFE_TICKS = LIFE_T > 0 and LIFE_T * 60 or huge
COMP   = max(0, PROPN("Fuse Tick Compensation")) + 0.5
RADIUS = max(0.1, PROPN("Max Detonation Distance"))
R2 = RADIUS * RADIUS

TGT_VW    = 8             -- ticks of target-position differencing for velocity
COAST_MAX = 40            -- ticks to coast the target through a datalink gap

PRED_VMAX = 20            -- m/tick: reject implausible closing speeds in the
                          -- entry prediction (a feed glitch, not physics)

launchTk = 0
fired    = false
firedTk  = 0
relN     = 0
relX, relY, relZ = 0, 0, 0
r1X, r1Y, r1Z = 0, 0, 0
r2X, r2Y, r2Z = 0, 0, 0
tgtN = 0
tgtHX, tgtHY, tgtHZ = {}, {}, {}
for i = 1, TGT_VW + 1 do tgtHX[i], tgtHY[i], tgtHZ[i] = 0, 0, 0 end
lastTX, lastTY, lastTZ = 0, 0, 0
tvX, tvY, tvZ = 0, 0, 0
coastN = -1

function retired() end

-- squared distance from the origin to the segment a-b
function segDist2(ax, ay, az, bx, by, bz)
    dx, dy, dz = bx - ax, by - ay, bz - az
    l2 = dx * dx + dy * dy + dz * dz
    if l2 < 1e-9 then return ax * ax + ay * ay + az * az end
    s = -(ax * dx + ay * dy + az * dz) / l2
    s = s < 0 and 0 or (s > 1 and 1 or s)
    px, py, pz = ax + dx * s, ay + dy * s, az + dz * s
    return px * px + py * py + pz * pz
end

-- ticks until the relative position enters the detonation sphere, or nil
function entryTicks(x, y, z, vx, vy, vz)
    c = x * x + y * y + z * z - R2
    if c <= 0 then return 0 end
    a = vx * vx + vy * vy + vz * vz
    if a < 1e-9 or a > PRED_VMAX * PRED_VMAX then return nil end
    b = 2 * (x * vx + y * vy + z * vz)
    d = b * b - 4 * a * c
    if d < 0 then return nil end
    t = (-b - sqrt(d)) / (2 * a)
    return t >= 0 and t or nil
end

function onTick()
    if fired then
        ONB(1, true)
        firedTk = firedTk + 1
        if firedTk > 60 then onTick = retired end
        return
    end

    launchTk = IN(30) >= .5 and launchTk + 1 or 0
    if launchTk == 0 then
        relN, tgtN, coastN = 0, 0, -1
        ONB(1, false)
        return
    end

    if launchTk > LIFE_TICKS then
        ONB(1, false)
        onTick = retired
        return
    end

    tx, ty, tz = IN(1), IN(2), IN(3)
    ownX, ownY, ownZ = IN(7), IN(9), IN(8)

    if tx == 0 and ty == 0 and tz == 0 then
        -- datalink gap: coast the target on its last derived velocity
        tgtN = 0
        if coastN >= 0 and coastN < COAST_MAX then
            coastN = coastN + 1
            tx = lastTX + tvX * coastN
            ty = lastTY + tvY * coastN
            tz = lastTZ + tvZ * coastN
        else
            relN = 0
            ONB(1, false)
            return
        end
    else
        for i = 1, TGT_VW do
            tgtHX[i], tgtHY[i], tgtHZ[i] = tgtHX[i + 1], tgtHY[i + 1], tgtHZ[i + 1]
        end
        tgtHX[TGT_VW + 1], tgtHY[TGT_VW + 1], tgtHZ[TGT_VW + 1] = tx, ty, tz
        tgtN = tgtN + 1
        if tgtN > TGT_VW then
            tvX = (tx - tgtHX[1]) / TGT_VW
            tvY = (ty - tgtHY[1]) / TGT_VW
            tvZ = (tz - tgtHZ[1]) / TGT_VW
        else
            tvX, tvY, tvZ = 0, 0, 0
        end
        lastTX, lastTY, lastTZ, coastN = tx, ty, tz, 0
    end

    r2X, r2Y, r2Z = r1X, r1Y, r1Z
    r1X, r1Y, r1Z = relX, relY, relZ
    relX, relY, relZ = tx - ownX, ty - ownY, tz - ownZ
    relN = relN + 1

    if launchTk <= SAFETY_TICKS then
        ONB(1, false)
        return
    end

    d2 = relX * relX + relY * relY + relZ * relZ
    fire = d2 <= R2
    if not fire and relN >= 2 then
        fire = segDist2(r1X, r1Y, r1Z, relX, relY, relZ) <= R2
    end
    if not fire and relN >= 3 then
        et = entryTicks(relX, relY, relZ,
                        (relX - r2X) * 0.5, (relY - r2Y) * 0.5, (relZ - r2Z) * 0.5)
        fire = et ~= nil and et <= COMP
    end

    fired = fire
    ONB(1, fired)
end

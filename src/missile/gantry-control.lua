-- gantry-control.lua -- launcher rack aiming and fire sequencing
--
-- Points the launch rack at the locked target within its yaw/pitch travel
-- limits and fires up to MISSILES rounds, one bool pulse per fire edge.
--
-- NUMBER INPUTS: 1-3 target position, 4/6/5 rack position (x/z/y raw), 7-9
-- rack pitch / roll / heading, 10-12 vehicle body pitch / roll / heading.
-- BOOL INPUTS: 1 target valid, 2 armed, 3 fire, 4 lock quality ok,
-- 5 free-fire override (fire without a qualified target).
-- OUTPUTS: numbers 1 rounds remaining, 2 yaw velocity command, 3 pitch
-- position command; bools 1..MISSILES fire pulses.
-- PROPERTIES: Min/Max Launcher Yaw (deg), Min/Max Launcher Pitch (deg).

GN = input.getNumber
GB = input.getBool
ON = output.setNumber
OB = output.setBool
PN = property.getNumber
TAU = math.pi * 2
MISSILES = 3
YAW_GAIN = 4
EPS = 1e-8
AZIMUTH_RATIO2 = 1e-6     -- min horizontal component before yaw is commanded

function clamp(a, b, c)
    return a < b and b or (a > c and c or a)
end

function minmax(d, e)
    return math.min(d, e), math.max(d, e)
end

function turnDistance(d, e)
    return math.abs((d - e + 0.5) % 1 - 0.5)
end

-- nearest reachable yaw: inside the travel window, or whichever limit is
-- closer around the circle
function closestYaw(a)
    if a >= YAW_MIN and a <= YAW_MAX then
        return a
    end
    return turnDistance(a, YAW_MIN) <= turnDistance(a, YAW_MAX) and YAW_MIN or YAW_MAX
end

function axes(f, g, h)
    y = -h * TAU
    p = f * TAU
    r = g * TAU
    cy, sy, cp, sp, cr, sr = math.cos(y), math.sin(y), math.cos(p), math.sin(p), math.cos(r), math.sin(r)
    return {
        f = {sy * cp, cy * cp, sp},
        r = {cy * cr + sy * sp * sr, -sy * cr + cy * sp * sr, -cp * sr},
        u = {cy * sr - sy * cr * sp, -sy * sr - cy * cr * sp, cr * cp}
    }
end

function dot(a, y, i, j)
    return a * j[1] + y * j[2] + i * j[3]
end

ya, yb = minmax(clamp(PN("Min Launcher Yaw (deg)") / 360, -0.5, 0.5),
                clamp(PN("Max Launcher Yaw (deg)") / 360, -0.5, 0.5))
pa, pb = minmax(clamp(PN("Min Launcher Pitch (deg)") / 90, -1, 1),
                clamp(PN("Max Launcher Pitch (deg)") / 90, -1, 1))
YAW_MIN, YAW_MAX = ya, yb
PITCH_MIN, PITCH_MAX = pa, pb
shotsFired = 0
firePrev = false
yawCmd, pitchCmd = 0, 0

function onTick()
    OB(4, false)
    for k = 1, MISSILES do
        OB(k, false)
    end
    fireNow = GB(3)
    fireEdge = fireNow and not firePrev
    firePrev = fireNow

    dx = GN(1) - GN(4)
    dy = GN(2) - GN(6)
    dz = GN(3) - GN(5)
    range2 = dx * dx + dy * dy + dz * dz
    validTarget = range2 > 1

    body = axes(GN(10), GN(11), GN(12))
    rack = axes(GN(7), GN(8), GN(9))

    -- target and current rack direction in body frame
    tr, tf, tu = dot(dx, dy, dz, body.r), dot(dx, dy, dz, body.f), dot(dx, dy, dz, body.u)
    cr = dot(rack.f[1], rack.f[2], rack.f[3], body.r)
    cf = dot(rack.f[1], rack.f[2], rack.f[3], body.f)
    cu = dot(rack.f[1], rack.f[2], rack.f[3], body.u)
    targetHoriz2 = tr * tr + tf * tf
    rackHoriz2 = cr * cr + cf * cf

    -- default: hold the current pitch inside limits, no yaw motion
    currentPitch = math.atan(cu, math.sqrt(math.max(rackHoriz2, 0))) * 4 / TAU
    pitchCmd = clamp(currentPitch, PITCH_MIN, PITCH_MAX)
    yawCmd = 0

    qualifiedTarget = GB(1) and GB(4) and validTarget
    aiming = GB(2) and qualifiedTarget and not GB(5)
    if aiming then
        pitchCmd = clamp(math.atan(tu, math.sqrt(math.max(targetHoriz2, 0))) * 4 / TAU,
                         PITCH_MIN, PITCH_MAX)
        if targetHoriz2 > range2 * AZIMUTH_RATIO2 and rackHoriz2 > EPS then
            yawSet = closestYaw(-math.atan(tr, tf) / TAU)
            yawPos = -math.atan(cr, cf) / TAU
            yawCmd = clamp((yawSet - yawPos) * YAW_GAIN, -1, 1)
            -- never drive into a hard stop
            if yawPos >= YAW_MAX and yawCmd > 0 or yawPos <= YAW_MIN and yawCmd < 0 then
                yawCmd = 0
            end
        end
    end

    fireAllowed = GB(2) and (GB(5) or qualifiedTarget)
    if fireEdge and fireAllowed and shotsFired < MISSILES then
        shotsFired = shotsFired + 1
        OB(shotsFired, true)
    end
    ON(1, MISSILES - shotsFired)
    ON(2, yawCmd)
    ON(3, pitchCmd)
end

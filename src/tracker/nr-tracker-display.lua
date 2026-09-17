-- nr-tracker-display.lua -- tracker HUD and camera overlay
--
-- Touchscreen UI over the gimbal camera: projects the tracked targets into
-- the camera view, draws the lock/hook markers with range and speed labels,
-- shows fault and status lines, and drives the camera pivot to keep the lock
-- centred. Buttons: START/STOP, SWEEP, LOCK, HOLD (touch or seat hotkeys).
--
-- NUMBER INPUTS: 1/3/2 camera position (x/z/y raw), 4-6 camera pitch / roll /
-- heading, 7-9 touch x / touch y / press, 10 radar bias diagnostic,
-- 11 contributing radars, 12-14 lock velocity, 15-29 target slots from the
-- coprocessor, 30 track count, 31 zoom, 32 status word.
-- BOOL INPUTS: 1 locked, 3/4 pass-throughs, 5 out-of-range, 6 hazard,
-- 7/8/10 seat hotkeys (power / sweep / hold), 9 seat lock button,
-- 11/12 pass-throughs.
-- OUTPUTS: numbers 1/2 pivot commands, 3 zoom; bools 1 system on, 2 sweep,
-- 3 lock request, 4 hold, 9 live, 13 locked.

INN = input.getNumber
INB = input.getBool
OUN = output.setNumber
OUB = output.setBool
pi2 = math.pi * 2

CAM_OFF_Y = 0.5           -- camera offset from the reported position, blocks
CAM_OFF_Z = -0.5

BOOT_TICKS = 30

DISP_LEAD = 1             -- ticks of velocity lead on the drawn lock marker

BIAS_WARN = 12            -- m of per-radar bias before the wiring warning

PIV_MAX = 0.125           -- pivot command range, turns

-- display hold bands: values move only when they change by more than this,
-- so labels do not flicker at the sensor quantisation
HOLD_PIV = 0.0002
HOLD_PX  = 2
HOLD_RNG = 2
HOLD_SPD = 1
VEL_ALPHA = 0.06

AIM_X = 1
AIM_Y = 1

UI_BG_R, UI_BG_G, UI_BG_B = 10, 12, 15
UI_FILL_R, UI_FILL_G, UI_FILL_B = 20, 25, 31
UI_DIM_R, UI_DIM_G, UI_DIM_B = 80, 92, 104
UI_TXT_R, UI_TXT_G, UI_TXT_B = 190, 202, 214
UI_ACC_R, UI_ACC_G, UI_ACC_B = 62, 205, 255
UI_GRN_R, UI_GRN_G, UI_GRN_B = 40, 220, 90
UI_RED_R, UI_RED_G, UI_RED_B = 235, 70, 50
UI_AMB_R, UI_AMB_G, UI_AMB_B = 255, 180, 40

FAULTS = { "LOCK STUCK?", "NO TRACKS", "OFF AXIS", "NO MNT POS" }

function vec(x, y, z) return { x = x, y = y, z = z } end
function dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
function clamp(x, lo, hi) return x < lo and lo or (x > hi and hi or x) end
function isTouching(px, py, rx, ry, rw, rh) return px > rx and py > ry and px < rx + rw and py < ry + rh end
function txtW(s) return #s * 5 - 1 end

function hold(prev, now, band)
    return (prev == nil or math.abs(now - prev) > band) and now or prev
end

function axesFromYPR(yaw, pitch, roll)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return {
        forward = vec(sy * cp, cy * cp, sp),
        right   = vec(cy * cr + sy * sp * sr, -sy * cr + cy * sp * sr, -cp * sr),
        up      = vec(cy * sr - sy * cr * sp, -sy * sr - cy * cr * sp, cr * cp)
    }
end

-- rotate the camera axes by the pivot the display itself commanded last tick,
-- so projection matches what the camera actually shows
function pivotAxes(ax, yawT, pitchT)
    cy, sy = math.cos(yawT * pi2), math.sin(yawT * pi2)
    cp, sp = math.cos(pitchT * pi2), math.sin(pitchT * pi2)
    f1 = vec(ax.forward.x * cy + ax.right.x * sy, ax.forward.y * cy + ax.right.y * sy, ax.forward.z * cy + ax.right.z * sy)
    r1 = vec(-ax.forward.x * sy + ax.right.x * cy, -ax.forward.y * sy + ax.right.y * cy, -ax.forward.z * sy + ax.right.z * cy)
    f2 = vec(f1.x * cp + ax.up.x * sp, f1.y * cp + ax.up.y * sp, f1.z * cp + ax.up.z * sp)
    u2 = vec(-f1.x * sp + ax.up.x * cp, -f1.y * sp + ax.up.y * cp, -f1.z * sp + ax.up.z * cp)
    return { forward = f2, right = r1, up = u2 }
end

function d2s(x) return string.format(x >= 10 and "%.0f" or "%.1f", x) end

sysOn = false
sweepOn = false

holdOn = false
lockPush = false
bootT = 0
wasPress = false
touchX, touchY, press = 0, 0, false
zin = 0
fovRad = math.pi / 180 * 35
animT = 0
seatEdge = {}

camPos = vec(0, 0, 0)
camAxes = axesFromYPR(0, 0, 0)
lockVelF = vec(0, 0, 0)
pvxH, pvyH = 0, 0
pvxD, pvyD = 0, 0
rngH = {}
tgts = {}
locked = false
trkCount, rawCount, radarsUp, bias = 0, 0, 0, 0
lockSpeedF = 0
lockScreenX, lockScreenY = nil, nil
outOfRange, hazard = false, false
lockFault = 0
hookNum, hookWas, hookT = 0, 0, -999
hookSlot = 2

w, h = 160, 96
function startRect() return (w - 46) / 2, h - 34, 46, 14 end
function barBtn(i) bbw = math.floor((w - 10) / 4) return 2 + (i - 1) * (bbw + 2), h - 13, bbw, 10 end

function seatTap(ch)
    st = INB(ch)
    fresh = st and not seatEdge[ch]
    seatEdge[ch] = st
    return fresh
end

function onTick()
    animT = animT + 1

    touchX, touchY = INN(7), INN(8)
    press = INN(9) >= 0.5
    tap = press and not wasPress
    wasPress = press

    tPwr, tSwp, tHld = seatTap(7), seatTap(8), seatTap(10)

    if tap then
        if sysOn then
            if isTouching(touchX, touchY, barBtn(1)) then sweepOn = not sweepOn end
            if isTouching(touchX, touchY, barBtn(3)) then holdOn = not holdOn end
            if isTouching(touchX, touchY, barBtn(4)) then sysOn, bootT, holdOn = false, 0, false end
        elseif isTouching(touchX, touchY, startRect()) then
            sysOn, bootT = true, 0
        end
    end

    if tPwr then
        if sysOn then sysOn, bootT, holdOn = false, 0, false else sysOn, bootT = true, 0 end
    end
    if sysOn then
        if tSwp then sweepOn = not sweepOn end
        if tHld then holdOn = not holdOn end
    end
    if sysOn and bootT < BOOT_TICKS then bootT = bootT + 1 end

    lockPush = sysOn and bootT >= BOOT_TICKS
        and (INB(9) or (press and isTouching(touchX, touchY, barBtn(2))))

    zin = INN(31)
    fovRad = ((1 - zin) * 2.1893 + 0.025) / 2

    camAxes = axesFromYPR(-INN(6) * pi2, INN(4) * pi2, INN(5) * pi2)

    camPos = vec(INN(1) + camAxes.forward.x * CAM_OFF_Y + camAxes.up.x * CAM_OFF_Z,
                 INN(3) + camAxes.forward.y * CAM_OFF_Y + camAxes.up.y * CAM_OFF_Z,
                 INN(2) + camAxes.forward.z * CAM_OFF_Y + camAxes.up.z * CAM_OFF_Z)

    live = sysOn and bootT >= BOOT_TICKS
    rawCount = INN(30)
    trkCount = live and rawCount or 0
    bias = live and INN(10) or 0
    radarsUp = live and INN(11) or 0
    locked = live and INB(1)

    -- unpack the status word: fault code + 8 * hooked track number
    dsg = live and INN(32) or 0
    lockFault = dsg % 8
    hookNum = math.floor(dsg / 8)
    hookSlot = locked and 1 or 2
    if hookNum ~= hookWas then hookT = animT end
    hookWas = hookNum

    for k = 1, 5 do
        px, py, pz = INN(12 + k * 3), INN(13 + k * 3), INN(14 + k * 3)
        tgts[k] = (live and not (px == 0 and py == 0 and pz == 0)) and vec(px, py, pz) or nil
    end

    lockVel = vec(INN(12) / 60, INN(13) / 60, INN(14) / 60)
    lockVelF = vec(lockVelF.x + VEL_ALPHA * (lockVel.x - lockVelF.x),
                   lockVelF.y + VEL_ALPHA * (lockVel.y - lockVelF.y),
                   lockVelF.z + VEL_ALPHA * (lockVel.z - lockVelF.z))
    if locked and tgts[1] then

        tgts[1] = vec(tgts[1].x + lockVelF.x * DISP_LEAD,
                      tgts[1].y + lockVelF.y * DISP_LEAD,
                      tgts[1].z + lockVelF.z * DISP_LEAD)
        lockSpeedF = hold(lockSpeedF, 60 * (lockVelF.x ^ 2 + lockVelF.y ^ 2 + lockVelF.z ^ 2) ^ .5, HOLD_SPD)
    else
        lockSpeedF, lockScreenX, lockScreenY = 0, nil, nil
    end

    outOfRange, hazard = live and INB(5), live and INB(6)

    -- pivot the camera toward the lock
    pvx, pvy = 0, 0
    if locked and tgts[1] then
        aim = vec(tgts[1].x - camPos.x, tgts[1].y - camPos.y, tgts[1].z - camPos.z)
        af, ar, au = dot(aim, camAxes.forward), dot(aim, camAxes.right), dot(aim, camAxes.up)
        if af > 0 then
            pvx = clamp(math.atan(ar, af) / pi2, -PIV_MAX, PIV_MAX)
            pvy = clamp(math.atan(au, (af * af + ar * ar) ^ .5) / pi2, -PIV_MAX, PIV_MAX)
        end
    end

    pvxH, pvyH = hold(pvxH, pvx, HOLD_PIV), hold(pvyH, pvy, HOLD_PIV)

    camAxes = pivotAxes(camAxes, pvxD, pvyD)

    OUN(1, AIM_X * pvxH / PIV_MAX)
    OUN(2, AIM_Y * pvyH / PIV_MAX)
    pvxD, pvyD = pvxH, pvyH
    OUN(3, zin)
    OUB(1, sysOn)
    OUB(2, sysOn and sweepOn)
    OUB(3, lockPush)
    OUB(4, sysOn and holdOn)

    OUB(5, live and INB(11))
    OUB(6, live and INB(12))
    OUB(7, INB(3))
    OUB(8, INB(4))
    OUB(9, live)
    OUB(13, locked)
end

function project(p)
    rel = vec(p.x - camPos.x, p.y - camPos.y, p.z - camPos.z)
    lx, ly, lz = dot(rel, camAxes.right), dot(rel, camAxes.forward), dot(rel, camAxes.up)
    if ly <= 0 then return 0, 0, false end
    return math.floor(w / 2 + (lx / ly) * (h / 2) / tanF + 0.5), math.floor(h / 2 - (lz / ly) * (h / 2) / tanF + 0.5),
        true, (lx * lx + ly * ly + lz * lz) ^ .5
end

function rotSquare(x, y, r, ang)
    px2, py2 = x + r * math.cos(ang), y + r * math.sin(ang)
    for i = 1, 4 do
        a = ang + i * (math.pi / 2)
        nx, ny = x + r * math.cos(a), y + r * math.sin(a)
        screen.drawLine(px2, py2, nx, ny)
        px2, py2 = nx, ny
    end
end

function drawButton(label, lit, r, g, b, x, y, bw, bh)
    hot = press and isTouching(touchX, touchY, x, y, bw, bh)
    if lit then screen.setColor(r * 0.16, g * 0.16, b * 0.16)
    else screen.setColor(UI_FILL_R, UI_FILL_G, UI_FILL_B) end
    if hot then screen.setColor(r * 0.3, g * 0.3, b * 0.3) end
    screen.drawRectF(x, y, bw, bh)
    if lit then screen.setColor(r, g, b) else screen.setColor(UI_DIM_R, UI_DIM_G, UI_DIM_B) end
    screen.drawRect(x, y, bw - 1, bh - 1)
    if lit then screen.setColor(r, g, b) else screen.setColor(UI_TXT_R, UI_TXT_G, UI_TXT_B) end
    screen.drawText(x + (bw - txtW(label)) / 2, y + (bh - 4) / 2, label)
end

function corners(x1, y1, x2, y2, len)
    for ax = 0, 1 do
        for ay = 0, 1 do
            cbx, cby = ax > 0 and x2 or x1, ay > 0 and y2 or y1
            screen.drawLine(cbx, cby, cbx + (ax > 0 and -len or len), cby)
            screen.drawLine(cbx, cby, cbx, cby + (ay > 0 and -len or len))
        end
    end
end

function centreText(y, s) screen.drawText((w - txtW(s)) / 2, y, s) end

function card()
    screen.setColor(UI_BG_R, UI_BG_G, UI_BG_B)
    screen.drawRectF(0, 0, w, h)
    screen.setColor(UI_DIM_R * 0.6, UI_DIM_G * 0.6, UI_DIM_B * 0.6)
    corners(3, 3, w - 4, h - 4, 8)
end

function onDraw()
    w, h = screen.getWidth(), screen.getHeight()
    tanF = math.tan(fovRad)

    if not sysOn then
        card()

        screen.setColor(UI_TXT_R, UI_TXT_G, UI_TXT_B)
        centreText(22, "NR TRACKING V1")
        screen.setColor(UI_ACC_R, UI_ACC_G, UI_ACC_B)
        screen.drawLine(w / 2 - 44, 30, w / 2 + 44, 30)

        blink = 0.55 + 0.45 * math.sin(animT * 0.06)
        screen.setColor(UI_DIM_R * blink, UI_DIM_G * blink, UI_DIM_B * blink)
        centreText(36, "STANDBY")

        drawButton("START", true, UI_GRN_R, UI_GRN_G, UI_GRN_B, startRect())
        return
    end

    if bootT < BOOT_TICKS then
        card()

        screen.setColor(UI_TXT_R, UI_TXT_G, UI_TXT_B)
        centreText(24, "SPINNING UP")

        bw = 96
        bx = (w - bw) / 2
        screen.setColor(UI_DIM_R * 0.5, UI_DIM_G * 0.5, UI_DIM_B * 0.5)
        screen.drawRect(bx, 40, bw - 1, 5)
        screen.setColor(UI_ACC_R, UI_ACC_G, UI_ACC_B)

        screen.drawRectF(bx + 1, 41, (bw - 2) * bootT / BOOT_TICKS, 4)

        screen.setColor(UI_DIM_R, UI_DIM_G, UI_DIM_B)
        centreText(52, "CONTACTS " .. string.format("%.0f", rawCount))
        return
    end

    barY = h - 16

    outOfView = false
    if hookNum > 0 and tgts[hookSlot] then
        fsx, fsy, fon = project(tgts[hookSlot])
        outOfView = not (fon and fsx >= 0 and fsx <= w and fsy >= 0 and fsy <= h)
    end

    -- project all slots to screen marks; the lock's screen position is
    -- hold-banded so the marker does not jitter
    marks = {}
    for k = 1, 5 do
        if tgts[k] then
            sx, sy, on, rng = project(tgts[k])
            if on then
                if k == 1 then
                    lockScreenX = hold(lockScreenX, sx, HOLD_PX)
                    lockScreenY = hold(lockScreenY, sy, HOLD_PX)
                    sx, sy = lockScreenX, lockScreenY
                end
                rngH[k] = hold(rngH[k], rng, HOLD_RNG)
                marks[#marks + 1] = { x = sx, y = sy, rng = rngH[k], pri = k == 1 and 1 or 2,
                                      hk = hookNum > 0 and k == hookSlot }
            elseif k == 1 then
                lockScreenX, lockScreenY = nil, nil
            end
        end
    end
    table.sort(marks, function(a, b) return a.pri < b.pri or (a.pri == b.pri and a.rng < b.rng) end)

    -- boresight cross, green when a mark sits under it
    if not sweepOn and not locked then
        cxc, cyc, gate = w / 2, h / 2, false

        for i = 1, #marks do
            if (marks[i].x - cxc) ^ 2 + (marks[i].y - cyc) ^ 2 < 64 then gate = true end
        end
        if gate then screen.setColor(UI_GRN_R, UI_GRN_G, UI_GRN_B)
        else screen.setColor(UI_DIM_R, UI_DIM_G, UI_DIM_B) end
        for a = 0, 3 do
            ux, uy = a < 2 and a * 2 - 1 or 0, a < 2 and 0 or a * 2 - 5
            screen.drawLine(cxc + ux * 3, cyc + uy * 3, cxc + ux * 7, cyc + uy * 7)
        end
        screen.drawRectF(cxc, cyc, 1, 1)
    end

    -- draw marks front-to-back, skipping any whose label box would overlap
    drawn = {}
    for i = 1, #marks do
        mk = marks[i]
        isLock = mk.pri == 1
        R = isLock and 7 or 3
        rad = isLock and (R + 4) or R
        dist = d2s(mk.rng) .. "m"
        lw = 2.5 * #dist
        spd = isLock and (d2s(lockSpeedF) .. "m/s") or nil
        lwv = spd and 2.5 * #spd or 0
        labW = math.max(rad, lw, lwv)
        ml, mr = mk.x - labW, mk.x + labW
        mt, mb = mk.y - rad - (mk.hk and 6 or 0), mk.y + rad + (spd and 13 or 7)
        clear = true
        for j = 1, #drawn do
            d = drawn[j]
            if not (mr < d.l or ml > d.r or mb < d.t or mt > d.b) then clear = false break end
        end
        if clear then
            ang = animT * 0.05
            if isLock then

                screen.setColor(255, 80, 30)
                screen.drawCircle(mk.x, mk.y, R)
                rotSquare(mk.x, mk.y, R - 1, ang)
                screen.drawRectF(mk.x, mk.y, 1, 1)
                screen.setColor(255, 175, 45)
                rotSquare(mk.x, mk.y, R - 1, -ang + 0.7854)
            else
                screen.setColor(UI_GRN_R, UI_GRN_G, UI_GRN_B)
                rotSquare(mk.x, mk.y, R, 0.7854)
            end
            screen.drawText(mk.x + 1 - lw, mk.y + rad + 2, dist)
            if spd then screen.drawText(mk.x + 1 - lwv, mk.y + rad + 8, spd) end

            if mk.hk then
                screen.setColor(UI_AMB_R, UI_AMB_G, UI_AMB_B)
                corners(mk.x - R - 3, mk.y - R - 3, mk.x + R + 3, mk.y + R + 3, 2)
                screen.drawText(mk.x - 6, mk.y - rad - 6, string.format("T%02d", hookNum))
            end
            drawn[#drawn + 1] = { l = ml, r = mr, t = mt, b = mb }
        end
    end

    if trkCount > 0 then screen.setColor(UI_GRN_R, UI_GRN_G, UI_GRN_B)
    else screen.setColor(UI_DIM_R, UI_DIM_G, UI_DIM_B) end
    screen.drawText(2, 2, "TRK " .. string.format("%.0f", trkCount))

    if hookNum > 0 then
        screen.setColor(255, locked and 80 or UI_AMB_G, locked and 30 or UI_AMB_B)
        screen.drawText(2, 9, string.format("TGT %02d", hookNum))
    end

    if locked then
        if radarsUp >= 3 then screen.setColor(UI_DIM_R, UI_DIM_G, UI_DIM_B)
        else screen.setColor(UI_AMB_R, UI_AMB_G, UI_AMB_B) end
        screen.drawText(2, 16, "RDR " .. string.format("%.0f", radarsUp) .. "/4")
    end

    if sweepOn then screen.setColor(UI_GRN_R, UI_GRN_G, UI_GRN_B); sm = "SWEEP"
    else screen.setColor(UI_AMB_R, UI_AMB_G, UI_AMB_B); sm = "PNT" end
    screen.drawText(w - txtW(sm) - 2, 2, sm)
    if holdOn then
        screen.setColor(UI_ACC_R, UI_ACC_G, UI_ACC_B)
        screen.drawText(w - txtW("HOLD") - 2, 9, "HOLD")
    end

    if bias > BIAS_WARN then
        screen.setColor(UI_RED_R, UI_RED_G, UI_RED_B)
        centreText(2, "RADAR SIGN? " .. d2s(bias) .. "m")
    end

    msg = nil
    if hookNum > 0 and (not locked or animT - hookT < 120) then

        blink = (animT - hookT < 30 and (animT - hookT) % 12 > 5) and 0.3 or 1
        screen.setColor(UI_AMB_R * blink, UI_AMB_G * blink, UI_AMB_B * blink)
        msg = string.format(locked and "LOCK TGT %02d" or "HOOK TGT %02d", hookNum)
    elseif not tgts[1] then
        blink = 0.55 + 0.45 * math.sin(animT * 0.1)
        screen.setColor(UI_AMB_R * blink, UI_AMB_G * blink, UI_AMB_B * 0.25 * blink)
        msg = sweepOn and "SWEEPING" or "NO LOCK"
    end
    if msg then centreText(barY - 6, msg) end

    warnY = 11

    if lockFault > 0 then
        screen.setColor(UI_RED_R, UI_RED_G, UI_RED_B)
        centreText(warnY, FAULTS[lockFault])
        warnY = warnY + 7
    end
    if outOfRange then
        screen.setColor(UI_AMB_R, UI_AMB_G, 0)
        centreText(warnY, "OUT OF RNG")
        warnY = warnY + 7
    end
    if hazard then
        screen.setColor(UI_RED_R, 40, 40)
        centreText(warnY, "HAZARD")
        warnY = warnY + 7
    end

    if outOfView then
        screen.setColor(255, 150, 0)
        centreText(warnY, "OUT OF VIEW")
    end

    screen.setColor(UI_DIM_R * 0.5, UI_DIM_G * 0.5, UI_DIM_B * 0.5)
    screen.drawLine(0, barY, w, barY)
    drawButton("SWEEP", sweepOn, UI_GRN_R, UI_GRN_G, UI_GRN_B, barBtn(1))
    drawButton("LOCK", locked, UI_RED_R, 110, 60, barBtn(2))
    drawButton("HOLD", holdOn, UI_ACC_R, UI_ACC_G, UI_ACC_B, barBtn(3))
    drawButton("STOP", true, UI_RED_R, UI_RED_G, UI_RED_B, barBtn(4))
end

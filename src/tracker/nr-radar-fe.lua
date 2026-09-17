-- nr-radar-fe.lua -- per-radar front end
--
-- One instance per radar (set RADAR to 1..8 before pasting). Converts the
-- radar block's polar contacts to world coordinates, merges detections of the
-- same object, and picks one contact to forward to the tracker.
--
-- The radar block reports up to 6 contacts as distance / azimuth / elevation
-- in the radar's own frame. The radars are mounted rolled 90 degrees, so the
-- block's azimuth channel sweeps the vehicle's vertical axis and its
-- elevation channel the horizontal one; RIG_SIDE carries the per-position
-- horizontal sign.
--
-- NUMBER INPUTS: i*4-3 .. i*4 per contact slot i = distance / azimuth /
-- elevation / time-since-detected; 25/27/26 mount position (raw physics-
-- sensor order), 28-30 mount pitch / roll / heading (turns).
-- BOOL INPUTS: 1..6 contact detected per slot.
-- OUTPUTS: numbers 1-3 world contact position; bool 1 contact valid,
-- bool 2 fresh sample this tick.

getN = input.getNumber
getB = input.getBool
setN = output.setNumber
setB = output.setBool
m = math
sine = m.sin
cosine = m.cos
TAU = m.pi * 2

RADAR = 1                 -- which rig position this instance occupies

BLOCK      = 0.25         -- m per block
RIG_X      = { -3, -2, 2, 3, -5, -4, 4, 5 }   -- radar offsets on the rig, blocks
RIG_Z      = -1
RIG_SIDE   = { -1, -1, 1, 1, -1, -1, 1, 1 }   -- horizontal-axis sign per position
MIN_RANGE  = 20           -- m: ignore returns off the rig itself
PICK_JUMP  = 60           -- m: max tick-to-tick jump before re-picking

SAME_OBJ   = 30           -- m: contacts closer than this are the same object

slX, slY, slZ = {}, {}, {}
ceX, ceY, ceZ = {}, {}, {}

function configure()
    SIDE = RIG_SIDE[RADAR]
    OFF_X, OFF_Y, OFF_Z = RIG_X[RADAR] * BLOCK, 0, RIG_Z * BLOCK
end
configure()

function axesFromYPR(yaw, pitch, roll)
    cy, sy = cosine(yaw), sine(yaw)
    cp, sp = cosine(pitch), sine(pitch)
    cr, sr = cosine(roll), sine(roll)
    axR = { cy * cr + sy * sp * sr, -sy * cr + cy * sp * sr, -cp * sr }
    axF = { sy * cp, cy * cp, sp }
    axU = { cy * sr - sy * cr * sp, -sy * sr - cy * cr * sp, cr * cp }
end

function toWorldX(lr, lf, lu) return lr * axR[1] + lf * axF[1] + lu * axU[1] end
function toWorldY(lr, lf, lu) return lr * axR[2] + lf * axF[2] + lu * axU[2] end
function toWorldZ(lr, lf, lu) return lr * axR[3] + lf * axF[3] + lu * axU[3] end

function onTick()

    mountX, mountY, mountZ = getN(25), getN(27), getN(26)
    mountPitch, mountRoll, mountHdg = getN(28), getN(29), getN(30)
    axesFromYPR(-mountHdg * TAU, mountPitch * TAU, mountRoll * TAU)

    radarX = mountX + toWorldX(OFF_X, OFF_Y, OFF_Z)
    radarY = mountY + toWorldY(OFF_X, OFF_Y, OFF_Z)
    radarZ = mountZ + toWorldZ(OFF_X, OFF_Y, OFF_Z)

    -- polar contacts to world points; the rolled mount swaps the channels
    best, bestD, alt, nsl = 0, 1e18, 0, 0
    for i = 1, 6 do
        dist = getN(i * 4 - 3)
        if getB(i) and dist >= MIN_RANGE then

            vert  = getN(i * 4 - 2) * TAU
            horiz = getN(i * 4 - 1) * SIDE * TAU

            cv = cosine(vert)
            lr = dist * cv * sine(horiz)
            lf = dist * cv * cosine(horiz)
            lu = dist * sine(vert)
            wx = radarX + toWorldX(lr, lf, lu)
            wy = radarY + toWorldY(lr, lf, lu)
            wz = radarZ + toWorldZ(lr, lf, lu)

            nsl = nsl + 1
            slX[nsl], slY[nsl], slZ[nsl] = wx, wy, wz
            if nsl == 1 then fresh = getN(i * 4) < 0.5 end
        end
    end

    -- merge contacts within SAME_OBJ of each other, then pick: prefer the
    -- contact nearest last tick's pick, fall back to the first
    for j = 1, nsl do
        sx, sy, sz, sn = slX[j], slY[j], slZ[j], 1
        for k = 1, nsl do
            if k ~= j and (slX[k] - slX[j]) ^ 2 + (slY[k] - slY[j]) ^ 2
                        + (slZ[k] - slZ[j]) ^ 2 <= SAME_OBJ ^ 2 then
                sx, sy, sz, sn = sx + slX[k], sy + slY[k], sz + slZ[k], sn + 1
            end
        end
        ceX[j], ceY[j], ceZ[j] = sx / sn, sy / sn, sz / sn

        if alt == 0 then alt, aX, aY, aZ = j, ceX[j], ceY[j], ceZ[j] end

        d = havePrev and ((ceX[j] - pvX) ^ 2 + (ceY[j] - pvY) ^ 2 + (ceZ[j] - pvZ) ^ 2) or 0
        if d < bestD then best, bestD, bx, by, bz = j, d, ceX[j], ceY[j], ceZ[j] end
    end

    if not havePrev or (aX - pvX) ^ 2 + (aY - pvY) ^ 2 + (aZ - pvZ) ^ 2 <= PICK_JUMP ^ 2
       or best == 0 or bestD > PICK_JUMP ^ 2 then
        best, bx, by, bz = alt, aX, aY, aZ
    end

    if best > 0 then
        setN(1, bx); setN(2, by); setN(3, bz)
        setB(1, true)
        pvX, pvY, pvZ, havePrev = bx, by, bz, true
    else

        setN(1, 0); setN(2, 0); setN(3, 0)
        setB(1, false)
        havePrev = false
    end

    setB(2, fresh)

end

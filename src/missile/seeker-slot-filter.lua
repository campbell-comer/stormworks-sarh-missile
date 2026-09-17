-- seeker-slot-filter.lua -- onboard radar contact filter
--
-- One instance per missile radar. The radar block reports up to 8 contacts,
-- but its output slots reshuffle whenever the contact count changes, so a
-- track cannot follow a slot number. This MC follows the OBJECT instead:
-- nearest-to-prediction association across slots with a short coast, plus a
-- ban list driven by the flight computer's reject signal.
--
-- NUMBER INPUTS: i*4-3 .. i*4-1 per slot i = distance / azimuth / elevation.
-- BOOL INPUTS: 1..8 contact detected per slot; 9 = reject command from
-- sarh-guidance (this contact is the wrong object: ban it, pick another).
-- NUMBER OUTPUTS: 1-3 distance / azimuth / elevation of the tracked contact
-- (zeros when none); 4 a 4-phase diagnostic mux of the best alternative
-- contact (tick 0: 10000 + trkSlot + 10*altSlot + 100*detections + 1000*bans,
-- ticks 1-3: alt distance / azimuth / elevation).

IN, INB, ON = input.getNumber, input.getBool, output.setNumber
sin, cos, abs, sqrt = math.sin, math.cos, math.abs, math.sqrt
TAU = math.pi * 2
SLOTS = 8

GATE_BASE = 20            -- m association gate, plus a range-proportional term
GATE_FRAC = 0.06

COAST_MAX = 6             -- ticks the track may coast on its own velocity

MIN_RNG = 10              -- plausibility bounds; also reject unwired zeros
MAX_RNG = 1000

ACQ_GATE = 2.5            -- gate multiplier on the first tick of a track

REJ_HOLD = 45             -- ticks a banned slot stays banned

REJ_N    = 3              -- consecutive reject ticks before acting

sok, sx, sy, sz, sd, sa, se, sage, ban = {}, {}, {}, {}, {}, {}, {}, {}, {}
for i = 1, SLOTS do sok[i], sage[i], ban[i] = false, 0, 0 end

trkN, missN, trkSlot, rejN = 0, 0, 0, 0
diagN = 0
lx, ly, lz, ld = 0, 0, 0, 0
vx, vy, vz = 0, 0, 0

outD, outA, outE, pick = 0, 0, 0, 0

function onTick()

    -- read all slots into local cartesian (radar frame)
    for i = 1, SLOTS do
        d, a, e = IN(i * 4 - 3), IN(i * 4 - 2), IN(i * 4 - 1)

        sok[i] = INB(i) and d >= MIN_RNG and d <= MAX_RNG and abs(a) <= 0.5 and abs(e) <= 0.5
        sage[i] = sok[i] and sage[i] + 1 or 0
        if sok[i] then
            h = d * cos(e * TAU)
            sx[i], sy[i], sz[i] = h * sin(a * TAU), h * cos(a * TAU), d * sin(e * TAU)
            sd[i], sa[i], se[i] = d, a, e
        end
    end

    for i = 1, SLOTS do
        if ban[i] > 0 then ban[i] = ban[i] - 1 end
    end

    -- reject command: after REJ_N consecutive ticks, ban the tracked slot's
    -- identity and drop the track so acquisition picks something else
    if INB(9) and trkSlot > 0 then
        rejN = rejN + 1
        if rejN >= REJ_N then
            ban[trkSlot] = REJ_HOLD
            trkN, trkSlot, missN, rejN = 0, 0, 0, 0
        end
    else
        rejN = 0
    end

    -- association: predict the last position forward one tick and take the
    -- nearest in-gate contact, preferring the slot it was in last tick
    pick = 0
    if trkN > 0 then
        px, py, pz = lx + vx, ly + vy, lz + vz
        gate = (GATE_BASE + GATE_FRAC * ld) * (trkN < 2 and ACQ_GATE or 1)
        best = gate * gate

        if trkSlot > 0 and sok[trkSlot] then
            dx, dy, dz = sx[trkSlot] - px, sy[trkSlot] - py, sz[trkSlot] - pz
            r2 = dx * dx + dy * dy + dz * dz
            if r2 < best then best, pick = r2, trkSlot end
        end
        for i = 1, SLOTS do

            if sok[i] and i ~= trkSlot and ban[i] == 0 then
                dx, dy, dz = sx[i] - px, sy[i] - py, sz[i] - pz
                r2 = dx * dx + dy * dy + dz * dz
                if r2 < best then best, pick = r2, i end
            end
        end
    end

    -- acquisition: oldest unbanned contact, or oldest contact at all
    if pick == 0 and trkN == 0 then

        for i = 1, SLOTS do
            if sok[i] and ban[i] == 0 and (pick == 0 or sage[i] > sage[pick]) then pick = i end
        end

        if pick == 0 then
            for i = 1, SLOTS do
                if sok[i] and (pick == 0 or sage[i] > sage[pick]) then pick = i end
            end
        end
        vx, vy, vz = 0, 0, 0
    end

    if pick > 0 then
        if trkN > 0 then vx, vy, vz = sx[pick] - lx, sy[pick] - ly, sz[pick] - lz end
        lx, ly, lz, ld = sx[pick], sy[pick], sz[pick], sd[pick]
        outD, outA, outE = sd[pick], sa[pick], se[pick]
        trkN, missN, trkSlot = trkN + 1, 0, pick
    else
        missN = missN + 1
        if trkN > 0 and missN <= COAST_MAX then
            -- coast on the last velocity; output zeros so downstream logic
            -- treats the gap honestly
            lx, ly, lz = lx + vx, ly + vy, lz + vz
            ld = sqrt(lx * lx + ly * ly + lz * lz)
        else
            trkN, trkSlot = 0, 0
        end
        outD, outA, outE = 0, 0, 0
    end

    ON(1, outD); ON(2, outA); ON(3, outE)

    -- diagnostic mux: best alternative contact (unbanned preferred, then oldest)
    diagN = (diagN + 1) % 4
    altSlot, detCount, banCount = 0, 0, 0
    for i = 1, SLOTS do
        if ban[i] > 0 then banCount = banCount + 1 end
        if sok[i] then
            detCount = detCount + 1
            if i ~= pick then
                if altSlot == 0
                   or (ban[i] == 0 and ban[altSlot] > 0)
                   or ((ban[i] > 0) == (ban[altSlot] > 0) and sage[i] > sage[altSlot]) then
                    altSlot = i
                end
            end
        end
    end
    if diagN == 0 then
        ON(4, 10000 + trkSlot + 10 * altSlot + 100 * detCount + 1000 * banCount)
    elseif altSlot == 0 then
        ON(4, 0)
    elseif diagN == 1 then
        ON(4, sd[altSlot])
    elseif diagN == 2 then
        ON(4, sa[altSlot])
    else
        ON(4, se[altSlot])
    end
end

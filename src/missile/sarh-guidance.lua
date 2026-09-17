-- sarh-guidance.lua -- SARH missile flight computer
--
-- One microcontroller. Datalink target state and onboard seeker in, fin
-- commands out. The datalink supplies target position and velocity from the
-- ground radar tracker; two fixed missile-mounted radars refine the aim point
-- in the terminal phase. The guidance law is ZEM/TPN in direction form
-- (calcTang): with lead gain L, the commanded cross-track velocity is
-- (1+L)*tgtTang - L*misCross, i.e. proportional navigation with N = 1 + L.
-- It uses relative position and velocity only and assumes nothing above
-- constant target velocity.
--
-- Conventions: world frame is X east / Y north / Z up, positions in metres,
-- angles at IO boundaries in turns. The physics sensor reports x east / y up /
-- z north, so inputs 7-9 and 15-17 are remapped in-script.
--
-- NUMBER INPUTS
--   1-3    datalink target position (world m)
--   4-6    datalink target velocity (m/s); (0,0,0) = not provided, velocity is
--          then derived by differencing 1-3 over TGT_WIN ticks
--   7-9    own position, raw physics-sensor order (remapped 1,3,2)
--   10-12  own pitch / roll / compass heading (turns)
--   13-14  fin telemetry taps (radar 2), read by no logic
--   15-17  own angular velocity, raw physics-sensor order
--   18-19  fin telemetry taps (radar 1), read by no logic
--   20-22  ground-truth target position, test rig only (already east/north/up)
--   23-25  seeker radar 2: distance / azimuth / elevation
--   26-28  seeker radar 1: distance / azimuth / elevation
--   29     radar 1 slot-filter diagnostic mux, read by no logic
--   30     LAUNCHED / ACTIVE level: >= 0.5 armed. A number rather than a bool
--          so it rides an existing composite write block (an extra bool write
--          block would cost every upstream signal one tick)
--   31-32  spare
--
-- Seeker geometry: the radars are mounted rolled 90 degrees, so the block's
-- azimuth channel sweeps the missile's vertical axis and its elevation channel
-- sweeps the horizontal. Azimuth/elevation are turns in the radar's own frame
-- and roll with the airframe. SKR_AZ_SIGN is the single mounting sign.
--
-- NUMBER OUTPUTS: 1 yaw fin, 2 pitch fin, 3 roll fin, remainder telemetry.
-- The fin buses are not the same scale: outputs 1-2 feed rocket fins, which
-- scale composite input by 0.1 and clamp to [-1,1], so +-10 is full deflection
-- (MAIN_FIN_FULL). Output 3 is the roll effector, written directly, +-1 full.
--
-- BOOL OUTPUT 1: seeker reject. Wired to both seeker-slot-filter MCs' bool
-- input 9; true when the seeker contact disagrees with a fresh datalink aim
-- beyond the phantom-veto bounds ("ban that identity, try another slot").
--
-- PROPERTIES: P, D (fin PID gains), Tick Lag (datalink transport delay to
-- extrapolate over), Offset X/Y/Z (physics sensor to centre of mass), Safety
-- Ticks (arm delay after launch), Missile Lifetime (seconds, 0 = unlimited).

m = math
IN = input.getNumber

ON = output.setNumber
ONB = output.setBool

PROPN = property.getNumber

sin, cos, atan, asin, sqrt, abs, max, min, huge =
    m.sin, m.cos, m.atan, m.asin, m.sqrt, m.abs, m.max, m.min, m.huge
pi = m.pi
pi2 = pi * 2
DEG = 180 / pi

BUILD = 81302             -- build stamp, published on channel 21

MIN_SPD  = 0.5            -- m/tick floor for guidance math

MAIN_FIN_FULL     = 10    -- full deflection on the yaw/pitch fin bus
TICKS_PER_SECOND  = 60

-- Roll hold: rate loop inside an angle loop, plus an integrator that only
-- accumulates near wings-level so it cannot wind up during hard manoeuvres.
ROLL_P = 0.15
ROLL_I = 0.40
ROLL_D = 0.053

ROLL_RATE_MAX = min(max(1.8 * ROLL_P, 0.20), 1.20)

ROLL_KI_RATE  = 5.00      -- integrator gates: |rate| and |angle| must be small
ROLL_KI_ANGLE = 0.50

ROLL_DEADBAND = 0.005     -- turns of bank ignored entirely

ROLL_LEVEL_MIN_COS  = 0.05  -- fade the bank loop out near vertical flight,
ROLL_LEVEL_COS_GAIN = 8.0   -- where bank angle is ill-defined
ROLL_FIN_FULL = 1.00

ROLL_MAX = ROLL_FIN_FULL
ROLL_KI_MAX = 0.60 * ROLL_MAX

ROLL_DOB_G     = 3.2      -- rate-demand feedforward divisor

RTLP_HOLD = 300           -- ticks of datalink coast at full guidance authority
RTLP_FADE = 120           -- then fade to the flight stabiliser over this many

STAB_FLAT = 0.05          -- m/tick of horizontal velocity below which the
                          -- stabiliser falls back to the nose direction
STAB_LATCH_MIN = 120      -- m/s: latch the stabiliser heading above this speed

-- Gain schedule: control effectiveness grows with dynamic pressure, so loop
-- gains shrink as speed builds.
SCHED_VREF = 340
SCHED_VMIN = 60.0
SCHED_MIN  = 0.25
SCHED_MAX  = 2.00

gainScale = 1

-- Navigation authority schedule. More authority at speed means harder pulls,
-- which bleed speed, which costs lateral acceleration (a_lat ~ fin * V^2):
-- an induced-drag spiral. The flat NAV_AUTH_MIN cap up to ~350 m/s and the
-- 1/V falloff above it are the brake on that spiral; NAV_AUTH_HIMIN keeps
-- terminal authority alive at the top of the envelope.
NAV_LIN      = 154
NAV_AUTH_MIN = 0.55
NAV_AUTH_HIMIN = 0.25

navScale = NAV_AUTH_MIN

-- Terminal authority boost, active only while the seeker is driving the aim
-- point (its bearing error shrinks with range, so the terminal command is
-- worth tracking harder). Ramped in below TERM_R rather than stepped, and
-- capped at NAV_AUTH_MIN, a gain the airframe already flies.
TERM_SKR = 1.8            -- multiplier at full ramp; 1.0 disables
TERM_R   = 200            -- m: ramp starts here (matches SKR_AIM_MAX so the
                          -- aim handover and the gain change never share a tick)
TERM_RW  = 50             -- m of ramp width

TERM_SK1 = TERM_SKR - 1
navUse = NAV_AUTH_MIN     -- published on ch 32

-- Seeker acceptance chain.
SKR_AZ_SIGN = -1          -- the one mounting sign in the seeker path; re-derive
                          -- after any rebuild (a flip steers away from target)

SKR_ANG_MAX = 0.30        -- turns. Slot-shift guard: when the radar block's
                          -- contact count changes its output slots move, and a
                          -- bearing channel can briefly carry a distance

SKR_GATE = 35             -- degrees of seeker/datalink disagreement tolerated
                          -- when the feed is stale but usable

-- Phantom veto: with a fresh datalink the seeker contact must sit within
-- these metric bounds of the datalink aim, measured transverse to the line of
-- sight (radial disagreement is where the seeker's legitimate improvement
-- lives, so it is deliberately not tested).
SKR_DL_PMAX = 25          -- m transverse
SKR_DL_ZMAX = 12          -- m vertical (phantoms tend to climb)
SKR_DL_MFRESH = 10        -- ticks of coast within which the metric veto applies

SKR_ON = 3                -- consecutive accepted ticks before the seeker may
                          -- take over the aim point

skrN, skrOk, skrDev = 0, false, 180

SKR_RNG_MIN = 10          -- m; also rejects the unwired-input-reads-0 case
SKR_RNG_MAX = 900

SKR_AIM_MAX = 200         -- m: seeker drives the aim point only inside this

SKR_DL_FRESH = 45         -- ticks of coast beyond which a stale datalink may
                          -- no longer veto a live seeker

-- Seeker self-track: a short memory of accepted contacts, so acceptance can
-- also be judged against the seeker's own recent history.
SKR_TRK_GATE = 40
SKR_TRK_MAX  = 200
SKR_TRK_COAST = 15

SKR_LAG = 2               -- ticks: seeker measurements are paired with the
                          -- pose from this many ticks ago, then advanced by
                          -- target velocity to now

FIN_TAP = 1               -- telemetry mux select for channel 11 (0 = seeker diag)

finPh = 0
FIN_CH = {18, 19, 13, 14}

skrTrk, skrTrkAge, skrRng, skrCand = nil, 0, 0, nil
pRight, pFwd, pUp, pOwn, poseN = nil, nil, nil, nil, false
p2Right, p2Fwd, p2Up, p2Own = nil, nil, nil, nil

-- Guidance velocity source: when DLPN_W > 0 the law's target velocity is
-- differenced from the datalink POSITION stream over this window (fresher
-- than the velocity channel by several ticks); the velocity channel still
-- drives prediction, seeker advance and the warm-up fallback.
DLPN_W = 0

dlpnHist = {}
dlpnN    = 0
dlpnVel  = nil

PN_LEAD      = 0.6        -- L in the ZEM/TPN command; N = 1 + L
PN_CROSS_MAX = 0.7        -- cap on cross-track command as a fraction of speed

REFUSE_MAX = 120          -- ticks calcTang may refuse (target tangential speed
                          -- exceeds ours) before falling back to pure pursuit
COAST_FLOOR = 150         -- m altitude floor for coasting through a refusal

refuseN    = 0
CUT_NEAR   = 60           -- closest-approach detection: once inside CUT_NEAR
CUT_MARGIN = 20           -- and opening by CUT_MARGIN, zero the fins for the
                          -- prox fuse and stop guiding

function navAuthority(spd)
    return clamp(NAV_LIN / max(spd - 15, 1), NAV_AUTH_HIMIN, NAV_AUTH_MIN)
end

function updateGainScale(spd)
    gainScale = clamp(SCHED_VREF / max(spd, SCHED_VMIN), SCHED_MIN, SCHED_MAX)
    navScale = navAuthority(spd)
end

CMD_ANGLE_MAX    = pi * 0.5   -- softAngle asymptote
CMD_REVERSE_HOLD = pi * 0.75  -- commands further astern than this hold the
                              -- current turn direction instead of flipping

-- Self-track guard: reject a datalink "target" that is actually this missile
-- (co-moving, inside a tube behind the nose), which happens when the ground
-- tracker locks onto the outgoing round.
SELF_TUBE  = 200
SELF_LEN   = 700
SELF_AHEAD = 25
SELF_DV    = 0.5
SELF_COS   = 0.5
SELF_N     = 5
KEEPOUT   = 120           -- reject targets near the launch point once clear
ARM_SEP   = 60
RAIL_SPD  = 0.1           -- m/tick: below this the missile is still on the rail

function Vec(x, y, z) return {x = x, y = y, z = z} end
function zeroVec() return Vec(0, 0, 0) end

function vAdd(a, b)    return Vec(a.x + b.x, a.y + b.y, a.z + b.z) end
function vSub(a, b)    return Vec(a.x - b.x, a.y - b.y, a.z - b.z) end
function vScale(a, s)  return Vec(a.x * s, a.y * s, a.z * s) end
function vDiv(a, s)    return vScale(a, 1 / s) end
function vDot(a, b)    return a.x * b.x + a.y * b.y + a.z * b.z end
function vLen(a)  return sqrt(a.x * a.x + a.y * a.y + a.z * a.z) end
function vNorm(a) return vDiv(a, vLen(a)) end

function isZeroVec(a) return a.x == 0 and a.y == 0 and a.z == 0 end

function removeNaN(x) return (x ~= x or abs(x) == huge) and 0 or x end
function clamp(x, lo, hi) return min(max(x, lo), hi) end
function wrapTurns(x) return (x + 0.5) % 1 - 0.5 end
function wrapRad(x) return (x + pi) % pi2 - pi end
function softAngle(x)
    return x / sqrt(1 + (x / CMD_ANGLE_MAX) ^ 2)
end

-- Body axes from the physics sensor's pitch/roll/heading (turns). Heading is
-- negated: the sensor's compass runs opposite to the yaw convention here.
function axesFromPhysics(pitchT, rollT, headingT, yaw, pitch, roll, cy, sy, cp, sp, cr, sr)
    yaw, pitch, roll = -headingT * pi2, pitchT * pi2, rollT * pi2
    cy, sy = cos(yaw), sin(yaw)
    cp, sp = cos(pitch), sin(pitch)
    cr, sr = cos(roll), sin(roll)
    return Vec(sy * cp, cy * cp, sp),
           Vec(cy * cr + sy * sp * sr, -sy * cr + cy * sp * sr, -cp * sr),
           Vec(cy * sr - sy * cr * sp, -sy * sr - cy * cr * sp, cr * cp)
end

-- Seeker bearing to world direction. Channels are crossed because the mount
-- is rolled 90 degrees: called as seekDir(<horizontal ch>, <vertical ch>).
function seekDir(azIn, elIn, a, e, m)
    a, e = IN(azIn), IN(elIn)
    m = min(abs(a), abs(e))
    -- reject unwired (exact 0) and slot-shifted (angle out of range) samples
    if m < 1e-12 or max(abs(a), abs(e)) > SKR_ANG_MAX then return nil end
    a, e = SKR_AZ_SIGN * a * pi2, e * pi2
    return vNorm(skrToWorld(sin(a) * cos(e), cos(a) * cos(e), sin(e)))
end

function skrToWorld(x, y, z)
    return Vec(
        sRight.x * x + sFwd.x * y + sUp.x * z,
        sRight.y * x + sFwd.y * y + sUp.y * z,
        sRight.z * x + sFwd.z * y + sUp.z * z
    )
end

function seekPos(dIn, azIn, elIn, d, dir)
    d = IN(dIn)
    if d < SKR_RNG_MIN or d > SKR_RNG_MAX then return nil end
    dir = seekDir(azIn, elIn)
    return dir and vAdd(sOwn, vScale(dir, d)) or nil
end

function projectOnPlane(v, n)
    return vAdd(v, vScale(n, -vDot(n, v) / vDot(n, n)))
end

-- PD controller. The derivative is a TWO-tick difference, halved: a one-tick
-- difference has maximum gain at the Nyquist frequency (an error alternating
-- +-1 returns +-2) and drove the fins into a tick-rate limit cycle; the
-- two-tick form has a zero there, at half a tick of group delay.
function newPID(kp, kd)
    return {
        p = kp, d = kd,
        lastErr = 0, prevErr = 0,
        run = function(self, setpoint, process, scale)
            local err, der
            scale = scale or 1
            err  = setpoint - process
            der  = wrapRad(err - self.prevErr) * 0.5
            self.prevErr = self.lastErr
            self.lastErr = err
            return err * self.p * scale + der * self.d * scale
        end
    }
end

-- Mounting signs live here and only here; gains stay magnitudes so a negative
-- gain can never hide a flipped fin. Re-check after any rebuild.
MAIN_FIN_YAW_SIGN   = -1
MAIN_FIN_PITCH_SIGN = 1
ROLL_FIN_SIGN       = -1
PROP_P, PROP_D = abs(PROPN("P")), abs(PROPN("D"))
yawPID   = newPID(PROP_P, PROP_D)
pitchPID = newPID(PROP_P, PROP_D)

EXTRAP = PROPN("Tick Lag")
PHYSICS_OFFSET = Vec(PROPN("Offset X"), PROPN("Offset Y"), PROPN("Offset Z"))

SAFETY_TICKS = max(0, PROPN("Safety Ticks"))

LIFE_T = PROPN("Missile Lifetime")
LIFE_TICKS = LIFE_T > 0 and LIFE_T * TICKS_PER_SECOND or huge

launchTk = 0

primed   = false
rawOn    = false
prevRaw  = zeroVec()
lastRx   = zeroVec()
holdN    = 0
VEL_WIN = 4               -- ticks of own-position differencing for velocity

ownHistX, ownHistY, ownHistZ = {}, {}, {}
TGT_WIN      = 4          -- same, for the derived target velocity fallback

TGT_STEP_MAX = 8          -- m: reject single-tick steps in the datalink feed
TGT_HOLD_MAX = 3          -- for up to this many ticks (then accept: it moved)

tgtHist   = {}
tgtPrimed = false

rtOk   = false
rtPos  = zeroVec()
rtVel  = zeroVec()
coast  = 0

selfN     = 0
selfLatch = false
launchPos = zeroVec()
status = 3                -- published on ch 13; see status assignments below

ownSpd    = 0
velFromLink = false
stepRej   = false
crossCap  = false
asternHold = false
finSat    = false

rollAngle  = 0
rollRate   = 0
rateDemand = 0
rateErr    = 0
rollLevelWeight = 1
rollIntg   = 0
rollRaw    = 0
rollSat    = false
yawSide    = 1
cmdYawUse  = 0
rateHist   = {}
rateSlot   = 1
rateFilled = 0
sensorMean = 0
for slot = 1, 8 do rateHist[slot] = 0 end
rollPrevAng = 0
stabHold = nil
tgtRangeNow = 1e9
missMin     = 1e9
postMiss    = false

ROLL_RATE_TAPS = 4        -- even boxcar on roll rate: immune to tick-rate noise

function updateRoll(pitchT, omega)
    -- Bank angle is 2-tap averaged, wrap-safely, for the same Nyquist reason
    -- as the PID derivative: the roll input alternates several degrees tick
    -- to tick and the bank loop was chattering across its own deadband.
    rollRawAng = atan(-rightAxis.z, upAxis.z) / pi2
    rollAngle = rollPrevAng + wrapTurns(rollRawAng - rollPrevAng) * 0.5
    rollPrevAng = rollRawAng
    rateHist[rateSlot] = vDot(omega, fwdAxis)
    rateSlot = rateSlot % ROLL_RATE_TAPS + 1
    rateFilled = min(rateFilled + 1, ROLL_RATE_TAPS)
    sensorMean = 0
    for slot = 1, rateFilled do sensorMean = sensorMean + rateHist[slot] end
    rollRate = sensorMean / max(rateFilled, 1)
    rollLevelWeight =
        clamp((abs(cos(wrapTurns(pitchT) * pi2)) - ROLL_LEVEL_MIN_COS)
              * ROLL_LEVEL_COS_GAIN, 0, 1)
end

function rollControl()
    bankErr  = rollAngle - clamp(rollAngle, -ROLL_DEADBAND, ROLL_DEADBAND)
    angleErr = sin(bankErr * pi2) * 2 * rollLevelWeight
    rateDemand = clamp(-ROLL_P * angleErr, -ROLL_RATE_MAX, ROLL_RATE_MAX)
    rateErr = rateDemand - rollRate
    -- integrate only near wings-level, and never against saturation
    if abs(rollAngle) < ROLL_KI_ANGLE and abs(rollRate) < ROLL_KI_RATE
       and rollLevelWeight >= 1 and not (rollSat and rollRaw * rateErr > 0) then
        rollIntg = clamp(rollIntg + ROLL_I * gainScale * rateErr / TICKS_PER_SECOND,
                         -ROLL_KI_MAX, ROLL_KI_MAX)
    end

    rollRaw = rateDemand * gainScale / ROLL_DOB_G + ROLL_D * gainScale * rateErr + rollIntg
    rollSat = abs(rollRaw) > ROLL_MAX
    return clamp(rollRaw, -ROLL_MAX, ROLL_MAX)
end

function writeFins(yawFin, pitchFin)
    -- clamp the yaw/pitch pair by magnitude so saturation preserves direction
    finMag = sqrt(yawFin ^ 2 + pitchFin ^ 2)
    finSat = finMag > MAIN_FIN_FULL
    if finSat then
        yawFin   = yawFin   * MAIN_FIN_FULL / finMag
        pitchFin = pitchFin * MAIN_FIN_FULL / finMag
    end
    rollFin = ROLL_FIN_SIGN * rollControl()
    ON(1, yawFin)
    ON(2, pitchFin)
    ON(3, rollFin)
end

function clearCommandDebug()
    ON(8, 0); ON(9, 0); ON(10, 0)
    ONB(1, skrRej)
    ON(32, navScale)
    ON(4, skrRng)
end

function predictTarget(n)
    return vAdd(rtPos, vScale(rtVel, n)), rtVel
end

-- Flight stabiliser fallback: hold the horizontal velocity direction (or the
-- nose direction when nearly stationary).
function stabDirection(h)
    h = Vec(misVel.x, misVel.y, 0)
    if vLen(h) < STAB_FLAT then h = Vec(fwdAxis.x, fwdAxis.y, 0) end
    return vLen(h) > 1e-6 and vNorm(h) or nil
end

-- The guidance law. Splits missile and target velocity into components
-- tangential to the line of sight, commands cross-track velocity
-- tgtTang - PN_LEAD * (misCross - tgtTang), fills the rest of the speed
-- budget along the LOS. Returns nil (refuses) when the target's tangential
-- speed exceeds the missile's total speed.
function calcTang(relPos, tgtVel, misVel)
    los     = vNorm(relPos)
    misSpd  = max(vLen(misVel), MIN_SPD)
    tgtTang = projectOnPlane(tgtVel, los)
    crossCap = false
    if misSpd ^ 2 - vLen(tgtTang) ^ 2 < 0 then return nil end
    misCross = projectOnPlane(misVel, los)

    pnErr = vSub(misCross, tgtTang)

    crossCmd = vAdd(tgtTang, vScale(pnErr, -PN_LEAD))
    ccLen = vLen(crossCmd)
    crossCap = ccLen > PN_CROSS_MAX * misSpd
    if crossCap then
        crossCmd = vScale(crossCmd, PN_CROSS_MAX * misSpd / ccLen)
        ccLen    = vLen(crossCmd)
    end
    ortLen = (misSpd ^ 2 - ccLen ^ 2) ^ .5
    return vAdd(vScale(los, ortLen), crossCmd)
end

function retiredTick() end

function onTick()

    launchTk = IN(30) >= .5 and launchTk + 1 or 0

    if launchTk > LIFE_TICKS then
        for channel = 1, 32 do ON(channel, 0) end
        ONB(1, false)
        onTick = retiredTick
        return
    end

    -- pre-arm: outputs zeroed, all state reset, so dropping input 30 to 0
    -- fully re-arms a test rig
    if launchTk <= SAFETY_TICKS then
        for channel = 1, 32 do ON(channel, 0) end
        ON(13, 6)
        ON(21, BUILD)
        ONB(1, false)

        rollRate, rollIntg, rollSat = 0, 0, false
        rateDemand, rateErr = 0, 0
        cmdYawUse = 0
        ownSpd = 0
        holdN = 0
        rateFilled, rateSlot, sensorMean = 0, 1, 0
        for slot = 1, ROLL_RATE_TAPS do rateHist[slot] = 0 end
        rollPrevAng = 0
        stabHold = nil
        postMiss, missMin, tgtRangeNow = false, 1e9, 1e9
        refuseN = 0
        dlpnN = 0

        navUse = NAV_AUTH_MIN
        gainScale, navScale = 1, NAV_AUTH_MIN

        skrN, skrOk, skrDev = 0, false, 180

        pOwn, p2Own, poseN = nil, nil, false

        skrTrk, skrTrkAge, skrRng, skrCand = nil, 0, 0, nil

        yawPID.prevErr, yawPID.lastErr = 0, 0
        pitchPID.prevErr, pitchPID.lastErr = 0, 0
        return
    end

    stepRej, crossCap, asternHold, finSat = false, false, false, false

    tgtRaw = Vec(IN(1), IN(2), IN(3))
    linkVel = Vec(IN(4) / 60, IN(5) / 60, IN(6) / 60)
    pitchT, rollT, headingT = IN(10), IN(11), IN(12)

    -- keep the two previous poses; seeker samples lag the pose by SKR_LAG
    if poseN then
        p2Right, p2Fwd, p2Up, p2Own = pRight, pFwd, pUp, pOwn
        pRight, pFwd, pUp, pOwn = rightAxis, fwdAxis, upAxis, ownPos
    end
    poseN = true
    fwdAxis, rightAxis, upAxis = axesFromPhysics(pitchT, rollT, headingT)

    sRight, sFwd, sUp = rightAxis, fwdAxis, upAxis
    omegaVec = Vec(IN(15), IN(17), IN(16))
    updateRoll(pitchT, omegaVec)

    rawPos = Vec(IN(7), IN(9), IN(8))
    ownPos = vAdd(rawPos, skrToWorld(PHYSICS_OFFSET.x, PHYSICS_OFFSET.y, PHYSICS_OFFSET.z))
    ownX, ownY, ownZ = ownPos.x, ownPos.y, ownPos.z

    truPos = Vec(IN(20), IN(21), IN(22))

    ON(15, ownX); ON(16, ownY); ON(17, ownZ)
    ON(18, pitchT); ON(19, IN(28)); ON(20, headingT)

    ON(22, rollT)

    ON(23, truPos.x); ON(24, truPos.y); ON(25, truPos.z)

    if not primed then
        for slot = 1, VEL_WIN + 1 do
            ownHistX[slot], ownHistY[slot], ownHistZ[slot] = ownX, ownY, ownZ
        end
        launchPos = Vec(ownX, ownY, ownZ)
        primed = true
    end

    -- own velocity from a VEL_WIN-tick position difference
    for slot = 1, VEL_WIN do
        ownHistX[slot], ownHistY[slot], ownHistZ[slot] =
            ownHistX[slot + 1], ownHistY[slot + 1], ownHistZ[slot + 1]
    end
    ownHistX[VEL_WIN + 1], ownHistY[VEL_WIN + 1], ownHistZ[VEL_WIN + 1] = ownX, ownY, ownZ
    misVel = Vec((ownX - ownHistX[1]) / VEL_WIN,
                 (ownY - ownHistY[1]) / VEL_WIN,
                 (ownZ - ownHistZ[1]) / VEL_WIN)
    misLen = vLen(misVel)
    ownSpd = misLen * TICKS_PER_SECOND
    updateGainScale(ownSpd)

    -- raw seeker telemetry echoes
    ON(26, IN(24)); ON(31, IN(25))
    ON(14, IN(26)); ON(27, IN(23))

    if FIN_TAP > 0 then
        finPh = (finPh + 1) % 5
        ON(11, finPh == 0 and 30000 + FIN_TAP or IN(FIN_CH[finPh]))
    else
        ON(11, IN(29))
    end
    if misLen < RAIL_SPD then
        launchPos = Vec(ownX, ownY, ownZ)
    end

    -- Datalink intake: step rejection, then a TGT_WIN-tick derived velocity
    -- as fallback when the velocity channels are unwired.
    if isZeroVec(tgtRaw) then
        rawOn, derVel = false, zeroVec()
    else
        if not rawOn then prevRaw, lastRx, holdN = tgtRaw, tgtRaw, 0 end
        stepRej = holdN < TGT_HOLD_MAX
                  and vLen(vSub(tgtRaw, lastRx)) > TGT_STEP_MAX
        lastRx = tgtRaw
        if stepRej then
            holdN = holdN + 1
            tgtRaw = prevRaw
        else
            holdN = 0
        end
        for slot = 1, TGT_WIN do
            tgtHist[slot] = tgtHist[slot + 1]
        end
        tgtHist[TGT_WIN + 1] = tgtRaw
        if not tgtPrimed then
            for slot = 1, TGT_WIN + 1 do tgtHist[slot] = tgtRaw end
            tgtPrimed = true
        end
        derVel  = vDiv(vSub(tgtRaw, tgtHist[1]), TGT_WIN)
        prevRaw = tgtRaw
        rawOn   = true
    end

    velFromLink = not isZeroVec(linkVel)
    candVel = velFromLink and linkVel or derVel
    candSpd = vLen(candVel)
    ON(28, candVel.x * TICKS_PER_SECOND)
    ON(29, candVel.y * TICKS_PER_SECOND)
    ON(30, candVel.z * TICKS_PER_SECOND)

    -- self-track guard: a "target" co-moving with us inside a tube behind the
    -- nose is this missile as seen by the ground tracker
    selfHit = false
    if rawOn then
        misSp  = misLen
        candSp = candSpd
        if misSp > RAIL_SPD and candSp > 1e-9 then
            axis  = vScale(misVel, 1 / misSp)
            rel   = vSub(tgtRaw, ownPos)
            along = vDot(rel, axis)
            selfHit = along < SELF_AHEAD and along > -SELF_LEN
                      and vLen(vSub(rel, vScale(axis, along))) < SELF_TUBE
                      and abs(candSp - misSp) < SELF_DV
                      and vDot(candVel, axis) / candSp > SELF_COS
        end
    end
    if selfHit then
        selfN = min(selfN + 1, SELF_N * 3)
    else
        selfN = max(selfN - 2, 0)
    end
    if selfN >= SELF_N then selfLatch = true elseif selfN == 0 then selfLatch = false end

    keepBad = rawOn
              and vLen(vSub(Vec(ownX, ownY, ownZ), launchPos)) > ARM_SEP
              and vLen(vSub(tgtRaw, launchPos)) < KEEPOUT

    accepted = rawOn and not selfLatch and not keepBad

    if accepted then
        rtOk = true
        rtPos, rtVel, coast = tgtRaw, candVel, 0
        status = 0

        for slot = 1, DLPN_W do dlpnHist[slot] = dlpnHist[slot + 1] end
        dlpnHist[DLPN_W + 1] = rtPos
        dlpnN = dlpnN + 1
    else
        coast = coast + 1
        dlpnN = 0
        -- 1 self-track latched, 2 keepout, 3 never had a target, 4 coasting
        status = selfLatch and 1 or (keepBad and 2 or (rtOk and 4 or 3))
    end

    if postMiss then status = 8 end

    ON(12, IN(27))

    ON(13, status)
    ON(5, rtPos.x); ON(6, rtPos.y); ON(7, rtPos.z)

    guideDir, authority = nil, 0
    dlpnVel = nil

    skrOk, skrDev, skrRng, skrCand, skrRej = false, 180, 0, nil, false

    tgtPos = zeroVec()
    if rtOk then
        tgtPos, tgtVel = predictTarget(coast + EXTRAP)
        authority = clamp(1 - (coast - RTLP_HOLD) / RTLP_FADE, 0, 1)

        dlRel   = vSub(tgtPos, ownPos)
        dlRange = vLen(dlRel)

        -- pair seeker channels with the pose from SKR_LAG ticks ago
        sOwn = ownPos
        if SKR_LAG > 1 and p2Own then
            sRight, sFwd, sUp, sOwn = p2Right, p2Fwd, p2Up, p2Own
        elseif SKR_LAG > 0 and pOwn then
            sRight, sFwd, sUp, sOwn = pRight, pFwd, pUp, pOwn
        end

        -- full position from either radar when the range is valid, else a
        -- bearing-only candidate at datalink range
        sp1, sp2 = seekPos(26, 28, 27), seekPos(23, 25, 24)
        skrPos = (sp1 and sp2) and vScale(vAdd(sp1, sp2), 0.5) or sp1 or sp2
        skr1, skr2 = seekDir(28, 27), seekDir(25, 24)
        skrDir = (skr1 and skr2) and vNorm(vAdd(skr1, skr2)) or skr1 or skr2

        skrCand = skrPos or ((skrDir and dlRange > 1e-6)
                             and vAdd(sOwn, vScale(skrDir, dlRange)) or nil)

        if SKR_LAG > 0 and skrCand then
            skrCand = vAdd(skrCand, vScale(tgtVel, SKR_LAG))
            if skrPos then skrPos = vAdd(skrPos, vScale(tgtVel, SKR_LAG)) end
        end
        if skrDir and dlRange > 1e-6 then
            skrDev = vLen(vSub(skrDir, vScale(dlRel, 1 / dlRange))) * DEG
        end

        -- acceptance: metric phantom veto against a fresh datalink, angular
        -- gate against a stale one, self-track continuity beyond that
        if skrCand then
            dlAgree = skrDev < SKR_GATE
            if coast <= SKR_DL_MFRESH and dlRange > 1e-6 then
                sedOff  = vSub(skrCand, tgtPos)
                dlAgree = vLen(projectOnPlane(sedOff, dlRel)) < SKR_DL_PMAX
                          and abs(sedOff.z) < SKR_DL_ZMAX

                skrRej = not dlAgree
            end
            if skrTrk and skrTrkAge <= SKR_TRK_COAST then
                skrOk = vLen(vSub(skrCand, vAdd(skrTrk, vScale(tgtVel, skrTrkAge + 1))))
                            < min(SKR_TRK_GATE * (skrTrkAge + 1), SKR_TRK_MAX)
                        and (coast > SKR_DL_FRESH or dlAgree)
            else
                skrOk = coast <= SKR_DL_FRESH and dlAgree
            end
        end
        if skrOk then
            skrTrk, skrTrkAge = skrCand, 0
        elseif skrTrk then
            skrTrkAge = skrTrkAge + 1
            if skrTrkAge > SKR_TRK_COAST then skrTrk = nil end
        end
        skrN = skrOk and skrN + 1 or 0

        relPos = dlRel
        if skrN >= SKR_ON then
            -- the seeker takes the aim point inside SKR_AIM_MAX
            skrRel = vSub(skrCand, ownPos)
            skrR   = vLen(skrRel)

            if skrR <= SKR_AIM_MAX then
                relPos, tgtPos, skrRng = skrRel, skrCand, skrR
            end
        end

        tgtRangeNow = vLen(relPos)
        if tgtRangeNow < missMin then missMin = tgtRangeNow end
        if missMin < CUT_NEAR and tgtRangeNow > missMin + CUT_MARGIN then
            postMiss = true
        end

        if DLPN_W > 0 and dlpnN > DLPN_W then
            dlpnVel = vDiv(vSub(dlpnHist[DLPN_W + 1], dlpnHist[1]), DLPN_W)
        end
        lead = calcTang(relPos, dlpnVel or tgtVel, misVel)
        if lead then
            guideDir = vNorm(lead)
            refuseN  = 0
        end
    end

    if postMiss then
        writeFins(0, 0)
        clearCommandDebug()
        return
    end

    -- calcTang refusal: coast briefly (fins level) above the altitude floor,
    -- then fall back to pure pursuit
    if guideDir == nil and authority >= 1 then
        refuseN = refuseN + 1
        if refuseN <= REFUSE_MAX and ownPos.z > COAST_FLOOR then
            writeFins(0, 0)
            clearCommandDebug()
            return
        end
        if tgtRangeNow > 1e-6 then guideDir = vNorm(relPos) end
    end
    if guideDir == nil and authority >= 1 then
        writeFins(0, 0)
        clearCommandDebug()
        return
    end

    -- blend guidance with the flight stabiliser by coast authority
    stabDir = stabDirection()
    if guideDir == nil then
        if stabHold == nil and stabDir ~= nil and ownSpd > STAB_LATCH_MIN then stabHold = stabDir end
        cmdDir = stabHold or stabDir
    elseif authority >= 1 or stabDir == nil then
        cmdDir = guideDir
    else
        cmdDir = vAdd(vScale(guideDir, authority), vScale(stabDir, 1 - authority))
        cmdDir = vLen(cmdDir) > 0.05 and vNorm(cmdDir) or stabDir
    end
    if cmdDir == nil then
        writeFins(0, 0)
        clearCommandDebug()
        return
    end

    -- world command direction to body-frame yaw/pitch angles
    cmdRight = vDot(cmdDir, rightAxis)
    cmdFwd   = vDot(cmdDir, fwdAxis)
    cmdDown  = -vDot(cmdDir, upAxis)
    cmdYaw   = atan(cmdRight, cmdFwd)
    cmdPitch = atan(cmdDown, sqrt(cmdRight ^ 2 + cmdFwd ^ 2))
    cyAbs = abs(cmdYaw)
    asternHold = cyAbs > CMD_REVERSE_HOLD
    if asternHold then
        -- command far astern: hold the established turn direction and keep
        -- pitch referenced to the horizontal projection so it stays sane
        cmdYawUse = yawSide * softAngle(cyAbs)
        flatDir = Vec(cmdDir.x, cmdDir.y, 0)
        if vLen(flatDir) > 1e-6 then
            flatDir  = vNorm(flatDir)
            flatR    = vDot(flatDir, rightAxis)
            flatF    = vDot(flatDir, fwdAxis)
            cmdPitch = atan(-vDot(flatDir, upAxis), sqrt(flatR ^ 2 + flatF ^ 2))
        end
    else
        cmdYawUse = softAngle(cmdYaw)
        yawSide = cmdYaw >= 0 and 1 or -1
    end

    -- terminal boost while the seeker drives the aim point
    navUse = navScale
    if skrRng > 0 and TERM_SK1 > 0 then
        navUse = min(navScale * (1 + TERM_SK1 * clamp((TERM_R - tgtRangeNow) / TERM_RW, 0, 1)),
                     NAV_AUTH_MIN)
    end
    yawFin = MAIN_FIN_YAW_SIGN * yawPID:run(0, removeNaN(cmdYawUse), navUse)
    pitchFin = MAIN_FIN_PITCH_SIGN
               * pitchPID:run(0, removeNaN(cmdPitch), navUse)
    writeFins(yawFin, pitchFin)
    ON(4, skrRng)

    ON(8, cmdDir.x); ON(9, cmdDir.y); ON(10, cmdDir.z)
    ON(21, BUILD)
    ONB(1, skrRej)
    ON(32, navUse)

end

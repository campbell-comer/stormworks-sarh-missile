-- nr-tracker.lua -- radar target tracker: 9-state EKF + fixed-lag smoother
--
-- Fuses contacts from eight fixed radars (via one nr-radar-fe.lua each) into
-- a single track, and publishes a smoothed, delay-compensated position /
-- velocity / acceleration solution for the datalink.
--
-- Architecture, per tick:
--   1. Per-radar contacts are associated to the track and batched into short
--      BURSTS in a range/azimuth/elevation frame anchored at the burst's
--      first sample. A robust midhull (median-anchored trimmed midrange)
--      condenses each burst before it enters the filter, which suppresses the
--      radar's uniform quantisation noise far better than averaging.
--   2. A 9-state EKF (position/velocity/acceleration per axis) runs on the
--      condensed polar measurements. Process noise follows a Singer model:
--      acceleration decays with time constant SG_TAU and the injected noise
--      level adapts between Q_MIN and Q_MAX on normalised innovation (NIS),
--      so the filter is stiff on straight targets and compliant in turns.
--   3. A fixed-lag RTS smoother re-runs the last L_SMOOTH ticks (L_MANV while
--      manoeuvring) so the published state benefits from hindsight.
--   4. The output is propagated forward by the smoother lag plus the known
--      composite bus delay, so the datalink carries an estimate of the target
--      NOW, not as it was when measured.
--   5. A coordinated-turn detector estimates turn rate from the velocity
--      history and publishes the implied centripetal acceleration.
--
-- NUMBER INPUTS: 1..24 contact x/y/z per radar (3 each), 25-27 mount position
-- in raw physics-sensor order (x east / y up / z north, remapped in-script),
-- 28-30 coprocessor seed position.
-- BOOL INPUTS: 1..8 contact valid per radar, 9..16 fresh-sample flags,
-- 17 lock active.
-- NUMBER OUTPUTS: 1-3 position, 4-6 velocity (m/s), 7-9 acceleration,
-- 10 worst per-radar bias (radar-sign diagnostic), 11 contributing radar
-- count, 12-17 velocity/position mirror for the display bus.

getN = input.getNumber
getB = input.getBool
setN = output.setNumber
setB = output.setBool
m = math
sqrtf, absf, cosine, floorf, sinef = m.sqrt, m.abs, m.cos, m.floor, m.sin
atan2, asinf, expf = m.atan, m.asin, m.exp
minf, maxf = m.min, m.max
TAU = m.pi * 2

L_SMOOTH    = 45          -- smoother lag (ticks) on quiet tracks
L_MANV      = 8           -- shortened lag while the adaptive Q is elevated

AGE_FE      = 1           -- known pipeline delays: front-end, write chain,
AGE_CHAIN   = 2           -- and the output hop; the solution is propagated
AGE_OUT     = 1           -- forward by smoother lag + BUS_DELAY
BUS_DELAY   = AGE_FE + AGE_CHAIN + AGE_OUT

Q_MIN       = 1e-6        -- adaptive process-noise bounds
Q_MAX       = 1e-2

NIS_TRIG    = 0.25        -- normalised innovation level that opens Q

Q_UP        = 1.0         -- Q slew rates toward Q_MAX / Q_MIN
Q_DN        = 0.02

SG_TAU      = 14          -- Singer acceleration time constant (ticks)

SG_A        = 1 / SG_TAU
SG_ED       = m.exp(-SG_A)
SG_VA       = (1 - SG_ED) / SG_A
SG_PA       = (-1 + SG_A + SG_ED) / (SG_A * SG_A)
ROBUST_GATE = 16          -- Mahalanobis d2 cap: outliers are de-weighted,
                          -- not discarded

AGE_K       = 3.0         -- measurement-noise inflation per tick of sample age

ASSOC_GATE  = 120         -- m association gate, contact to track

STARVE_MAX  = 12          -- ticks with no contributing radar before reinit

NRAD        = 8

LOCK_TRUST  = 0.25        -- velocity publishes once covariance drops below
LOCK_CAP    = 60          -- this fraction of its seed, or after LOCK_CAP ticks,
LOCK_RAMP   = 6           -- ramped in over LOCK_RAMP ticks

-- Coordinated-turn detection from the horizontal velocity history.
W_WIN       = 45          -- ticks between the two velocity samples compared
W_EMA       = 0.06
CT_PROP     = false       -- optionally propagate the output around the arc

W_FLOOR     = 0.04        -- rad/s below which turn rate is treated as zero
W_ARC_CAP   = 3

W_LEAD_S    = 2.0
W_PROP_MIN  = 0.30
W_PROP_MAX  = 3.5

GATE_VMIN   = 15          -- m/s: no turn estimation below this speed
MAX_BURST   = 8           -- samples per burst
BURST_MAX_SPAN = 3        -- ticks a burst may span before being flushed

T, lockAge = 0, 0
ekfOn = false
gw, gpersist = 0, 0
sgPrev, sgT = false, 0

starve, recov = 0, false

-- EKF state: EX[1..9] = x,vx,ax, y,vy,ay, z,vz,az; EP = 9x9 covariance,
-- row-major in a flat array
EX, EP = {}, {}
ephi, enis = Q_MIN, 0
HH, PHT, KK, HP, SS, SI = {}, {}, {}, {}, {}, {}
Q_JERK = { 0.05, 0.125, 1 / 6, 0.125, 1 / 3, 0.5, 1 / 6, 0.5, 1 }
P_SEED = { 400, 25, 1e-4 }

-- smoother ring buffers: predicted state, filtered state, smoother gains
RN = L_SMOOTH + 2
SXP, SXF, SCG = {}, {}, {}
XS, DS, NS, prevPF = {}, {}, {}, {}
sIdx, prevPFok = 0, false

-- per-radar burst state
bn, baz0, bOX, bOY, bOZ, bt0 = {}, {}, {}, {}, {}, {}
bR, bA, bE = {}, {}, {}
rfresh, pick, pickD, rbias = {}, {}, {}, {}
pkx, pky, pkz = {}, {}, {}
for b = 1, NRAD do
    bn[b], baz0[b], bt0[b] = 0, 0, 0
    bOX[b], bOY[b], bOZ[b] = 0, 0, 0
    bR[b], bA[b], bE[b] = {}, {}, {}
    rfresh[b], pick[b], pickD[b], rbias[b] = false, 0, 0, 0
end

cx, cy, cz, crad = {}, {}, {}, {}
VRN = W_WIN + 4
vhx, vhy = {}, {}
for i = 0, VRN do vhx[i], vhy[i] = 0, 0 end

function clamp(x, lo, hi) return x < lo and lo or (x > hi and hi or x) end
function wrapRad(a) return a - floorf(a / TAU + 0.5) * TAU end
function dist3(ax, ay, az, bx, by, bz) return sqrtf((ax - bx) ^ 2 + (ay - by) ^ 2 + (az - bz) ^ 2) end

-- copy the 3x3 covariance block for one axis out of EP
function blk3(eb, dst, off)
    for r = 0, 2 do
        for c = 1, 3 do dst[off + r * 3 + c] = EP[(eb + r) * 9 + eb + c] end
    end
end

-- adjugate-based 3x3 inverse; returns the determinant, leaves adj in I
function inv3(S, I)
    for i = 0, 2 do
        for j = 0, 2 do
            j1, j2, i1, i2 = (j + 1) % 3, (j + 2) % 3, (i + 1) % 3, (i + 2) % 3
            I[i * 3 + j + 1] = S[j1 * 3 + i1 + 1] * S[j2 * 3 + i2 + 1] - S[j1 * 3 + i2 + 1] * S[j2 * 3 + i1 + 1]
        end
    end
    return S[1] * I[1] + S[2] * I[4] + S[3] * I[7]
end

-- Robust midhull: insertion-sort the burst, take the median, then the
-- midrange of samples within 1.5 half-widths of it. Returns the estimate and
-- the surviving sample count.
function robustMid(buf, n, h)
    for i = 2, n do
        rmV, rmJ = buf[i], i - 1
        while rmJ >= 1 and buf[rmJ] > rmV do buf[rmJ + 1] = buf[rmJ]; rmJ = rmJ - 1 end
        buf[rmJ + 1] = rmV
    end
    rmMed = (buf[floorf((n + 1) / 2)] + buf[floorf(n / 2) + 1]) / 2
    rmLo, rmHi, rmK = 0, 0, 0
    for i = 1, n do
        if absf(buf[i] - rmMed) <= 1.5 * h then
            if rmK == 0 then rmLo, rmHi = buf[i], buf[i] end
            if buf[i] < rmLo then rmLo = buf[i] end
            if buf[i] > rmHi then rmHi = buf[i] end
            rmK = rmK + 1
        end
    end
    if rmK < 1 then return rmMed, 1 end
    return (rmLo + rmHi) / 2, rmK
end

function ekfInit(px, py, pz)
    for i = 1, 9 do EX[i] = 0 end
    EX[1], EX[4], EX[7] = px, py, pz

    for i = 1, 81 do EP[i] = 0 end
    for a = 0, 2 do
        for r = 1, 3 do EP[(3 * a + r - 1) * 10 + 1] = P_SEED[r] end
    end

    ephi, enis, ekfOn = Q_MIN, 0, true
    sgOn, sPA, sVA, sED = false, 0.5, 1, 1
    sIdx, prevPFok = 0, false
    lockAge, gw, gpersist, vTrust = 0, 0, 0, nil
    for b = 1, NRAD do bn[b] = 0 end
end

function ekfPredict()
    -- adapt process noise on NIS, then pick the transition constants: plain
    -- constant-acceleration while quiet, Singer decay while manoeuvring
    ephi = ephi + (((enis > NIS_TRIG) and Q_UP or Q_DN)
                   * (((enis > NIS_TRIG) and Q_MAX or Q_MIN) - ephi))

    sgOn = ephi > Q_MIN * 10
    if sgOn then sPA, sVA, sED = SG_PA, SG_VA, SG_ED else sPA, sVA, sED = 0.5, 1, 1 end

    for a = 0, 2 do
        eb = 3 * a
        EX[eb + 1] = EX[eb + 1] + EX[eb + 2] + sPA * EX[eb + 3]
        EX[eb + 2] = EX[eb + 2] + sVA * EX[eb + 3]
        EX[eb + 3] = sED * EX[eb + 3]
    end

    -- P <- F P F' exploiting the block-diagonal F (row pass then column pass)
    for a = 0, 2 do
        r1, r2, r3 = 3 * a * 9, (3 * a + 1) * 9, (3 * a + 2) * 9
        for j = 1, 9 do
            EP[r1 + j] = EP[r1 + j] + EP[r2 + j] + sPA * EP[r3 + j]
            EP[r2 + j] = EP[r2 + j] + sVA * EP[r3 + j]
            EP[r3 + j] = sED * EP[r3 + j]
        end
    end
    for a = 0, 2 do
        c1, c2, c3 = 3 * a + 1, 3 * a + 2, 3 * a + 3
        for i = 0, 8 do
            EP[i * 9 + c1] = EP[i * 9 + c1] + EP[i * 9 + c2] + sPA * EP[i * 9 + c3]
            EP[i * 9 + c2] = EP[i * 9 + c2] + sVA * EP[i * 9 + c3]
            EP[i * 9 + c3] = sED * EP[i * 9 + c3]
        end
    end

    for a = 0, 2 do
        eb = 3 * a
        for r = 0, 2 do
            for c = 1, 3 do
                ei = (eb + r) * 9 + eb + c
                EP[ei] = EP[ei] + ephi * Q_JERK[r * 3 + c]
            end
        end
    end
end

-- One polar measurement update. nAge is the sample age in ticks (the state is
-- projected BACK by nAge for the residual, via the k2/k3 terms folded into
-- the Jacobian columns); nMid is the surviving midhull count, which sets how
-- much the quantisation-noise variance was reduced.
function ekfUpdate(zr, za, ze, ox, oy, oz, nAge, nMid)

    k2 = -nAge
    if sgOn then
        k3e = expf(SG_A * nAge)
        k3 = (-1 - SG_A * nAge + k3e) / (SG_A * SG_A)
    else
        k3 = 0.5 * nAge * nAge
    end
    bx = EX[1] + k2 * EX[2] + k3 * EX[3] - ox
    by = EX[4] + k2 * EX[5] + k3 * EX[6] - oy
    bz = EX[7] + k2 * EX[8] + k3 * EX[9] - oz
    rr = sqrtf(bx * bx + by * by + bz * bz); if rr < 1e-6 then rr = 1e-6 end
    rxy = sqrtf(bx * bx + by * by); if rxy < 1e-6 then rxy = 1e-6 end

    HH[1],  HH[4],  HH[7]  = bx / rr, by / rr, bz / rr
    HH[10], HH[13], HH[16] = by / (rxy * rxy), -bx / (rxy * rxy), 0
    HH[19], HH[22], HH[25] = -bx * bz / (rr * rr * rxy), -by * bz / (rr * rr * rxy), rxy / (rr * rr)
    for j = 1, 9 do
        HH[j * 3 - 1], HH[j * 3] = HH[j * 3 - 2] * k2, HH[j * 3 - 2] * k3
    end

    y1 = zr - rr
    y2 = wrapRad(za - atan2(bx, by))
    y3 = ze - asinf(clamp(bz / rr, -1, 1))

    -- Measurement noise: the radar's error is uniform, radial 1% of range and
    -- angular 0.001 turns, hence the /3 (variance of a uniform half-width).
    -- 6/((n+1)(n+2)) is the variance reduction of the midhull of n samples,
    -- and age inflates the noise since the target moved under the burst.
    rInfl = (6 / ((nMid + 1) * (nMid + 2))) * (1 + AGE_K * nAge)
    cel = absf(cosine(ze)); if cel < 0.1 then cel = 0.1 end
    Rr = ((0.01 * zr) ^ 2 / 3) * rInfl
    Rz = ((0.001 * TAU) ^ 2 / 3) * rInfl
    Ra = Rz / (cel * cel)

    for i = 0, 8 do
        for c = 1, 3 do
            hb, acc = (c - 1) * 9, 0
            for j = 1, 9 do acc = acc + EP[i * 9 + j] * HH[hb + j] end
            PHT[i * 3 + c] = acc
        end
    end

    for r = 0, 2 do
        for c = 1, 3 do
            acc = 0
            for j = 1, 9 do acc = acc + HH[r * 9 + j] * PHT[(j - 1) * 3 + c] end
            SS[r * 3 + c] = acc
        end
    end
    SS[1], SS[5], SS[9] = SS[1] + Rr, SS[5] + Ra, SS[9] + Rz

    det = inv3(SS, SI)
    if absf(det) < 1e-30 then return end
    d2 = (y1 * (SI[1] * y1 + SI[2] * y2 + SI[3] * y3)
        + y2 * (SI[4] * y1 + SI[5] * y2 + SI[6] * y3)
        + y3 * (SI[7] * y1 + SI[8] * y2 + SI[9] * y3)) / det

    -- robust step: instead of discarding an outlier, scale S up so the
    -- update shrinks smoothly with distance
    if d2 > ROBUST_GATE then
        det = det * d2 / ROBUST_GATE
        d2 = ROBUST_GATE
    end
    for k = 1, 9 do SI[k] = SI[k] / det end
    enis = 0.7 * enis + 0.3 * d2 / 3

    for i = 0, 8 do
        for c = 1, 3 do
            KK[i * 3 + c] = PHT[i * 3 + 1] * SI[c] + PHT[i * 3 + 2] * SI[3 + c] + PHT[i * 3 + 3] * SI[6 + c]
        end
    end
    for i = 1, 9 do
        EX[i] = EX[i] + KK[(i - 1) * 3 + 1] * y1 + KK[(i - 1) * 3 + 2] * y2 + KK[(i - 1) * 3 + 3] * y3
    end
    for r = 0, 2 do
        hb = r * 9
        for j = 1, 9 do
            acc = 0
            for q = 1, 9 do acc = acc + HH[hb + q] * EP[(q - 1) * 9 + j] end
            HP[hb + j] = acc
        end
    end

    for i = 0, 8 do
        for j = 1, 9 do
            EP[i * 9 + j] = EP[i * 9 + j] - KK[i * 3 + 1] * HP[j] - KK[i * 3 + 2] * HP[9 + j] - KK[i * 3 + 3] * HP[18 + j]
        end
    end
end

-- Fixed-lag smoother bookkeeping. Push the predicted state each tick, the
-- filtered state after updates, and the per-axis RTS gain G = P_f F' (P_p)^-1
-- computed from the PREVIOUS filtered covariance.
function smootherPushPredict()
    sIdx = sIdx + 1
    sl = (sIdx % RN) * 9
    for i = 1, 9 do SXP[sl + i] = EX[i] end
    if prevPFok then
        cg = ((sIdx - 1) % RN) * 27
        for a = 0, 2 do
            eb, pb = 3 * a, a * 9
            blk3(eb, SS, 0)
            dt3 = inv3(SS, SI)
            if absf(dt3) < 1e-30 then
                for z = 1, 9 do SCG[cg + pb + z] = 0 end
            else
                for k = 1, 9 do SI[k] = SI[k] / dt3 end
                for r = 1, 3 do
                    pr = pb + (r - 1) * 3

                    g1 = prevPF[pr + 1] + prevPF[pr + 2] + sPA * prevPF[pr + 3]
                    g2 = prevPF[pr + 2] + sVA * prevPF[pr + 3]
                    g3 = sED * prevPF[pr + 3]
                    for c = 1, 3 do
                        SCG[cg + pr + c] = g1 * SI[c] + g2 * SI[3 + c] + g3 * SI[6 + c]
                    end
                end
            end
        end
    end
end

function smootherPushFilter()
    sl = (sIdx % RN) * 9
    for i = 1, 9 do SXF[sl + i] = EX[i] end
    for a = 0, 2 do blk3(3 * a, prevPF, a * 9) end
    prevPFok = true
end

-- Run the RTS recursion backward over the lag window; returns the number of
-- steps actually smoothed.
function smootherRun()
    sl = (sIdx % RN) * 9
    for i = 1, 9 do XS[i] = SXF[sl + i] end

    steps = minf((ephi > Q_MIN * 10) and L_MANV or L_SMOOTH, sIdx - 1)
    for kk = sIdx - 1, sIdx - steps, -1 do
        ks, kp, cg = (kk % RN) * 9, ((kk + 1) % RN) * 9, (kk % RN) * 27
        for i = 1, 9 do DS[i] = XS[i] - SXP[kp + i] end
        for a = 0, 2 do
            eb, pb = 3 * a, a * 9
            for r = 1, 3 do
                acc = SXF[ks + eb + r]
                for c = 1, 3 do acc = acc + SCG[cg + pb + (r - 1) * 3 + c] * DS[eb + c] end
                NS[eb + r] = acc
            end
        end
        for i = 1, 9 do XS[i] = NS[i] end
    end
    return steps
end

-- Condense a finished burst with the midhull and feed it to the EKF. The
-- half-widths passed to robustMid are the radar's quantisation widths.
function flushBurst(b)
    hA = 0.001 * TAU
    ce = maxf(absf(cosine(bE[b][1])), 0.1)
    eR, kR = robustMid(bR[b], bn[b], 0.01 * bR[b][1])
    eA, kA = robustMid(bA[b], bn[b], hA / ce)
    eE, kE = robustMid(bE[b], bn[b], hA)
    ekfUpdate(eR, baz0[b] + eA, eE, bOX[b], bOY[b], bOZ[b], bn[b], minf(kR, kA, kE))
    bn[b] = 0
end

function onTick()
    T = T + 1
    lockActive = getB(NRAD * 2 + 1)
    setB(13, lockActive)

    mountX, mountY, mountZ = getN(25), getN(27), getN(26)

    cn = 0

    for b = 1, NRAD do
        rfresh[b] = getB(NRAD + b)
        if getB(b) then
            cb = (b - 1) * 3
            cn = cn + 1
            cx[cn], cy[cn], cz[cn], crad[cn] = getN(cb + 1), getN(cb + 2), getN(cb + 3), b
        end
    end

    if lockActive then
        -- starvation watchdog: no radar contributing for STARVE_MAX ticks
        -- drops the filter so it can re-seed from the coprocessor
        if ekfOn and contributing == 0 then starve = starve + 1 else starve = 0 end
        if starve >= STARVE_MAX then ekfOn, starve, recov = false, 0, true end

        if not ekfOn then
            sx, sy, sz = getN(28), getN(29), getN(30)
            for i = 1, cn do

                if not ekfOn and dist3(cx[i], cy[i], cz[i], sx, sy, sz) <= ASSOC_GATE then
                    ekfInit(sx, sy, sz)
                    -- a re-seed after starvation keeps velocity trust
                    if recov then lockAge, recov = LOCK_CAP, false end
                end
            end
        end
    else
        ekfOn, starve, recov = false, 0, false
    end

    disagree, contributing = 0, 0
    if ekfOn then
        lockAge = lockAge + 1
        ekfPredict()
        smootherPushPredict()

        -- nearest in-gate contact per radar
        for b = 1, NRAD do pick[b], pickD[b] = 0, 1e9 end
        for i = 1, cn do
            d = dist3(cx[i], cy[i], cz[i], EX[1], EX[4], EX[7])
            if d <= ASSOC_GATE and d < pickD[crad[i]] then pick[crad[i]], pickD[crad[i]] = i, d end
        end
        for b = 1, NRAD do
            i = pick[b]
            if i > 0 then
                sx, sy, sz, sn = cx[i], cy[i], cz[i], 1
                pkx[b], pky[b], pkz[b] = sx / sn, sy / sn, sz / sn
            end
        end

        ex1, ex4 = EX[1], EX[4]
        losx, losy = ex1 - mountX, ex4 - mountY
        lh = maxf(sqrtf(losx * losx + losy * losy), 1)

        for b = 1, NRAD do
            i = pick[b]
            if i > 0 then
                contributing = contributing + 1

                lx, ly, lz = pkx[b], pky[b], pkz[b]

                -- slow EMA of each radar's cross-LOS offset; a persistent
                -- offset means a mis-signed or mis-placed radar on the rig
                rbias[b] = 0.99 * rbias[b] + 0.01 * ((ly - ex4) * losx - (lx - ex1) * losy) / lh

                if bn[b] > 0 and T - bt0[b] > BURST_MAX_SPAN then bn[b] = 0 end

                if rfresh[b] and bn[b] > 0 then flushBurst(b) end
                if bn[b] == 0 then
                    bOX[b], bOY[b], bOZ[b] = mountX, mountY, mountZ
                    baz0[b] = atan2(lx - mountX, ly - mountY)
                    bt0[b] = T
                end
                dx, dy, dz = lx - bOX[b], ly - bOY[b], lz - bOZ[b]
                sr = sqrtf(dx * dx + dy * dy + dz * dz)
                if sr > 1 and bn[b] < MAX_BURST then

                    bn[b] = bn[b] + 1
                    bR[b][bn[b]] = sr
                    bA[b][bn[b]] = wrapRad(atan2(dx, dy) - baz0[b])
                    bE[b][bn[b]] = asinf(clamp(dz / sr, -1, 1))
                end
            end

            if absf(rbias[b]) > disagree then disagree = absf(rbias[b]) end
        end

        smootherPushFilter()
        steps = smootherRun()

        -- propagate the smoothed state forward over the lag + bus delay
        np = steps + BUS_DELAY

        if sgOn then
            opE = expf(-SG_A * np)
            opV = (1 - opE) / SG_A
            opP = (-1 + SG_A * np + opE) / (SG_A * SG_A)
        else
            opV, opP = np, 0.5 * np * np
        end
        outX = XS[1] + XS[2] * np + opP * XS[3]
        outY = XS[4] + XS[5] * np + opP * XS[6]
        outZ = XS[7] + XS[8] * np + opP * XS[9]
        outVX, outVY, outVZ = (XS[2] + opV * XS[3]) * 60, (XS[5] + opV * XS[6]) * 60, (XS[8] + opV * XS[9]) * 60

        -- coordinated-turn rate from velocity headings W_WIN ticks apart,
        -- EMA-smoothed, published only while its sign persists
        svx, svy = XS[2] * 60, XS[5] * 60
        vhx[T % VRN], vhy[T % VRN] = svx, svy

        if sgOn ~= sgPrev then sgT = T end
        sgPrev = sgOn
        if (svx * svx + svy * svy) > GATE_VMIN * GATE_VMIN and T > W_WIN and T - sgT > W_WIN then
            ovx, ovy = vhx[(T - W_WIN) % VRN], vhy[(T - W_WIN) % VRN]
            if (ovx * ovx + ovy * ovy) > 1 then
                wRaw = atan2(ovx * svy - ovy * svx, ovx * svx + ovy * svy) / (W_WIN / 60)
                gw = (1 - W_EMA) * gw + W_EMA * wRaw
                if (wRaw >= 0) == (gw >= 0) then gpersist = minf(1, gpersist + 0.05)
                else gpersist = maxf(0, gpersist - 0.15) end
            end
        else
            gw, gpersist = 0, 0
        end

        wGate = gw * clamp(absf(gw) / W_FLOOR - 1, 0, 1) * gpersist
        wUse = clamp(wGate, -W_ARC_CAP / W_LEAD_S, W_ARC_CAP / W_LEAD_S)
        wProp = clamp(wGate, -W_PROP_MAX, W_PROP_MAX)

        if sgOn and CT_PROP and absf(wProp) > W_PROP_MIN then
            -- rotate the propagation around the turn arc instead of extrapolating straight
            cth = wProp * np / 60
            ccos, csin = cosine(cth), sinef(cth)
            outVX, outVY = svx * ccos - svy * csin, svx * csin + svy * ccos
            ca1, ca2 = csin / wProp, (1 - ccos) / wProp
            outX = XS[1] + ca1 * svx - ca2 * svy
            outY = XS[4] + ca2 * svx + ca1 * svy
        end

        outAX, outAY = -wUse * outVY, wUse * outVX

        -- velocity trust: publish once the velocity covariance has converged
        -- (or after LOCK_CAP ticks), ramped in over LOCK_RAMP
        if not vTrust and (lockAge >= LOCK_CAP
            or EP[11] + EP[41] + EP[71] < LOCK_TRUST * 3 * P_SEED[2]) then vTrust = lockAge end
        sw = vTrust and clamp((lockAge - vTrust) / LOCK_RAMP, 0, 1) or 0
        ov1, ov2, ov3 = outVX * sw, outVY * sw, outVZ * sw
        setN(1, outX); setN(2, outY); setN(3, outZ)
        setN(4, ov1); setN(5, ov2); setN(6, ov3)
        setN(7, outAX * sw); setN(8, outAY * sw); setN(9, 0)

        setN(12, ov1); setN(13, ov2); setN(14, ov3)
        setN(15, outX); setN(16, outY); setN(17, outZ)
    else
        for i = 1, 9 do setN(i, 0) end
        for i = 12, 17 do setN(i, 0) end
        lockAge, gw, gpersist = 0, 0, 0
    end
    setN(10, disagree)
    setN(11, contributing)
end

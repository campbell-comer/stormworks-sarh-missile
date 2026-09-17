-- nr-scan-stagger.lua -- staggered radar power-up
--
-- Brings the eight radars up one tick apart so their fixed scan phases are
-- spread across the tick cycle instead of sampling the target simultaneously.
-- Bool input 1 is master power; bool outputs 1..8 gate each radar.

ticks = 0

function onTick()
    power = input.getBool(1)

    if power then
        if ticks < 12 then ticks = ticks + 1 end
    else
        ticks = 0
    end

    for i = 1, 8 do output.setBool(i, power and ticks > i - 1) end
end

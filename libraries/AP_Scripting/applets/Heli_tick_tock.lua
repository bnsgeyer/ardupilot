--[[
   Heli tick-tock for traditional helicopters.

   The rotor disc is rocked back and forth through vertical about the pitch
   (or roll) axis while the collective reverses on every swing, so that the
   vertical part of the rotor thrust always points up and the heli bounces in
   place like a pendulum.

   Raise the aux switch while flying: the script switches to Guided, swings
   the heli up into the tick-tock, rocks it for TTOC_CYCLES cycles (or while
   the switch stays high), brings it back upright and returns to the mode it
   was flown in. Roll, pitch or yaw stick input hands control straight back
   to that mode.

   Needs a 3D collective setup: H_COL_ANG_MIN must give about as much negative
   thrust as H_COL_ANG_MAX gives positive thrust.
--]]

local PARAM_TABLE_KEY = 146
local PARAM_TABLE_PREFIX = "TTOC_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local MODE_GUIDED = 4
local SPOOL_THROTTLE_UNLIMITED = 3
local GRAVITY_MSS = 9.80665
local UPDATE_MS = 10
local SWITCH_HIGH = 2

-- Guided timeout while the manoeuvre runs. Rates are sent every 10ms, and if
-- the script stops the last rate is flown until this expires.
local MANOEUVRE_GUID_TIMEOUT_S = 0.3
-- tracking error must last this long before recovery starts, well inside the
-- 2 seconds the crash check needs before it disarms
local ABORT_TIME_S = 0.5
-- recovery hands back once the disc is this close to level, or after the timeout
local RECOVER_LEVEL_DEG = 10
local RECOVER_RATE_DEGS = 20
local RECOVER_TIMEOUT_S = 3
-- mean vertical thrust the swing must be able to make, as a multiple of
-- weight, leaving room for the altitude correction
local MIN_SUPPORT_RATIO = 1.1
--[[
   Thrust the entry and exit may use, as a multiple of hover thrust. Holding
   height as the disc passes vertical would take the collective to full one
   way and then the other, and that torque swing yaws the heli, which the
   rest of the entry turns into roll. The crossing is brief, so it costs
   little height to let it sink instead.
--]]
local JOIN_THRUST_MAX = 2
--[[
   Share of the planned thrust the rotor is taken to deliver until the swing
   has measured it. A rotor moving along its own axis loses thrust, by a third
   in RealFlight on a 2 second swing, so start pessimistic: the first swing
   then gets full collective rather than winding up to it.
--]]
local EFFICIENCY_START = 0.5
local EFFICIENCY_MIN = 0.3
local EFFICIENCY_MAX = 1.5
-- fraction of ATC_ACC_P_MAX or ATC_ACC_R_MAX the planned swing may use
local ACCEL_MARGIN = 0.9
-- AP_MotorsHeli limits on H_COL_HOVER
local COL_HOVER_MIN = 0.3
local COL_HOVER_MAX = 0.8

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 17), 'could not add param table')

-- bind a parameter to a variable
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: TTOC_ENABLE
  // @DisplayName: Tick-tock enable
  // @Description: Enables the heli tick-tock applet
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local TTOC_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: TTOC_AUX_FUNC
  // @DisplayName: Tick-tock aux function
  // @Description: RCx_OPTION scripting function that starts the tick-tock when switched high
  // @Values: 300:Scripting1,301:Scripting2,302:Scripting3,303:Scripting4,304:Scripting5,305:Scripting6,306:Scripting7,307:Scripting8
  // @User: Standard
--]]
local TTOC_AUX_FUNC = bind_add_param('AUX_FUNC', 2, 300)

--[[
  // @Param: TTOC_AXIS
  // @DisplayName: Tick-tock axis
  // @Description: Axis the disc is rocked about. Pitch rocks the heli nose over tail with elevator, roll rocks it side to side with aileron.
  // @Values: 0:Pitch,1:Roll
  // @User: Standard
--]]
local TTOC_AXIS = bind_add_param('AXIS', 3, 0)

--[[
  // @Param: TTOC_DIR
  // @DisplayName: Tick-tock direction
  // @Description: Which way the heli is turned to stand the disc vertical. For pitch, 1 is nose up (tail down) and -1 is nose down. For roll, 1 is right side down and -1 is left side down.
  // @Values: -1:Nose down or left side down,1:Nose up or right side down
  // @User: Standard
--]]
local TTOC_DIR = bind_add_param('DIR', 4, -1)

--[[
  // @Param: TTOC_ANGLE
  // @DisplayName: Tick-tock swing angle
  // @Description: How far the disc swings either side of vertical. Larger angles spend less time with the disc near vertical, so need less collective to hold height.
  // @Units: deg
  // @Range: 20 80
  // @User: Standard
--]]
local TTOC_ANGLE = bind_add_param('ANGLE', 5, 50)

--[[
  // @Param: TTOC_PERIOD
  // @DisplayName: Tick-tock period
  // @Description: Time for one full tick and tock. The swing's peak angular acceleration must fit within ATC_ACC_P_MAX (or ATC_ACC_R_MAX for roll).
  // @Units: s
  // @Range: 0.3 5
  // @User: Standard
--]]
local TTOC_PERIOD = bind_add_param('PERIOD', 6, 2.0)

--[[
  // @Param: TTOC_CYCLES
  // @DisplayName: Tick-tock cycles
  // @Description: Number of tick-tock cycles to fly. 0 keeps going while the switch is held high.
  // @Range: 0 50
  // @User: Standard
--]]
local TTOC_CYCLES = bind_add_param('CYCLES', 7, 3)

--[[
  // @Param: TTOC_REV_ANG
  // @DisplayName: Tick-tock collective reversal angle
  // @Description: The collective reverses smoothly while the disc is within this angle of vertical, instead of jumping from full positive to full negative.
  // @Units: deg
  // @Range: 1 20
  // @User: Advanced
--]]
local TTOC_REV_ANG = bind_add_param('REV_ANG', 8, 5)

--[[
  // @Param: TTOC_ATT_P
  // @DisplayName: Tick-tock attitude gain
  // @Description: Rate added per radian of error between the planned and actual attitude
  // @Units: 1/s
  // @Range: 0 10
  // @User: Advanced
--]]
local TTOC_ATT_P = bind_add_param('ATT_P', 9, 3)

--[[
  // @Param: TTOC_POS_P
  // @DisplayName: Tick-tock position gain
  // @Description: Velocity asked for per metre of position error, using the position averaged over one tick-tock period
  // @Units: 1/s
  // @Range: 0 2
  // @User: Advanced
--]]
local TTOC_POS_P = bind_add_param('POS_P', 10, 0.3)

--[[
  // @Param: TTOC_VEL_P
  // @DisplayName: Tick-tock velocity gain
  // @Description: Acceleration asked for per m/s of velocity error, using the velocity averaged over one tick-tock period
  // @Units: 1/s
  // @Range: 0 3
  // @User: Advanced
--]]
local TTOC_VEL_P = bind_add_param('VEL_P', 11, 0.6)

--[[
  // @Param: TTOC_VEL_I
  // @DisplayName: Tick-tock velocity integrator gain
  // @Description: Integrator on the averaged velocity error, for wind and for collective needed to hold height
  // @Range: 0 1
  // @User: Advanced
--]]
local TTOC_VEL_I = bind_add_param('VEL_I', 12, 0.3)

--[[
  // @Param: TTOC_ACC_MAX
  // @DisplayName: Tick-tock correction acceleration limit
  // @Description: Largest acceleration each of the corrections along the swing, across it and in height may ask for. The tail rotor pushes sideways hard at tick-tock collective, about 4m/s/s on a 700-size heli in RealFlight.
  // @Units: m/s/s
  // @Range: 0 6
  // @User: Advanced
--]]
local TTOC_ACC_MAX = bind_add_param('ACC_MAX', 13, 6)

--[[
  // @Param: TTOC_ABORT_ANG
  // @DisplayName: Tick-tock abort angle
  // @Description: If the heli is further than this from the planned attitude for half a second, the script stops the tick-tock and levels the heli
  // @Units: deg
  // @Range: 10 60
  // @User: Advanced
--]]
local TTOC_ABORT_ANG = bind_add_param('ABORT_ANG', 14, 30)

--[[
  // @Param: TTOC_ALT_MIN
  // @DisplayName: Tick-tock minimum height
  // @Description: The tick-tock will not start below this height above home
  // @Units: m
  // @Range: 0 200
  // @User: Standard
--]]
local TTOC_ALT_MIN = bind_add_param('ALT_MIN', 15, 15)

--[[
  // @Param: TTOC_STK_ABORT
  // @DisplayName: Tick-tock stick abort level
  // @Description: Roll, pitch or yaw stick input beyond this fraction of full throw hands control back to the pilot's mode
  // @Range: 0.05 1
  // @User: Standard
--]]
local TTOC_STK_ABORT = bind_add_param('STK_ABORT', 16, 0.3)

--[[
  // @Param: TTOC_ALT_LOSS
  // @DisplayName: Tick-tock height loss limit
  // @Description: If the heli sinks this far below the height it started at, the script stops the tick-tock and levels the heli. A rotor that cannot make the thrust the swing needs, for instance because the swing is too slow, otherwise sinks all the way down.
  // @Units: m
  // @Range: 2 50
  // @User: Standard
--]]
local TTOC_ALT_LOSS = bind_add_param('ALT_LOSS', 17, 10)

-- bound before anything else so a non-heli build fails here, at load
local H_COL_ANG_MIN = bind_param('H_COL_ANG_MIN')
local H_COL_ANG_MAX = bind_param('H_COL_ANG_MAX')
local H_COL_ZERO_THRST = bind_param('H_COL_ZERO_THRST')
local H_COL_HOVER = bind_param('H_COL_HOVER')
local ATC_ANGLE_BOOST = bind_param('ATC_ANGLE_BOOST')
local GUID_TIMEOUT = bind_param('GUID_TIMEOUT')
local ATC_ACC_P_MAX = bind_param('ATC_ACC_P_MAX')
local ATC_ACC_R_MAX = bind_param('ATC_ACC_R_MAX')
local ATC_RATE_P_MAX = bind_param('ATC_RATE_P_MAX')
local ATC_RATE_R_MAX = bind_param('ATC_RATE_R_MAX')
local ACRO_RP_RATE_TC = bind_param('ACRO_RP_RATE_TC')
local SCHED_LOOP_RATE = bind_param('SCHED_LOOP_RATE')

local rc_roll = assert(rc:get_channel(math.floor(assert(param:get('RCMAP_ROLL')))))
local rc_pitch = assert(rc:get_channel(math.floor(assert(param:get('RCMAP_PITCH')))))
local rc_yaw = assert(rc:get_channel(math.floor(assert(param:get('RCMAP_YAW')))))

local STATE = {IDLE=0, ENTRY=1, SWING=2, EXIT=3, RECOVER=4, SETTLE=5}
local state = STATE.IDLE
-- start true so a switch already high at boot does not start a manoeuvre
local last_switch_high = true
-- the manoeuvre being flown, nil when idle
local m = nil

-- constrain a value between limits
local function constrain(v, lo, hi)
    return math.max(lo, math.min(hi, v))
end

-- Hamilton product of quaternions stored as {w, x, y, z}
local function q_mul(a, b)
    return {
        a[1]*b[1] - a[2]*b[2] - a[3]*b[3] - a[4]*b[4],
        a[1]*b[2] + a[2]*b[1] + a[3]*b[4] - a[4]*b[3],
        a[1]*b[3] - a[2]*b[4] + a[3]*b[1] + a[4]*b[2],
        a[1]*b[4] + a[2]*b[3] - a[3]*b[2] + a[4]*b[1],
    }
end

-- conjugate (inverse) of a unit quaternion
local function q_conj(q)
    return {q[1], -q[2], -q[3], -q[4]}
end

-- rotation by angle (radians) about body axis 1=x, 2=y, 3=z
local function q_axis(axis, angle)
    local q = {math.cos(0.5*angle), 0, 0, 0}
    q[axis+1] = math.sin(0.5*angle)
    return q
end

-- rotate a vector {x, y, z} by a unit quaternion
local function q_rotate(q, v)
    local w, x, y, z = q[1], q[2], q[3], q[4]
    local tx = 2*(y*v[3] - z*v[2])
    local ty = 2*(z*v[1] - x*v[3])
    local tz = 2*(x*v[2] - y*v[1])
    return {
        v[1] + w*tx + y*tz - z*ty,
        v[2] + w*ty + z*tx - x*tz,
        v[3] + w*tz + x*ty - y*tx,
    }
end

-- rotation vector (axis times angle in radians) of the shortest rotation a quaternion describes
local function q_to_rotvec(q)
    local w, x, y, z = q[1], q[2], q[3], q[4]
    if w < 0 then
        w, x, y, z = -w, -x, -y, -z
    end
    local s = math.sqrt(x*x + y*y + z*z)
    if s < 1.0e-9 then
        return {2*x, 2*y, 2*z}
    end
    local k = 2*math.atan(s, w) / s
    return {k*x, k*y, k*z}
end

-- vehicle attitude as a body-to-NED quaternion, or nil
local function get_q_act()
    local q = ahrs:get_quaternion()
    if not q then
        return nil
    end
    return {q:q1(), q:q2(), q:q3(), q:q4()}
end

-- angle of a cosine-shaped move between two angles, starting and ending at rest
local function cos_segment(seg, tau)
    local half = 0.5*(seg.to - seg.from)
    local k = math.pi / seg.dur
    local x = k*tau
    return seg.from + half*(1 - math.cos(x)), half*k*math.sin(x), half*k*k*math.cos(x)
end

-- duration of a cosine-shaped move of delta radians peaking at accel rad/s/s
local function cos_segment_duration(delta, accel)
    return math.pi * math.sqrt(math.abs(delta) / (2*accel))
end

--[[
   angle, rate and angular acceleration at time t along a list of cosine
   segments, and whether the list has finished
--]]
local function profile(segs, t)
    for _, seg in ipairs(segs) do
        if seg.dur > 0 and t < seg.dur then
            local th, thd, thdd = cos_segment(seg, t)
            return th, thd, thdd, false
        end
        t = t - seg.dur
    end
    return segs[#segs].to, 0, 0, true
end

--[[
   Rotor thrust as a multiple of hover thrust when the thrust axis has
   component c along the wanted direction and the aim is K times weight
   along it. Outside the reversal band the thrust is K/c, limited by the
   collective range. Inside it the thrust ramps through zero so the
   collective reverses smoothly as the disc passes vertical.
--]]
local function thrust_ratio(c, K, s_rev, tw_lim)
    if math.abs(c) >= s_rev then
        return constrain(K / c, -tw_lim, tw_lim)
    end
    return math.min(K / s_rev, tw_lim) * c / s_rev
end

--[[
   Sideways velocity change over a list of segments flown holding height, in
   units of g times seconds, positive towards the side the disc tilts to at
   positive angles. Holding height while tilted by th pushes sideways at
   g*tan(th) until the collective runs out, and past vertical the reversed
   collective pushes back.
--]]
local function profile_impulse(segs, s_rev, tw_lim)
    local N = 48
    local sum = 0
    for _, seg in ipairs(segs) do
        local h = seg.dur / N
        for i = 0, N-1 do
            local th = cos_segment(seg, (i + 0.5)*h)
            sum = sum + thrust_ratio(math.cos(th), 1, s_rev, tw_lim)*math.sin(th)*h
        end
    end
    return sum
end

-- thrust axis components along the wanted direction at evenly spaced phases of a quarter swing of +-alpha
local function swing_samples(alpha)
    local N = 16
    local c = {}
    for i = 0, N-1 do
        c[i+1] = math.sin(alpha * math.cos((i + 0.5) * 0.5*math.pi / N))
    end
    return c
end

--[[
   Mean thrust along the wanted direction over a full swing, as a multiple
   of weight, when aiming for K, and its slope with K. Near vertical the
   collective runs out, so the mean is less than K.
--]]
local function mean_support(samples, K, s_rev, tw_lim)
    local sum = 0
    local slope = 0
    for _, c in ipairs(samples) do
        if c >= s_rev then
            if K < tw_lim*c then
                sum = sum + K
                slope = slope + 1
            else
                sum = sum + tw_lim*c
            end
        elseif K < tw_lim*s_rev then
            sum = sum + K*c*c / (s_rev*s_rev)
            slope = slope + c*c / (s_rev*s_rev)
        else
            sum = sum + tw_lim*c*c / s_rev
        end
    end
    local n = #samples
    return sum / n, slope / n
end

--[[
   One Newton step towards the aim K that gives a mean thrust of r times
   weight over the swing. r changes slowly, so a step each update tracks it.
--]]
local function track_aim(r)
    local F, slope = mean_support(m.samples, m.K, m.s_rev, m.tw_lim)
    m.K = constrain(m.K + (r - F) / math.max(slope, 0.1), math.min(r, m.tw_lim), m.tw_lim)
end

-- put back the parameters changed for the manoeuvre
local function restore_params()
    if m and m.params_saved then
        ATC_ANGLE_BOOST:set(m.boost_orig)
        GUID_TIMEOUT:set(m.timeout_orig)
        m.params_saved = false
    end
end

-- stop without changing mode, for when the mode has already changed or the vehicle disarmed
local function release(reason)
    restore_params()
    gcs:send_text(MAV_SEVERITY.WARNING, "TTOC: stopped, " .. reason)
    state = STATE.IDLE
    m = nil
end

-- stop and return to the mode the pilot was flying
local function hand_back(reason)
    restore_params()
    if not vehicle:set_mode(m.prev_mode) then
        gcs:send_text(MAV_SEVERITY.WARNING, "TTOC: could not restore mode")
    end
    gcs:send_text(MAV_SEVERITY.INFO, "TTOC: done, " .. reason)
    state = STATE.IDLE
    m = nil
end

-- abandon the tick-tock and level the heli before handing back
local function start_recover(reason)
    restore_params()
    gcs:send_text(MAV_SEVERITY.WARNING, "TTOC: recovering, " .. reason)
    state = STATE.RECOVER
    m.t = 0
end

--[[
   level the heli after the exit before handing back. The exit ends on plan,
   but the heli is still tilted and turning, and a position-holding mode
   handed that much rotation overshoots: LOITER in RealFlight took a heli
   handed back at 30 deg and 100 deg/s to an 87 deg tilt.
--]]
local function start_settle()
    restore_params()
    state = STATE.SETTLE
    m.t = 0
end

-- true if the pilot is moving the roll, pitch or yaw stick
local function pilot_override()
    local lim = TTOC_STK_ABORT:get()
    return math.abs(rc_roll:norm_input_dz()) > lim or
        math.abs(rc_pitch:norm_input_dz()) > lim or
        math.abs(rc_yaw:norm_input_dz()) > lim
end

--[[
   check the manoeuvre can be flown and plan it. Returns the plan, or nil
   and the reason it cannot be flown.
--]]
local function plan()
    if not arming:is_armed() or motors:get_spool_state() ~= SPOOL_THROTTLE_UNLIMITED or
        not vehicle:get_likely_flying() then
        return nil, "not flying"
    end
    local prev_mode = vehicle:get_mode()
    if prev_mode == MODE_GUIDED then
        return nil, "start from a pilot mode, not Guided"
    end
    if not ahrs:healthy() then
        return nil, "AHRS not healthy"
    end
    local pos = ahrs:get_relative_position_NED_origin()
    if not get_q_act() or not pos or not ahrs:get_velocity_NED() then
        return nil, "no position estimate"
    end
    local height = -ahrs:get_relative_position_D_home()
    if height < TTOC_ALT_MIN:get() then
        return nil, string.format("below TTOC_ALT_MIN %.0fm", TTOC_ALT_MIN:get())
    end

    local alpha = math.rad(constrain(TTOC_ANGLE:get(), 20, 80))
    local period = constrain(TTOC_PERIOD:get(), 0.3, 5)
    local omega = 2*math.pi / period
    local roll_axis = TTOC_AXIS:get() == 1

    -- collective positions as fractions of H_COL_MIN to H_COL_MAX, as AP_MotorsHeli works them out
    local ang_min = H_COL_ANG_MIN:get()
    local ang_max = H_COL_ANG_MAX:get()
    if ang_max - ang_min < 1 then
        return nil, "bad H_COL_ANG_MIN/MAX"
    end
    local mid = (constrain(H_COL_ZERO_THRST:get(), ang_min, ang_max) - ang_min) / (ang_max - ang_min)
    local hover = constrain(H_COL_HOVER:get(), COL_HOVER_MIN, COL_HOVER_MAX)
    if hover - mid < 0.05 then
        return nil, "H_COL_HOVER too close to zero thrust"
    end
    -- largest thrust available both ways, as a multiple of hover thrust
    local tw_lim = math.min(1 - mid, mid) / (hover - mid)
    -- holding height at the ends of the swing needs 1/sin(alpha) times hover thrust
    if tw_lim < 1 / math.sin(alpha) then
        return nil, string.format("needs %.1fx hover thrust both ways, have %.1fx: check H_COL_ANG_MIN", 1 / math.sin(alpha), tw_lim)
    end

    local acc_max_degss = roll_axis and ATC_ACC_R_MAX:get() or ATC_ACC_P_MAX:get()
    local swing_accel = alpha*omega*omega
    if acc_max_degss > 0 and swing_accel > ACCEL_MARGIN*math.rad(acc_max_degss) then
        local min_period = 2*math.pi*math.sqrt(alpha / (ACCEL_MARGIN*math.rad(acc_max_degss)))
        return nil, string.format("TTOC_PERIOD under %.2fs needs more ATC_ACC_%s_MAX", min_period, roll_axis and "R" or "P")
    end
    local rate_max_degs = roll_axis and ATC_RATE_R_MAX:get() or ATC_RATE_P_MAX:get()
    if rate_max_degs > 0 and alpha*omega > math.rad(rate_max_degs) then
        return nil, string.format("swing needs %.0fdeg/s, over ATC_RATE_%s_MAX", math.deg(alpha*omega), roll_axis and "R" or "P")
    end

    local s_rev = math.sin(math.rad(constrain(TTOC_REV_ANG:get(), 1, 20)))
    local samples = swing_samples(alpha)
    local support_max = mean_support(samples, tw_lim, s_rev, tw_lim)
    if support_max < MIN_SUPPORT_RATIO then
        return nil, string.format("swing holds only %.2fx weight: raise TTOC_ANGLE or collective range", support_max)
    end
    -- aim for holding height with the starting efficiency, so the swing starts with the aim it will need
    local aim = 1 / EFFICIENCY_START
    local K = 1
    for _ = 1, 4 do
        local F, slope = mean_support(samples, K, s_rev, tw_lim)
        K = constrain(K + (aim - F) / math.max(slope, 0.1), math.min(aim, tw_lim), tw_lim)
    end

    --[[
       The entry swings in one move from upright to the far end of the swing,
       and the exit back, so the swing starts and ends by turning round as it
       does at every other end. Stopping at the near end and setting off again
       the same way asks for a step of twice the swing's peak angular
       acceleration, which the heli overshoots. The entry's acceleration
       makes its peak rate the swing's.
    --]]
    local far = 0.5*math.pi + alpha
    local join_accel = math.min(swing_accel, 2*(alpha*omega)^2 / far)
    local join_dur = cos_segment_duration(far, join_accel)
    local entry = {{from=0, to=far, dur=join_dur}}
    local exit = {{from=far, to=0, dur=join_dur}}
    --[[
       The entry and exit each push the heli further one way than the other,
       so the manoeuvre is tilted by the acceleration that cancels that over
       each of them. Pitching nose up tilts the thrust backwards, and rolling
       right tilts it right.
    --]]
    local dir = TTOC_DIR:get() < 0 and -1 or 1
    local push_sign = (roll_axis and 1 or -1) * dir
    -- enough to hold height at the far end, where the swing takes over
    local join_tw_lim = math.min(tw_lim, math.max(JOIN_THRUST_MAX, 1 / math.sin(alpha)))
    local join_accel_along = -push_sign*GRAVITY_MSS*profile_impulse(entry, s_rev, join_tw_lim) / join_dur

    -- the rate input is shaped with ACRO_RP_RATE_TC, or 10 loops when that is zero
    local lead = ACRO_RP_RATE_TC:get()
    if lead <= 0 then
        lead = 10 / math.max(SCHED_LOOP_RATE:get(), 50)
    end

    local yaw = ahrs:get_yaw_rad()
    -- horizontal directions of the body's forward and right axes at the start
    local forward = {math.cos(yaw), math.sin(yaw)}
    local right = {-math.sin(yaw), math.cos(yaw)}
    return {
        prev_mode = prev_mode,
        yaw = yaw,
        q_yaw = q_axis(3, yaw),
        axis = roll_axis and 1 or 2,
        -- the swing moves the heli across the axis it rocks about, and not along it
        along = roll_axis and right or forward,
        cross = roll_axis and forward or right,
        dir = dir,
        alpha = alpha,
        omega = omega,
        period = period,
        cycles = math.floor(math.max(TTOC_CYCLES:get(), 0)),
        mid = mid,
        hover = hover,
        tw_lim = tw_lim,
        join_tw_lim = join_tw_lim,
        s_rev = s_rev,
        samples = samples,
        K = K,
        eta = EFFICIENCY_START,
        support = 0,
        entry = entry,
        exit = exit,
        join_accel_along = join_accel_along,
        lead = lead,
        p_target = {pos:x(), pos:y(), pos:z()},
        integ = {0, 0, 0},
        hist = {},
        hist_first = 1,
        hist_last = 0,
        win = {1, 1},
        flown = 0,
        t = 0,
        abort_t = 0,
        stop = false,
        params_saved = false,
    }
end

--[[
   record the position along the swing for the one-period averages, with the
   vertical velocity and the running total of commanded vertical thrust that
   the efficiency is measured from
--]]
local function push_hist(pos, vel, dt)
    local h = m.hist
    local p = {pos:x(), pos:y(), pos:z()}
    local last = h[m.hist_last]
    local s
    local cs = 0
    if last then
        s = {
            last.s[1] + 0.5*(last.p[1] + p[1])*dt,
            last.s[2] + 0.5*(last.p[2] + p[2])*dt,
            last.s[3] + 0.5*(last.p[3] + p[3])*dt,
        }
        cs = last.cs + m.support*dt
    else
        s = {0, 0, 0}
    end
    m.hist_last = m.hist_last + 1
    h[m.hist_last] = {t=m.t, p=p, s=s, vz=vel:z(), cs=cs}
end

--[[
   Position and velocity averaged over the last W seconds, or nil until that
   much has been recorded. which picks the window, whose start index is kept
   so it only moves forward. Averaging over a whole period removes the
   back-and-forth the swing itself causes, leaving the drift to correct.
   Also returns the mean vertical specific force measured over the window and
   the mean vertical thrust commanded, both as multiples of weight.
--]]
local function window_average(W, which)
    local h = m.hist
    local t_old = m.t - W
    local i = math.max(m.win[which], m.hist_first)
    if m.hist_last - i < 1 or h[i].t > t_old then
        return nil
    end
    while h[i+1].t <= t_old do
        i = i + 1
    end
    m.win[which] = i
    local a = h[i]
    local b = h[i+1]
    local f = (t_old - a.t) / (b.t - a.t)
    local last = h[m.hist_last]
    local pavg = {}
    local vavg = {}
    for k = 1, 3 do
        local p_old = a.p[k] + f*(b.p[k] - a.p[k])
        local s_old = a.s[k] + f*(b.s[k] - a.s[k])
        pavg[k] = (last.s[k] - s_old) / W
        vavg[k] = (last.p[k] - p_old) / W
    end
    local vz_old = a.vz + f*(b.vz - a.vz)
    local cs_old = a.cs + f*(b.cs - a.cs)
    local force_up = 1 - (last.vz - vz_old) / (W*GRAVITY_MSS)
    return pavg, vavg, force_up, (last.cs - cs_old) / W
end

-- drop history older than the full-period window needs
local function prune_hist()
    for i = m.hist_first, m.win[1] - 1 do
        m.hist[i] = nil
    end
    m.hist_first = math.max(m.hist_first, m.win[1])
end

-- PI correction on one axis (1 along the swing, 2 across it, 3 down) from its position error and velocity, returning acceleration
local function correct_axis(k, pos_err, v, dt, acc_max)
    local vel_err = TTOC_POS_P:get()*pos_err - v
    m.integ[k] = constrain(m.integ[k] + TTOC_VEL_I:get()*vel_err*dt, -acc_max, acc_max)
    return constrain(TTOC_VEL_P:get()*vel_err + m.integ[k], -acc_max, acc_max)
end

-- component of a horizontal north, east vector along a unit horizontal direction
local function horiz_dot(n, e, dir)
    return n*dir[1] + e*dir[2]
end

--[[
   Acceleration to correct position and height drift, NED.

   The swing pushes the heli back and forth along its plane and bounces it at
   every crossing, so along the swing position and velocity are averaged over
   a full period, and height over half a period. Nothing oscillates across the
   swing plane, where the tail rotor pushes steadily sideways, so that drift
   is corrected from the current position all through the manoeuvre. The
   entry and exit do not oscillate either, so their height correction uses
   the current height. Along the swing they hold the correction and add the
   planned tilt that cancels their own push.
--]]
local function correction_accel(pos, vel, dt)
    local acc_max = TTOC_ACC_MAX:get()
    local along = m.along
    local cross = m.cross
    local a_along = m.integ[1]
    local a_down = m.integ[3]
    local a_cross = correct_axis(2, horiz_dot(m.p_target[1] - pos:x(), m.p_target[2] - pos:y(), cross),
                                 horiz_dot(vel:x(), vel:y(), cross), dt, acc_max)
    if state == STATE.SWING then
        push_hist(pos, vel, dt)
        local pz, vz, force_up, support = window_average(0.5*m.period, 2)
        if pz then
            a_down = correct_axis(3, m.p_target[3] - pz[3], vz[3], dt, acc_max)
            -- learn the share of the commanded thrust the rotor really delivers over a half period
            if support > 0.2 then
                local eta = constrain(force_up / support, EFFICIENCY_MIN, EFFICIENCY_MAX)
                m.eta = m.eta + (eta - m.eta) * math.min(1, dt / (0.5*m.period))
            end
        end
        local pxy, vxy = window_average(m.period, 1)
        if pxy then
            prune_hist()
            a_along = correct_axis(1, horiz_dot(m.p_target[1] - pxy[1], m.p_target[2] - pxy[2], along),
                                   horiz_dot(vxy[1], vxy[2], along), dt, acc_max)
        end
    else
        a_down = correct_axis(3, m.p_target[3] - pos:z(), vel:z(), dt, acc_max)
        a_along = a_along + m.join_accel_along
    end
    return {along[1]*a_along + cross[1]*a_cross, along[2]*a_along + cross[2]*a_cross, a_down}
end

--[[
   Fly one step of the planned swing angle th (rad, rate thd, acceleration
   thdd), measured from upright towards the TTOC_DIR side.
--]]
local function fly(th, thd, thdd, dt)
    local q_act = get_q_act()
    local pos = ahrs:get_relative_position_NED_origin()
    local vel = ahrs:get_velocity_NED()
    if not q_act or not pos or not vel then
        start_recover("no position estimate")
        return
    end
    local sunk = pos:z() - m.p_target[3]
    if sunk > TTOC_ALT_LOSS:get() then
        start_recover(string.format("sank %.0fm", sunk))
        return
    end

    --[[
       Specific force the rotor must make on average. The whole manoeuvre is
       tilted so its vertical lies along it, which is how the swing corrects
       drift: the sideways pushes still cancel over a period, but about the
       tilted direction.
    --]]
    local acc = correction_accel(pos, vel, dt)
    local A = {acc[1], acc[2], acc[3] - GRAVITY_MSS}
    local A_len = math.sqrt(A[1]*A[1] + A[2]*A[2] + A[3]*A[3])
    local d = {A[1] / A_len, A[2] / A_len, A[3] / A_len}
    local r = A_len / GRAVITY_MSS

    -- shortest rotation from up {0, 0, -1} to d
    local q_tilt = {1 - d[3], d[2], -d[1], 0}
    local n = math.sqrt(q_tilt[1]*q_tilt[1] + q_tilt[2]*q_tilt[2] + q_tilt[3]*q_tilt[3])
    q_tilt = {q_tilt[1] / n, q_tilt[2] / n, q_tilt[3] / n, 0}

    local q_des = q_mul(q_mul(q_tilt, m.q_yaw), q_axis(m.axis, m.dir*th))
    local q_err = q_mul(q_conj(q_act), q_des)
    local e = q_to_rotvec(q_err)
    local err_deg = math.deg(math.sqrt(e[1]*e[1] + e[2]*e[2] + e[3]*e[3]))

    --[[
       Body rates: the planned swing rate, led by the rate shaping time
       constant, plus a correction towards the planned attitude. The planned
       rate is about the planned body axes, so it is turned into the actual
       body axes.
    --]]
    local w_des = {0, 0, 0}
    w_des[m.axis] = m.dir*(thd + m.lead*thdd)
    local w = q_rotate(q_err, w_des)
    local kp = TTOC_ATT_P:get()
    for i = 1, 3 do
        w[i] = w[i] + kp*e[i]
    end

    -- collective from where the thrust axis actually points, so it reverses as the disc passes vertical
    local u = q_rotate(q_act, {0, 0, -1})
    local c = u[1]*d[1] + u[2]*d[2] + u[3]*d[3]
    local K = r
    local tw_lim = m.join_tw_lim
    if state == STATE.SWING then
        track_aim(r / m.eta)
        K = m.K
        tw_lim = m.tw_lim
    end
    local tw = thrust_ratio(c, K, m.s_rev, tw_lim)
    local thr = constrain(m.mid + (m.hover - m.mid)*tw, 0, 1)
    m.support = c*tw

    if not vehicle:set_target_rate_and_throttle(math.deg(w[1]), math.deg(w[2]), math.deg(w[3]), thr) then
        release("Guided rejected the command")
        return
    end

    logger:write('TTOC', 'St,Ang,Err,C,TW,Thr,R,K,Eta,AN,AE,AD', 'Bfffffffffff',
                 state, math.deg(m.dir*th), err_deg, c, tw, thr, r, K, m.eta, acc[1], acc[2], acc[3])

    if err_deg > TTOC_ABORT_ANG:get() then
        m.abort_t = m.abort_t + dt
        if m.abort_t > ABORT_TIME_S then
            start_recover(string.format("%.0fdeg off the planned attitude", err_deg))
        end
    else
        m.abort_t = 0
    end
end

-- level the heli and hold height, then hand back
-- level the heli and hold height, then hand back once it is level and has stopped turning
local function run_recover()
    vehicle:set_target_angle_and_climbrate(0, 0, math.deg(m.yaw), 0, false, 0)
    local q = get_q_act()
    local gyro = ahrs:get_gyro()
    local settled = false
    if q and gyro then
        local u = q_rotate(q, {0, 0, -1})
        settled = math.deg(math.acos(constrain(-u[3], -1, 1))) < RECOVER_LEVEL_DEG and
            math.deg(gyro:length()) < RECOVER_RATE_DEGS
    end
    if settled or m.t > RECOVER_TIMEOUT_S then
        if state == STATE.SETTLE then
            hand_back(string.format("%d cycles", m.flown))
        else
            hand_back("recovered")
        end
    end
end

-- try to start the tick-tock
local function start()
    local plan_or_nil, reason = plan()
    if not plan_or_nil then
        gcs:send_text(MAV_SEVERITY.WARNING, "TTOC: " .. reason)
        return
    end
    m = plan_or_nil

    -- heli angle boost rescales and flips the collective, which would undo the collective worked out here
    m.boost_orig = ATC_ANGLE_BOOST:get()
    m.timeout_orig = GUID_TIMEOUT:get()
    m.params_saved = true
    ATC_ANGLE_BOOST:set(0)
    GUID_TIMEOUT:set(MANOEUVRE_GUID_TIMEOUT_S)

    if not vehicle:set_mode(MODE_GUIDED) then
        restore_params()
        gcs:send_text(MAV_SEVERITY.WARNING, "TTOC: could not enter Guided")
        m = nil
        return
    end
    m.last_us = micros()
    state = STATE.ENTRY
    gcs:send_text(MAV_SEVERITY.INFO, string.format("TTOC: start, %.0fdeg swing, %.2fs period", math.deg(m.alpha), m.period))
end

-- step the manoeuvre
local function run(switch_high)
    local now = micros()
    local dt = (now - m.last_us):tofloat() * 1.0e-6
    m.last_us = now
    m.t = m.t + dt

    if not arming:is_armed() then
        release("disarmed")
        return
    end
    if vehicle:get_mode() ~= MODE_GUIDED then
        release("mode changed")
        return
    end
    if state ~= STATE.RECOVER and pilot_override() then
        hand_back("pilot took over")
        return
    end
    if not switch_high then
        m.stop = true
    end

    if state == STATE.RECOVER or state == STATE.SETTLE then
        run_recover()
        return
    end

    local th, thd, thdd, done
    if state == STATE.ENTRY then
        th, thd, thdd, done = profile(m.entry, m.t)
        if done then
            if m.stop then
                state = STATE.EXIT
            else
                state = STATE.SWING
            end
            m.t = 0
        end
    elseif state == STATE.SWING then
        local cycles_done = math.floor(m.t / m.period)
        if cycles_done > 0 and (m.stop or (m.cycles > 0 and cycles_done >= m.cycles)) then
            state = STATE.EXIT
            m.flown = cycles_done
            m.t = m.t - cycles_done*m.period
            -- the swing's extra collective made up for losses the exit does not have
            m.integ[3] = 0
        end
    end

    if state == STATE.SWING then
        -- starts and ends at the far end, where the entry stops and the exit starts
        local x = m.omega*m.t
        th = 0.5*math.pi + m.alpha*math.cos(x)
        thd = -m.alpha*m.omega*math.sin(x)
        thdd = -m.alpha*m.omega*m.omega*math.cos(x)
    elseif state == STATE.EXIT then
        th, thd, thdd, done = profile(m.exit, m.t)
        if done then
            start_settle()
            return
        end
    end
    fly(th, thd, thdd, dt)
end

-- main update
local function update()
    local switch_high = rc:get_aux_cached(math.floor(TTOC_AUX_FUNC:get())) == SWITCH_HIGH
    if TTOC_ENABLE:get() < 1 then
        if state ~= STATE.IDLE then
            hand_back("disabled")
        end
    elseif state == STATE.IDLE then
        if switch_high and not last_switch_high then
            start()
        end
    else
        run(switch_high)
    end
    last_switch_high = switch_high
end

-- wrapper that puts things back and hands control back if the script errors
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "TTOC: internal error: " .. err)
        if m then
            restore_params()
            if state ~= STATE.IDLE then
                vehicle:set_mode(m.prev_mode)
            end
        end
        state = STATE.IDLE
        m = nil
        return protected_wrapper, 1000
    end
    return protected_wrapper, UPDATE_MS
end

gcs:send_text(MAV_SEVERITY.INFO, "TTOC: heli tick-tock loaded")

return protected_wrapper()

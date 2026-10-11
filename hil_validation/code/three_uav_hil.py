"""Three-UAV PX4 SITL tracker with HIL-equivalent stochastic wind commands.

PX4 closes attitude/rate loops and CopterSim supplies vehicle physics.  UE4 is
not launched; MATLAB consumes this process's telemetry CSV for live display.
"""
import argparse
import csv
import json
import math
import os
import random
import sys
import time

PSP_PATH = os.environ.get("PSP_PATH", r"C:\PX4PSP")
SDK_PATH = os.path.join(PSP_PATH, "RflySimAPIs", "RflySimSDK", "ctrl")
if SDK_PATH not in sys.path:
    sys.path.insert(0, SDK_PATH)
import PX4MavCtrlV4 as PX4MavCtrl

G = 9.81


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def limit(v, maximum):
    magnitude = norm(v)
    return v if magnitude <= maximum else [x * maximum / magnitude for x in v]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def quaternion_from_acceleration(acc, yaw):
    b3 = [-acc[0], -acc[1], G - acc[2]]
    b3 = [x / max(norm(b3), 1e-9) for x in b3]
    b1_yaw = [math.cos(yaw), math.sin(yaw), 0.0]
    b2 = [b3[1] * b1_yaw[2] - b3[2] * b1_yaw[1],
          b3[2] * b1_yaw[0] - b3[0] * b1_yaw[2],
          b3[0] * b1_yaw[1] - b3[1] * b1_yaw[0]]
    b2 = [0.0, 1.0, 0.0] if norm(b2) < 1e-6 else [x / norm(b2) for x in b2]
    b1 = [b2[1] * b3[2] - b2[2] * b3[1],
          b2[2] * b3[0] - b2[0] * b3[2],
          b2[0] * b3[1] - b2[1] * b3[0]]
    r00, r01, r02 = b1[0], b2[0], b3[0]
    r10, r11, r12 = b1[1], b2[1], b3[1]
    r20, r21, r22 = b1[2], b2[2], b3[2]
    trace = r00 + r11 + r22
    s = math.sqrt(max(trace + 1.0, 1e-9)) * 2.0
    q = [0.25 * s, (r21-r12)/s, (r02-r20)/s, (r10-r01)/s]
    return [x / norm(q) for x in q]


class StochasticWind:
    """Same OU turbulence plus cosine-gust model used by the USB HIL run."""
    def __init__(self, args, count):
        self.args, self.rng, self.count = args, random.Random(args.wind_seed), count
        self.last_t = None
        self.turbulence = [[0.0, 0.0, 0.0] for _ in range(count)]
        self.gusts = [[] for _ in range(count)]

    def sample(self, now):
        dt = 1.0 / self.args.rate if self.last_t is None else clamp(now-self.last_t, 1e-3, 0.2)
        self.last_t = now
        for i in range(self.count):
            for axis in range(3):
                tau = self.args.turb_tau_z if axis == 2 else self.args.turb_tau
                decay = math.exp(-dt / max(tau, 1e-3))
                sigma = self.args.turb_sigma * (self.args.wind_z_scale if axis == 2 else 1.0)
                self.turbulence[i][axis] = decay*self.turbulence[i][axis] + sigma*math.sqrt(1-decay*decay)*self.rng.gauss(0, 1)
        probability = 1.0-math.exp(-self.args.gust_rate*dt)
        for i in range(self.count):
            if self.rng.random() < probability:
                theta = self.rng.uniform(0, 2*math.pi)
                direction = [math.cos(theta), math.sin(theta), self.rng.uniform(-self.args.gust_z_scale, self.args.gust_z_scale)]
                direction = [x / norm(direction) for x in direction]
                self.gusts[i].append((now, self.rng.uniform(self.args.gust_duration_min, self.args.gust_duration_max),
                                      self.rng.uniform(self.args.gust_amp_min, self.args.gust_amp_max), direction))
        result = []
        for i in range(self.count):
            wind, active = list(self.turbulence[i]), []
            for started, duration, amplitude, direction in self.gusts[i]:
                age = now-started
                if age <= duration:
                    envelope = 0.5*(1-math.cos(2*math.pi*max(age, 0)/duration))
                    wind = add(wind, [amplitude*envelope*x for x in direction])
                    active.append((started, duration, amplitude, direction))
            self.gusts[i] = active
            result.append(limit(wind, self.args.wind_limit))
        return result


def parse_args():
    parser = argparse.ArgumentParser(description="Zhashui three-UAV PX4 SITL, displayed live by MATLAB")
    parser.add_argument("--mission", required=True)
    parser.add_argument("--log", required=True)
    parser.add_argument("--summary", required=True)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--kp", type=float, default=0.55)
    parser.add_argument("--kd", type=float, default=1.20)
    parser.add_argument("--max-acc", type=float, default=3.5)
    parser.add_argument("--uav2-max-acc", type=float, default=5.5,
                        help="UAV 2 compensation limit for its slower HIL response.")
    parser.add_argument("--lookahead-m", type=float, default=25.0)
    # The exported terrain route is sampled at about 9 m.  A 45 m capture
    # radius prevents a vehicle that cuts a corner from becoming pinned to a
    # slightly missed, already-passed sample point.
    parser.add_argument("--waypoint-radius-m", type=float, default=45.0)
    parser.add_argument("--hover-thrust", type=float, default=0.60)
    parser.add_argument("--max-thrust", type=float, default=0.90)
    parser.add_argument("--takeoff-seconds", type=float, default=5.0)
    parser.add_argument("--max-position-error-m", type=float, default=150.0,
                        help="Abort the run before an aircraft can continue a runaway track.")
    parser.add_argument("--wind-seed", type=int, default=20260924)
    parser.add_argument("--wind-limit", type=float, default=1.2)
    parser.add_argument("--wind-z-scale", type=float, default=0.25)
    parser.add_argument("--turb-sigma", type=float, default=0.30)
    parser.add_argument("--turb-tau", type=float, default=1.2)
    parser.add_argument("--turb-tau-z", type=float, default=1.8)
    parser.add_argument("--gust-rate", type=float, default=0.14)
    parser.add_argument("--gust-amp-min", type=float, default=0.40)
    parser.add_argument("--gust-amp-max", type=float, default=0.85)
    parser.add_argument("--gust-duration-min", type=float, default=0.7)
    parser.add_argument("--gust-duration-max", type=float, default=1.8)
    parser.add_argument("--gust-z-scale", type=float, default=0.18)
    return parser.parse_args()


def state(mav):
    return [float(x) for x in mav.truePosNED], [float(x) for x in mav.uavVelNED]


def main():
    args = parse_args()
    with open(args.mission, encoding="utf-8") as f:
        routes = json.load(f)["routes"]
    if len(routes) != 3:
        raise ValueError("Expected exactly three UAV routes.")
    mavs = [PX4MavCtrl.PX4MavCtrler(ID=i, ip="127.0.0.1") for i in range(1, 4)]
    for mav in mavs:
        mav.InitMavLoop(UDPMode=2)
    try:
        until = time.monotonic()+40
        while not all(float(m.uavTimeStmp) > 0 for m in mavs):
            if time.monotonic() > until:
                raise RuntimeError("Timed out waiting for three PX4 SITL telemetry streams.")
            time.sleep(0.1)
        origins = [state(mav)[0] for mav in mavs]
        for mav in mavs:
            mav.SendAttPX4([1, 0, 0, 0], args.hover_thrust, CtrlFlag=2, AltFlg=0)
            mav.initOffboardAtt()
            mav.SendMavArm(True)
        os.makedirs(os.path.dirname(args.log), exist_ok=True)
        wind = StochasticWind(args, 3)
        indices, started, period = [0, 0, 0], time.monotonic(), 1/args.rate
        fault = None
        with open(args.log, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t","uav","waypoint","distanceM","localN","localE","localD","targetN","targetE","targetD","windN","windE","windD","thrust","rollRad","pitchRad","yawRad","rollRateRadS","pitchRateRadS","yawRateRadS"])
            while True:
                tick, elapsed = time.monotonic(), time.monotonic()-started
                wind_cmd, complete = wind.sample(elapsed), 0
                for i, mav in enumerate(mavs):
                    pos, vel = state(mav)
                    local = [pos[k]-origins[i][k] for k in range(3)]
                    attitude = [float(x) for x in mav.uavAngEular]
                    attitude_rate = [float(x) for x in mav.uavAngRate]
                    route = routes[i]["waypointsNED"]
                    if elapsed < args.takeoff_seconds:
                        target = [0.0, 0.0, route[0][2]]
                    elif indices[i] >= len(route):
                        target, complete = route[-1], complete+1
                    else:
                        while indices[i] < len(route)-1 and norm([route[indices[i]][k]-local[k] for k in range(3)]) <= args.waypoint_radius_m:
                            indices[i] += 1
                        if indices[i] == len(route)-1 and norm([route[-1][k]-local[k] for k in range(3)]) <= args.waypoint_radius_m:
                            indices[i] += 1
                            target, complete = route[-1], complete+1
                        else:
                            target = route[indices[i]]
                            for candidate in route[indices[i]:]:
                                if norm([candidate[k]-local[k] for k in range(3)]) >= args.lookahead_m:
                                     target = candidate
                                     break
                    position_error = norm([target[k]-local[k] for k in range(3)])
                    if elapsed >= args.takeoff_seconds and position_error > args.max_position_error_m:
                        fault = (i + 1, position_error, local, target)
                        writer.writerow([elapsed, i+1, indices[i], position_error, *local, *target,
                                         *wind_cmd[i], 0.0, *attitude, *attitude_rate])
                        break
                    nominal = [args.kp*(target[k]-local[k])-args.kd*vel[k] for k in range(3)]
                    max_acc = args.uav2_max_acc if i == 1 else args.max_acc
                    acc = limit(add(nominal, wind_cmd[i]), max_acc)
                    yaw = math.atan2(target[1]-local[1], target[0]-local[0])
                    thrust = clamp(args.hover_thrust*norm([-acc[0],-acc[1],G-acc[2]])/G, 0.12, args.max_thrust)
                    mav.SendAttPX4(quaternion_from_acceleration(acc, yaw), thrust, CtrlFlag=2, AltFlg=0)
                    writer.writerow([elapsed, i+1, indices[i], norm([target[k]-local[k] for k in range(3)]), *local, *target,
                                     *wind_cmd[i], thrust, *attitude, *attitude_rate])
                f.flush()
                if fault is not None:
                    break
                if complete == 3:
                    break
                time.sleep(max(0.0, period-(time.monotonic()-tick)))
        with open(args.summary, "w", encoding="utf-8") as f:
            if fault is None:
                result = {"status":"completed"}
            else:
                uav_id, error_m, local, target = fault
                result = {"status":"aborted_position_error", "uavId":uav_id,
                          "positionErrorM":error_m, "localNED":local, "targetNED":target}
            result.update({"windModel":"OU turbulence plus cosine gusts, HIL-equivalent outer-loop acceleration disturbance",
                           "windSeed":args.wind_seed, "routes":routes})
            json.dump(result, f, indent=2)
    finally:
        for mav in mavs:
            try:
                mav.SendAttPX4([1,0,0,0], 0.0, CtrlFlag=2, AltFlg=0)
                mav.endMavLoop()
            except Exception:
                pass


if __name__ == "__main__":
    main()

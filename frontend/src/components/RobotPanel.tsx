import type { Snapshot } from "../types"

interface Props {
  state: Snapshot
}

function num(v: number | undefined) {
  return Number.isFinite(v) ? Number(v).toFixed(2) : "0.00"
}

export default function RobotPanel({ state }: Props) {
  const commands = state.commands.filter((command) => command.robot_id !== "fleet")
  const fleet = state.commands.find((command) => command.robot_id === "fleet")
  const proximity = state.proximity || []

  return (
    <section className="panel">
      <h2>ROBOT TELEMETRY AND MOTION COMMANDS</h2>
      <div className="tableWrap short">
        <table className="steps">
          <thead>
            <tr>
              <th>ROBOT</th>
              <th>CELL</th>
              <th>HEADING</th>
              <th>SPEED</th>
              <th>BATTERY</th>
              <th>SOURCE</th>
            </tr>
          </thead>
          <tbody>
            {state.robots.map((robot) => (
              <tr key={robot.robot_id}>
                <td>R{robot.robot_id}</td>
                <td>({robot.cell[0]},{robot.cell[1]})</td>
                <td>{num(robot.heading_deg)} deg</td>
                <td>{num(robot.speed_mps)} m/s</td>
                <td>{num(robot.battery_pct)} %</td>
                <td>{robot.source}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <h3>MOTION OUTPUT</h3>
      <div className="tableWrap short">
        <table className="steps">
          <thead>
            <tr>
              <th>ROBOT</th>
              <th>LINEAR</th>
              <th>ANGULAR</th>
              <th>LEFT WHEEL</th>
              <th>RIGHT WHEEL</th>
              <th>BRAKE</th>
              <th>CROSS TRACK</th>
              <th>ARRIVAL</th>
              <th>DRAIN</th>
            </tr>
          </thead>
          <tbody>
            {commands.map((command, index) => (
              <tr key={`${command.robot_id}-${index}`}>
                <td>R{command.robot_id}</td>
                <td>{num(command.linear_mps)}</td>
                <td>{num(command.angular_radps)}</td>
                <td>{num(command.left_wheel_radps)}</td>
                <td>{num(command.right_wheel_radps)}</td>
                <td>{num(command.brake_distance_m)}</td>
                <td>{num(command.cross_track_m)}</td>
                <td>{command.arrival ? "YES" : "NO"}</td>
                <td>{num(command.battery_drain_pct)} %</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <div className="note">
        FLEET UTILIZATION {fleet && fleet.utilization !== undefined ? num(fleet.utilization) : "0.00"} |
        MIN CLEARANCE {num(state.safety.min_clearance_m)} m |
        BRAKE DISTANCE {num(state.safety.brake_distance_m)} m
      </div>
      <div className="note">
        PROXIMITY {proximity.length
          ? proximity.map((item, index) => `${index + 1}:${String(item.level || "unknown")} ${String(item.reason || "")} ${String(item.distance_m || 0)}m`).join(" | ")
          : "CLEAR"}
      </div>
    </section>
  )
}

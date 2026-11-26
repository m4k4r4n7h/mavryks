import type { Snapshot } from "../types"

interface Props {
  state: Snapshot
  status: string
}

function fmt(p: [number, number] | undefined) {
  return p ? `(${p[0]},${p[1]})` : "NONE"
}

function num(v: number | undefined, digits: number) {
  return Number.isFinite(v) ? Number(v).toFixed(digits) : "0"
}

export default function StatusPanel({ state, status }: Props) {
  const safety = state.safety
  const config = state.config
  const vision = state.vision
  const quality = state.quality
  const metrics = state.metrics
  const alerts = safety && safety.alerts.length ? safety.alerts.join(",") : "NORMAL"
  const stopped = Boolean(state.estop || (safety && safety.estop))
  const rows: Array<[string, string]> = [
    ["SOCKET", status],
    ["VISION", vision && vision.online ? `ONLINE ${num(vision.fps, 1)} FPS` : "OFFLINE"],
    ["ESTOP", stopped ? "ACTIVE" : "CLEAR"],
    ["SAFETY", alerts],
    ["GRID", `${state.grid_size || 0} x ${state.grid_size || 0}`],
    ["MOVEMENT", config && config.diagonal ? "8 DIRECTION" : "4 DIRECTION"],
    ["COST MODE", config ? config.cost_mode.toUpperCase() : "EXACT"],
    ["WORKER", fmt(state.workers[0])],
    ["TARGETS", String(state.targets.length)],
    ["OBSTACLES", `${state.dynamic_obstacles.length} LIVE / ${state.static_obstacles.length} STATIC`],
    ["PATH CELLS", String(state.path.length)],
    ["REACHABLE", state.reachable ? "YES" : `NO (${state.reason})`],
    ["COST", num(state.cost, 3)],
    ["DISTANCE", num(state.distance, 3)],
    ["ETA", `${num(state.estimated_time_sec, 2)} s`],
    ["TURNS", String(state.turns)],
    ["CONGESTION AVG/MAX", `${num(state.avg_congestion, 3)} / ${num(state.max_congestion, 3)}`],
    ["SOLVE MS", num(metrics ? metrics.solve_ms : 0, 2)],
    ["NODES EXPANDED", String(metrics ? metrics.nodes_expanded : 0)],
    ["QUALITY", quality ? `${num(quality.score, 2)} (${quality.alerts.join(",") || "OK"})` : "NONE"],
    ["SPEED LIMIT", `${num(state.speed_limit_mps, 2)} m/s`],
    ["UPDATED", state.updated_at ? new Date(state.updated_at * 1000).toLocaleTimeString() : "NONE"],
  ]

  return (
    <section className="panel">
      <h2>OPERATOR STATUS</h2>
      <table className="kv">
        <tbody>
          {rows.map(([key, value]) => (
            <tr key={key}>
              <td>{key}</td>
              <td>{value}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </section>
  )
}

import { useEffect, useState } from "react"
import type { Snapshot } from "../types"

interface Props {
  state: Snapshot
  onEstop: (active: boolean) => void
  onSpeed: (speed: number) => void
  onToggleDiagonal: () => void
  onCostMode: (mode: "exact" | "industrial") => void
  onOptimize: (value: boolean) => void
  onReset: () => void
  onClearTargets: () => void
  onClearZones: () => void
}

export default function ControlPanel({
  state,
  onEstop,
  onSpeed,
  onToggleDiagonal,
  onCostMode,
  onOptimize,
  onReset,
  onClearTargets,
  onClearZones,
}: Props) {
  const [speed, setSpeed] = useState(1.4)
  useEffect(() => setSpeed(state.speed_limit_mps), [state.speed_limit_mps])
  const config = state.config
  const stopped = Boolean(state.estop || state.safety.estop)

  return (
    <section className="panel">
      <h2>SYSTEM CONTROL</h2>
      <div className="row">
        <button className="btn" onClick={() => onEstop(true)}>ESTOP</button>
        <button className="btn" onClick={() => onEstop(false)}>CLEAR ESTOP</button>
        <button className="btn" onClick={onClearTargets}>CLEAR TARGETS</button>
        <button className="btn" onClick={onClearZones}>CLEAR ZONES</button>
        <button className="btn" onClick={onReset}>RESET ALL</button>
      </div>
      <div className="row">
        <button className="btn" onClick={onToggleDiagonal}>
          MOVEMENT {config && config.diagonal ? "8-DIR" : "4-DIR"}
        </button>
        <select
          className="field"
          value={config ? config.cost_mode : "exact"}
          onChange={(event) => onCostMode(event.target.value as "exact" | "industrial")}
        >
          <option value="exact">EXACT COST</option>
          <option value="industrial">INDUSTRIAL COST</option>
        </select>
        <label>
          <input
            type="checkbox"
            checked={Boolean(config && config.optimize_order)}
            onChange={(event) => onOptimize(event.target.checked)}
          />
          OPTIMIZE WAYPOINTS
        </label>
      </div>
      <div className="row">
        <span>SPEED m/s</span>
        <input
          className="field num"
          type="number"
          min={0.1}
          max={3}
          step={0.1}
          value={speed}
          onChange={(event) => setSpeed(Number(event.target.value))}
        />
        <button className="btn" onClick={() => onSpeed(speed)}>SET SPEED</button>
      </div>
      <div className="note">SAFETY STATE: {stopped ? "STOPPED - COMMANDS HELD AT ZERO" : "NORMAL"}</div>
    </section>
  )
}
